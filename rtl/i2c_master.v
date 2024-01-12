`timescale 1ns/1ps
`include "debouncer.v"

module i2c_master (
    input  wire         clk,
    input  wire         asyn_rst_n,

    /* i2c interface */
    input  wire         scl_i,
    output wire         scl_o,
    input  wire         sda_i,
    output wire         sda_o,

    /* host interface */
    input  wire  [7:0]  host_addr_i,
    input  wire  [31:0] host_data_i,
    input  wire  [3:0]  host_cmd_i,
    input  wire         host_sel_i,
    output wire  [31:0] host_data_o,
    output wire         host_busy_o,
    output wire         host_error_o
);
/*
host interface
    host_addr_i     the address of I2C target
    host_data_i     the data write to I2C target, at most 4 bytes
    host_cmd_i      [3:2]   00  I2C write command, and clear host_ready_o and host_error_o
                            01  I2C read command, and clear host_ready_o and host_error_o
                            1X  reserved
                    [1:0]   00  transfer host_data[7:0]
                            01  transfer host_data[15:0]
                            10  transfer host_data[23:0]
                            11  transfer host_data[31:0]
    host_sel_i      The host assert a transaction to I2C master
    host_data_o     the data read from I2C target, at most 4 bytes
    host_busy_o     the last transaction is going on
    host_error_o    the last transaction has error
    --------------------------------------------------
    The host should make sure I2C master is not busy by checking host_busy_o before assert an new transaction.
    The host assert a new transcation by setting host_sel_i HIGH one cycle, and the address/data/cmd also should be ready on the same cycle.
    I2C master will set host_busy_o HIGH as it begin a transaction till it can get the next transaction.
    For read transaction, host can get the valid received data from host_data_o as the I2C master is back to non-busy.
    Sometimes I2C bus has problem, the master will assert host_error_o HIGH to inform the host the last transaction is abnormal finished.
    The host_error_o flag will be cleared when I2C master get the next transaction.
*/

localparam [3:0]
    STATE_BUSY  = 4'h0,
    STATE_IDLE  = 4'h1,
    STATE_START = 4'h2,
    STATE_ADDR  = 4'h3,
    STATE_CMD   = 4'h4,
    STATE_DATA  = 4'h5,
    STATE_ACK_A = 4'h6,
    STATE_ACK_D = 4'h7,
    STATE_STOP  = 4'h8;

localparam [1:0]
    CYC_STATE_LOW_1 = 2'b00,
    CYC_STATE_LOW_2 = 2'b01,
    CYC_STATE_WHIGH = 2'b10,
    CYC_STATE_HIGH  = 2'b11;

localparam [1:0]
    CMD_WRITE   = 2'b00,
    CMD_READ    = 2'b01;

localparam [7:0]
    T_SU_STA    = 8'hA0,    // 160 * 30ns = 4800ns
    T_HD_STA    = 8'h88,    // 136 * 30ns = 4080ns
    T_LOW_1     = 8'h20,    // 32 * 30ns  = 960ns
    T_LOW_2     = 8'h80,    // 128 * 30ns  = 3840ns
    T_HIGH      = 8'h88;    // 136 * 30ns = 4080ns

reg     [6:0]   addr_reg;
reg     [31:0]  tx_data_reg;
reg     [3:0]   cmd_reg;
reg             busy_reg;
reg             error_reg;
wire            cmd_write = (cmd_reg[3:2] == CMD_WRITE);
wire            cmd_read = (cmd_reg[3:2] == CMD_READ);
assign          host_busy_o = busy_reg;
assign          host_error_o = error_reg;
assign          host_data_o = rx_data_reg;

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n) begin
        addr_reg <= 7'b0;
        cmd_reg <= 4'b0;
    end else if (host_sel_i) begin
        addr_reg <= host_addr_i[6:0];
        cmd_reg <= host_cmd_i;
    end
end

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        tx_data_reg <= 32'b0;
    else if (host_sel_i && (host_cmd_i[3:2] == CMD_WRITE))
        tx_data_reg <= host_data_i;
end

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        busy_reg <= 1'b0;
    else if (~busy_reg)
        busy_reg <= (host_sel_i & ((host_cmd_i[3:2] == CMD_READ) || (host_cmd_i[3:2] == CMD_WRITE)));
    else if ((state_reg == STATE_STOP) && cycle_end)
        busy_reg <= 1'b0;
end

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        error_reg <= 1'b0;
    else if (host_sel_i)
        error_reg <= 1'b0;
    else if ((state_reg == STATE_ACK_A) && (state_next == STATE_STOP)) 
        error_reg <= 1'b1;
    else if ((state_reg == STATE_ACK_D) && cmd_write && cycle_end && ~acked_reg) 
        error_reg <= 1'b1;
end

wire            db_sda_i;   // debounce sda_i up to 50ns spike
wire            db_scl_i;   // debounce scl_i up to 50ns spike
wire            bus_busy = ~(db_scl_i & db_sda_i);
reg             db_sda_i_1;
wire            db_sda_change = db_sda_i ^ db_sda_i_1;

debouncer #(
    .DEFAULT(1'b1),
    .FILTER_SIZE(1)
) debouncer_SDA (
    .clk(clk),
    .asyn_rst_n(asyn_rst_n),
    .in(sda_i),
    .out(db_sda_i)
);

debouncer #(
    .DEFAULT(1'b1),
    .FILTER_SIZE(1)
) debouncer_SCL (
    .clk(clk),
    .asyn_rst_n(asyn_rst_n),
    .in(scl_i),
    .out(db_scl_i)
);

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        db_sda_i_1 <= 1'b1;
    else
        db_sda_i_1 <= db_sda_i;
end

reg     [3:0]   state_reg, state_next;
reg     [1:0]   cyc_state_reg, cyc_state_next;
wire            cycle_end;
reg     [7:0]   timer_cnt;
wire            timer_ready;
reg     [2:0]   bit_cnt;
reg     [1:0]   byte_cnt;
reg             acked_reg;
wire            sampling;
wire    [7:0]   tx_data_byte_0;
wire    [7:0]   tx_data_byte_1;
wire    [7:0]   tx_data_byte_2;
wire    [7:0]   tx_data_byte_3;
reg             sda_o_reg;
wire            sda_unmatch;
reg     [31:0]  rx_data_reg;

assign          sda_o = sda_o_reg;
assign          scl_o = cyc_state_reg[1];
assign          timer_ready = (timer_cnt == 8'h00);
assign          cycle_end = (cyc_state_reg == CYC_STATE_HIGH) & timer_ready;
assign          sampling = (cyc_state_reg == CYC_STATE_WHIGH) & db_scl_i;
assign          tx_data_byte_0 = tx_data_reg[7:0];
assign          tx_data_byte_1 = tx_data_reg[15:8];
assign          tx_data_byte_2 = tx_data_reg[23:16];
assign          tx_data_byte_3 = tx_data_reg[31:24];
assign          sda_unmatch = sampling & sda_o_reg & ~db_sda_i; 

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n) begin
        state_reg       <= STATE_BUSY;
        cyc_state_reg   <= CYC_STATE_HIGH;
    end else begin
        state_reg       <= state_next;
        cyc_state_reg   <= cyc_state_next;
    end
end

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        timer_cnt <= 8'hff;
    else if ((state_reg != STATE_IDLE) && (state_next == STATE_IDLE))
        timer_cnt <= T_SU_STA;
    else if ((state_reg != STATE_START) && (state_next == STATE_START))
        timer_cnt <= T_HD_STA;
    else if ((state_next != STATE_BUSY) && (state_next != STATE_IDLE) && (state_next != STATE_START)) begin
        if (cyc_state_reg == CYC_STATE_WHIGH) begin
            if (db_scl_i)
                timer_cnt <= T_HIGH;
        end else if (timer_ready)
            timer_cnt <= (cyc_state_reg == CYC_STATE_LOW_1) ? T_LOW_2 : T_LOW_1;
        else
            timer_cnt <= timer_cnt - 8'h01;
    end else if (~timer_ready)
        timer_cnt <= timer_cnt - 8'h01;
end

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        bit_cnt <= 3'h0;
    else if (state_reg == STATE_START)
        bit_cnt <= 3'h6;
    else if ((state_reg == STATE_ACK_A) || (state_reg == STATE_ACK_D))
        bit_cnt <= 3'h7;
    else if ((state_reg == STATE_ADDR) || (state_reg == STATE_DATA)) begin
        if (cycle_end)
            bit_cnt <= bit_cnt - 3'h1;
    end
end

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        byte_cnt <= 2'b00;
    else if (state_reg == STATE_ACK_A)
        byte_cnt <= 2'b00;
    else if ((state_reg == STATE_ACK_D) && (state_next == STATE_DATA))
        byte_cnt <= byte_cnt + 2'b01;
end

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        acked_reg <= 1'b0;
    else if (((state_reg == STATE_ACK_A) || (state_reg == STATE_ACK_D)) && sampling)
        acked_reg <= ~db_sda_i;
end

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        sda_o_reg <= 1'b1;
    else if ((state_reg == STATE_BUSY) || (state_reg == STATE_IDLE))
        sda_o_reg <= 1'b1;
    else if (state_reg == STATE_START)
        sda_o_reg <= 1'b0;
    else if ((cyc_state_reg == CYC_STATE_LOW_1) && timer_ready) begin
        if (state_reg == STATE_ADDR) begin
            sda_o_reg <= addr_reg[bit_cnt];
        end else if (state_reg == STATE_CMD) begin
            sda_o_reg <= cmd_reg[2];
        end else if (state_reg == STATE_ACK_A) begin
            sda_o_reg <= 1'b1;
        end else if (state_reg == STATE_DATA) begin
            if (cmd_read)
                sda_o_reg <= 1'b1;
            else if (cmd_write) begin
                case (byte_cnt)
                    2'b00: sda_o_reg <= tx_data_byte_0[bit_cnt];
                    2'b01: sda_o_reg <= tx_data_byte_1[bit_cnt];
                    2'b10: sda_o_reg <= tx_data_byte_2[bit_cnt];
                    2'b11: sda_o_reg <= tx_data_byte_3[bit_cnt];
                endcase
            end
        end else if (state_reg == STATE_ACK_D) begin
            if (cmd_read)
                sda_o_reg <= (byte_cnt == cmd_reg[1:0]);
            else if (cmd_write)
                sda_o_reg <= 1'b1;
        end else if (state_reg == STATE_STOP)
            sda_o_reg <= 1'b0;
    end
end

always @(posedge clk or negedge asyn_rst_n) begin
    if (~asyn_rst_n)
        rx_data_reg <= 32'h0;
    else if ((state_reg == STATE_DATA) && cmd_read && sampling) begin
        if (byte_cnt == 2'b00) begin
            rx_data_reg[0] <= db_sda_i;
            rx_data_reg[7:1] <= rx_data_reg[6:0];
        end else if (byte_cnt == 2'b01) begin
            rx_data_reg[8] <= db_sda_i;
            rx_data_reg[15:9] <= rx_data_reg[14:8];
        end else if (byte_cnt == 2'b10) begin
            rx_data_reg[16] <= db_sda_i;
            rx_data_reg[23:17] <= rx_data_reg[22:16];
        end else if (byte_cnt == 2'b11) begin
            rx_data_reg[24] <= db_sda_i;
            rx_data_reg[31:25] <= rx_data_reg[30:24];
        end
    end
end


always @(*) begin
    if ((state_next == STATE_BUSY) || (state_next == STATE_IDLE) || (state_next == STATE_START))
        cyc_state_next = CYC_STATE_HIGH;
    else if (cyc_state_reg == CYC_STATE_WHIGH) begin
        if (db_scl_i)
            cyc_state_next = CYC_STATE_HIGH;
        else
            cyc_state_next = cyc_state_reg;
    end else if (timer_ready)
        cyc_state_next = cyc_state_reg + 1;
    else
        cyc_state_next = cyc_state_reg;
end

always @(*) begin
    case (state_reg)
        STATE_BUSY: begin
            if (bus_busy)
                state_next = STATE_BUSY;
            else
                state_next = STATE_IDLE;
        end
        STATE_IDLE: begin
            if (bus_busy)
                state_next = STATE_BUSY;
            else if (timer_ready & busy_reg)
                state_next = STATE_START;
            else
                state_next = STATE_IDLE;
        end
        STATE_START: begin
            if (~db_scl_i)
                state_next = STATE_BUSY;
            else if (timer_ready)
                state_next = STATE_ADDR;
            else
                state_next = STATE_START;
        end
        STATE_ADDR: begin
            if (db_sda_change && db_scl_i)
                state_next = STATE_BUSY;
            else if (sda_unmatch)
                state_next = STATE_BUSY;
            else if ((bit_cnt == 3'h0) && cycle_end)
                state_next = STATE_CMD;
            else
                state_next = STATE_ADDR;
        end
        STATE_CMD: begin
            if (db_sda_change && db_scl_i)
                state_next = STATE_BUSY;
            else if (sda_unmatch)
                state_next = STATE_BUSY;
            else if (cycle_end)
                state_next = STATE_ACK_A;
            else
                state_next = STATE_CMD;
        end
        STATE_ACK_A: begin
            if (db_sda_change && db_scl_i)
                state_next = STATE_BUSY;
            else if (cycle_end) begin
                if (acked_reg)
                    state_next = STATE_DATA;
                else
                    state_next = STATE_STOP;
            end else
                state_next = STATE_ACK_A;
        end
        STATE_DATA: begin
            if (db_sda_change && db_scl_i)
                state_next = STATE_BUSY;
            else if (cmd_write && sda_unmatch)
                state_next = STATE_BUSY;
            else if ((bit_cnt == 3'h0) && cycle_end)
                state_next = STATE_ACK_D;
            else
                state_next = STATE_DATA;
        end
        STATE_ACK_D: begin
            if (db_sda_change && db_scl_i)
                state_next = STATE_BUSY;
            else if (cmd_read && sda_unmatch)
                state_next = STATE_BUSY;
            else if (cycle_end) begin
                if (~acked_reg || (byte_cnt == cmd_reg[1:0]))
                    state_next = STATE_STOP;
                else
                    state_next = STATE_DATA;
            end else
                state_next = STATE_ACK_D;
        end
        STATE_STOP: begin
            if (cycle_end) begin
                state_next = STATE_BUSY;
            end else
                state_next = STATE_STOP;
        end
        default:
            state_next = STATE_BUSY;
    endcase
end

endmodule