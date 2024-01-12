`timescale 1ns/1ps

module model_i2c_master (
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

localparam
    T_UNIT = 1,
    T_LOW = 4700,
    T_HIGH = 4000,
    T_BUF = 4700,
    T_VD_DAT = 3000,
    T_HD_DAT = 900,
    T_HD_STA = 4000,
    T_SU_STO = 4000;

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
    CMD_WRITE   = 2'b00,
    CMD_READ    = 2'b01;

reg             db_sda_i;
reg             db_scl_i;
reg             sda_o_reg;
reg             scl_o_reg;

reg     [6:0]   addr_reg;
reg     [31:0]  tx_data_reg;
reg     [31:0]  rx_data_reg;
reg     [3:0]   cmd_reg;

wire            cmd_write = (cmd_reg[3:2] == CMD_WRITE);
wire            cmd_read = (cmd_reg[3:2] == CMD_READ);
reg             busy_reg;
reg             error_reg;

assign          host_busy_o = busy_reg;
assign          host_error_o = error_reg;
assign          host_data_o = rx_data_reg;

reg     [3:0]   state;
wire            #(T_BUF, 0)     ready_to_start = db_sda_i & db_scl_i;
wire            #(T_HD_STA, 0)  start_enough = ~db_sda_i & db_scl_i;
wire            #(T_SU_STO, 0)  stop_enough = ~db_sda_i & db_scl_i;
reg     [3:0]   bit_cnt;
reg     [1:0]   byte_cnt;
reg             sda_out;
reg     [7:0]   data_reg;
reg             sda_unmatch;
reg             slave_ack;

initial begin
    sda_o_reg   = 1'b1;
    scl_o_reg   = 1'b1;
    state       = STATE_IDLE;
    busy_reg    = 1'b0;
    error_reg   = 1'b0;
    bit_cnt     = 4'h0;
    byte_cnt    = 2'b00;
    sda_out     = 1'b1;
    sda_unmatch = 1'b0;
    slave_ack   = 1'b0;
end

// I/O buffer
always @(*) begin
    if (sda_i == 1'b0)
        db_sda_i = 1'b0;
    else if (sda_i == 1'b1)
        db_sda_i = 1'b1;
end
buf (highz1, strong0) U1 (sda_o, sda_o_reg);

always @(*) begin
    if (scl_i == 1'b0)
        db_scl_i = 1'b0;
    else if (scl_i == 1'b1)
        db_scl_i = 1'b1;
end
buf (highz1, strong0) U2 (scl_o, scl_o_reg);

// Host Interface
always @(posedge host_sel_i) begin
    #T_UNIT
    addr_reg <= host_addr_i[6:0];
    cmd_reg <= host_cmd_i;
    tx_data_reg <= host_data_i;
    if (~busy_reg && ((host_cmd_i[3:2] == CMD_READ) || (host_cmd_i[3:2] == CMD_WRITE)))
        busy_reg <= 1'b1;
    error_reg <= 1'b0;
end

// start transaction
always @(posedge busy_reg) begin
    wait (state == STATE_IDLE && ready_to_start)
    state = STATE_START;
    sda_o_reg = 1'b0;
    wait (start_enough)
    scl_o_reg = 1'b0;
end

// Detect STOP
always @(db_sda_i) begin
    if (db_scl_i && state != STATE_START) begin
        state = STATE_IDLE;
    end
end

always @(posedge sda_unmatch) begin
    state = STATE_IDLE;
end


// @ the rising edge of SCL
// 1. sampling SDA and shift into data_reg
// 2. update bit_cnt at Address and Data phase
// 3. load first-byte to data_reg from tx_data_reg at Address-Ack phase
// 4. update byte_cnt and load/store data_reg at Data-Ack phase
always @(posedge db_scl_i) begin
    if (state == STATE_ADDR) begin
        bit_cnt = bit_cnt + 1;
        if (sda_o_reg != db_sda_i)
            sda_unmatch = 1'b1;
        if (bit_cnt < 4'h7)
            sda_out = addr_reg[6-bit_cnt];
    end else if (state == STATE_ACK_A) begin
        slave_ack = ~db_sda_i;
        if (cmd_write && slave_ack)
            data_reg = tx_data_reg[7:0];
    end else if (state == STATE_DATA) begin
        data_reg = {data_reg[6:0], db_sda_i};
        if (cmd_write) begin
            if (sda_o_reg != db_sda_i)
                sda_unmatch = 1'b1;
            sda_out = data_reg[7];
        end
        bit_cnt = bit_cnt + 1;
    end else if (state == STATE_ACK_D) begin
        if (cmd_write) begin
            slave_ack = ~db_sda_i;
            if (slave_ack && byte_cnt != cmd_reg[1:0]) begin
                byte_cnt = byte_cnt + 1;
                case (byte_cnt)
                    2'b01: data_reg = tx_data_reg[15:8];
                    2'b10: data_reg = tx_data_reg[23:16];
                    2'b11: data_reg = tx_data_reg[31:24];
                endcase
            end
        end else begin
            if (sda_o_reg != db_sda_i)
                sda_unmatch = 1'b1;
            case (byte_cnt)
                2'b00: rx_data_reg[7:0] = data_reg;
                2'b01: rx_data_reg[15:8] = data_reg;
                2'b10: rx_data_reg[23:16] = data_reg;
                2'b11: rx_data_reg[31:24] = data_reg;
            endcase
            if (byte_cnt != cmd_reg[1:0])
                byte_cnt = byte_cnt + 1;
        end
    end else if (state == STATE_STOP) begin
        wait (stop_enough)
        sda_o_reg = 1'b1;
        busy_reg = 1'b0;
    end
end

// Output SDA
always @(negedge db_scl_i) begin
    #T_HD_DAT 
    if (sda_out)
        sda_o_reg = 1'b1;
    else begin
        #(T_VD_DAT - T_HD_DAT)
        sda_o_reg <= 1'b0;
    end
end

// Output SCL
always @(negedge db_scl_i) begin
    if (state != STATE_IDLE) begin
        scl_o_reg = 1'b0;
        #T_LOW
        scl_o_reg = 1'b1;
    end
end

always @(posedge db_scl_i) begin
    if (state != STATE_IDLE && state != STATE_STOP) begin
        #T_HIGH
        scl_o_reg = 1'b0;
    end
end


// update sda_out at state transition
always @(state) begin
    case(state)
        STATE_ADDR: begin
            sda_out = addr_reg[6];
            bit_cnt = 4'h0;
            byte_cnt = 2'b00;
        end
        STATE_CMD: begin
            sda_out = cmd_read;
        end
        STATE_ACK_A: begin
            sda_out = 1'b1;
        end
        STATE_DATA: begin
            if (cmd_write)
                sda_out = data_reg[7];
            else
                sda_out = 1'b1;
            bit_cnt = 4'h0;
        end
        STATE_ACK_D: begin
            sda_out = cmd_write | cmd_read & (byte_cnt == cmd_reg[1:0]);
        end
        STATE_STOP: begin
            sda_out = 1'b0;
        end
        STATE_IDLE: begin
            sda_out = 1'b1;
            sda_o_reg = 1'b1;
            scl_o_reg = 1'b1;
            sda_unmatch = 1'b0;
        end
    endcase
end

// State change @ falling edge of SCL
always @(negedge db_scl_i) begin
    case(state)
        STATE_START: begin
            state <= STATE_ADDR;
        end
        STATE_ADDR: begin
            if (bit_cnt == 7) begin
                state <= STATE_CMD;
            end
        end
        STATE_CMD: begin
            state <= STATE_ACK_A;
        end
        STATE_ACK_A: begin
            if (slave_ack)
                state <= STATE_DATA;
            else begin
                state <= STATE_STOP;
                error_reg <= 1'b1;
            end
        end
        STATE_DATA: begin
            if (bit_cnt == 8)
                state <= STATE_ACK_D;
        end
        STATE_ACK_D: begin
            if (cmd_write) begin
                if (slave_ack && byte_cnt != cmd_reg[1:0])
                    state <= STATE_DATA;
                else
                    state <= STATE_STOP;
                error_reg <= ~slave_ack;
            end else begin
                if (sda_out == 1'b0)
                    state <= STATE_DATA;
                else
                    state <= STATE_STOP;
            end
        end
        STATE_STOP: begin
            state <= STATE_IDLE;
            busy_reg <= 1'b0;
        end
    endcase
end

endmodule