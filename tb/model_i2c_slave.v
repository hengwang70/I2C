`timescale 1ns/1ps

module model_i2c_slave #(
    parameter   DEVICE_ID = 7'b0001000,
    parameter   SIZE = 1
)(
    input  wire         scl_i,
    output wire         scl_o,
    input  wire         sda_i,
    output wire         sda_o
);

localparam
    T_EX_CLK = 1237,
    T_VD_DAT = 3000,
    T_HD_DAT = 900,
    T_HD_STA = 4000;

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

localparam
    READ    = 1'b1,
    WRITE   = 1'b0;

reg     db_sda_i;
reg     db_scl_i;
reg     sda_o_reg;
reg     scl_o_reg;


reg     [3:0]   state;
reg     [7:0]   addr_reg;
reg     [7:0]   mem[0:SIZE-1];
reg     [7:0]   data_reg;
reg             sda_out;
reg             master_ack;
integer         counter;
integer         index;
integer         rand;
wire            cmd_rw = addr_reg[0];
wire            valid_addr = (addr_reg[7:1] >= DEVICE_ID) & (addr_reg[7:1] <= (DEVICE_ID + SIZE - 1));

initial begin
    sda_o_reg   = 1'b1;
    scl_o_reg   = 1'b1;
    state       = STATE_IDLE;
    sda_out     = 1'b1;
    master_ack  = 1'b0;
    counter     = 0;
    for (index = 0; index < SIZE; index = index + 1)
        mem[index] = index + 8'h10;
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

// Detect START
always @(negedge db_sda_i) begin
    if (db_scl_i)
        state <= STATE_START;
end

// Detect STOP
always @(posedge db_sda_i) begin
    if (db_scl_i)
        state <= STATE_IDLE;
end

// @ the rising edge of SCL
// 1. sampling SDA, save to master_ack or shift into addr_reg/data_reg
// 2. update counter at Address and Data phase
// 3. set initial index and load data from memory at Address-Ack phase
// 4. update index and load/store memory at Data-Ack phase
always @(posedge db_scl_i) begin
    if (state == STATE_ADDR) begin
        addr_reg = {addr_reg[6:0], db_sda_i};
        counter = counter + 1;
    end else if (state == STATE_ACK_A) begin
        if (valid_addr) begin
            index = addr_reg[7:1] - DEVICE_ID;
            if (cmd_rw == READ)
                data_reg = mem[index];
        end
    end else if (state == STATE_DATA) begin
        data_reg = {data_reg[6:0], db_sda_i};
        if (cmd_rw == READ)
            sda_out = data_reg[7];
        counter = counter + 1;
    end else if ((state == STATE_ACK_D)) begin
        master_ack = ~db_sda_i;
        if (cmd_rw == READ) begin
            $display("read   mem[%h] = %h", index+DEVICE_ID, data_reg);
            if (master_ack) begin
                index = index + 1;
                if (index != SIZE)
                    data_reg = mem[index];
            end
        end else begin
            mem[index] = data_reg;
            $display("write  mem[%h] = %h", index+DEVICE_ID, data_reg);
            index = index + 1;
        end
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

always @(state) begin
    case(state)
        STATE_ADDR: begin
            counter = 0;
        end
        STATE_ACK_A: begin
            sda_out = valid_addr ? 1'b0 : 1'b1;
        end
        STATE_DATA: begin
            if (cmd_rw == READ)
                sda_out = data_reg[7];
            else
                sda_out = 1'b1;
            counter = 0;
        end
        STATE_ACK_D: begin
            sda_out = cmd_rw;
        end
        STATE_STOP: begin
            sda_out = 1'b1;
        end
    endcase
end

// SCL extension (randomly)
always @(negedge db_scl_i) begin
    rand = $random % 14;
    if (rand < 0)
        rand = -rand;
    #T_EX_CLK
    scl_o_reg = 1'b0;
    #(T_EX_CLK * rand)
    scl_o_reg = 1'b1;
end

// State change @ falling edge of SCL
always @(negedge db_scl_i) begin
    case(state)
        STATE_START: begin
            state <= STATE_ADDR;
            counter <= 0;
        end
        STATE_ADDR: begin
            if (counter == 8) begin
                state <= STATE_ACK_A;
            end
        end
        STATE_ACK_A: begin
            if (valid_addr)
                state <= STATE_DATA;
            else
                state <= STATE_STOP;
        end
        STATE_DATA: begin
            if (counter == 8)
                state <= STATE_ACK_D;
        end
        STATE_ACK_D: begin
            if (cmd_rw == READ) begin
                if (master_ack && (index < SIZE))
                    state <= STATE_DATA;
                else
                    state <= STATE_STOP;
            end else begin
                if (index < SIZE)
                    state <= STATE_DATA;
                else
                    state <= STATE_STOP;
            end
        end
    endcase
end

endmodule