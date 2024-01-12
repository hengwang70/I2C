`timescale 1ns/1ps
`include "../rtl/i2c_master.v"
`include "./model_i2c_slave.v"
`include "./model_i2c_master.v"
`include "./model_puller.v"

module t_i2c_master;

localparam
    T_F = 80,
    T_R = 400;

localparam
    I2C_MASTER      = 8'h5A,
    MODEL_MASTER    = 8'h9C;

reg             clk;
reg             asyn_rst_n;
reg             host_start;
reg    [7:0]    h_addr;
reg    [3:0]    h_cmd;
reg    [31:0]   h_data;
reg             h_sel_i2c_m;
reg             h_sel_model_m;

reg     [55:0]  program[0:15];
reg     [3:0]   pc;

initial begin
    $readmemh("t_i2c_master.mem", program, 0, 5);
    $dumpfile("t_i2c_master.vcd");
    $dumpvars;

        clk = 1'b0;
        asyn_rst_n = 1'b0;
        host_start = 1'b0;
        pc = 4'h0;
        h_sel_i2c_m = 1'b0;
        h_sel_model_m = 1'b0;
#1000   asyn_rst_n = 1'b1;
#1000   host_start = 1'b1;
        wait(host_start == 1'b0);
        wait(~i2c_m_busy && ~model_m_busy);
#10000;
    $finish;

end

always #15 clk <= ~clk;

always @(posedge clk) begin
    if (host_start) begin
        h_addr <= program[pc][47:40];
        h_cmd <= program[pc][35:32];
        h_data <= program[pc][31:0];
        if (~h_sel_i2c_m && ~i2c_m_busy) begin
            if (program[pc] == 56'h0) begin
                host_start <= 1'b0;
            end else if (program[pc][55:48] == I2C_MASTER) begin
                h_sel_i2c_m <= 1'b1;
                pc <= pc + 4'h1;
            end
        end
        if (~h_sel_model_m && ~model_m_busy) begin
            if (program[pc] == 56'h0) begin
                host_start <= 1'b0;
            end else if (program[pc][55:48] == MODEL_MASTER) begin
                h_sel_model_m <= 1'b1;
                pc <= pc + 4'h1;
            end
        end
        if (h_sel_i2c_m)
            h_sel_i2c_m <= 1'b0;
        if (h_sel_model_m)
            h_sel_model_m <= 1'b0;
    end
end


wire            i2c_bus_scl;
wire            i2c_bus_sda;

// Pull-Up delay model
wire            i2c_bus_scl_pull;
wire            i2c_bus_sda_pull;
model_puller #(.T_R(T_R), .T_F(T_F)) puller_U1 (.out(i2c_bus_scl_pull), .in(i2c_bus_scl));
model_puller #(.T_R(T_R), .T_F(T_F)) puller_U2 (.out(i2c_bus_sda_pull), .in(i2c_bus_sda));

// I2C_master I/O
wire            io_sda;
wire            io_scl;
buf (highz1, strong0) o_buf_sda (i2c_bus_sda, i2c_m_sda);
buf (highz1, strong0) o_buf_scl (i2c_bus_scl, i2c_m_scl);
buf                   i_buf_sda (io_sda, i2c_bus_sda_pull);
buf                   i_buf_scl (io_scl, i2c_bus_scl_pull);

wire            i2c_m_busy;
wire            i2c_m_error;
wire    [31:0]  i2c_m_data;

wire            model_m_busy;
wire            model_m_error;
wire    [31:0]  model_m_data;

model_i2c_slave #(
    .DEVICE_ID(7'b0110011),
    .SIZE(16)
) model_i2c_slave_U1 (
	.scl_i(i2c_bus_scl_pull),
	.scl_o(i2c_bus_scl),
	.sda_i(i2c_bus_sda_pull),
	.sda_o(i2c_bus_sda)
);

model_i2c_master model_i2c_master_U1 (
	/* i2c interface */
	.scl_i(i2c_bus_scl_pull),
	.scl_o(i2c_bus_scl),
	.sda_i(i2c_bus_sda_pull),
	.sda_o(i2c_bus_sda),
	
	/* host interface */
	.host_addr_i(h_addr),
	.host_data_i(h_data),
	.host_cmd_i(h_cmd),
	.host_sel_i(h_sel_model_m),
	.host_data_o(model_m_data),
	.host_busy_o(model_m_busy),
	.host_error_o(model_m_error)
);

i2c_master i2c_master_U1 (
	.clk(clk),
	.asyn_rst_n(asyn_rst_n),
	
	    /* i2c interface */
	.scl_i(io_scl),
	.scl_o(i2c_m_scl),
	.sda_i(io_sda),
	.sda_o(i2c_m_sda),
	
	    /* host interface */
	.host_addr_i(h_addr),
	.host_data_i(h_data),
	.host_cmd_i(h_cmd),
	.host_sel_i(h_sel_i2c_m),
	.host_data_o(i2c_m_data),
	.host_busy_o(i2c_m_busy),
	.host_error_o(i2c_m_error)
);

endmodule