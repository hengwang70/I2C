`timescale 1ns/1ps

module debouncer #(
    parameter   DEFAULT = 1'b1,
    parameter   FILTER_SIZE = 4
)(
    input   wire        clk,
    input   wire        asyn_rst_n,
    input   wire        in,
    output  reg         out
);

    reg     [FILTER_SIZE-1:0]  filter_counter;
    reg     q1_in;
    reg     q2_in;
    wire    cnt_rst;
    wire    cnt_finish;

    assign  cnt_rst = ~asyn_rst_n | q1_in ^ q2_in;
    assign  cnt_finish = filter_counter == {FILTER_SIZE{1'b1}};

    always @(posedge clk or negedge asyn_rst_n) begin
        if (~asyn_rst_n)
            q1_in <= DEFAULT;
        else if (in == 1'b0)
            q1_in <= 1'b0;
        else if (in == 1'b1)
            q1_in <= 1'b1;
    end
    
    always @(posedge clk or negedge asyn_rst_n) begin
        if (~asyn_rst_n)
            q2_in <= DEFAULT;
        else
            q2_in <= q1_in;
    end

    always @(posedge clk or posedge cnt_rst) begin
        if (cnt_rst)
            filter_counter <= 0;
        else if (~cnt_finish)
            filter_counter <= filter_counter + 1;
    end

    always @(posedge clk or negedge asyn_rst_n) begin
        if (~asyn_rst_n)
            out <= DEFAULT;
        else if (cnt_finish)
            out <= q2_in;
    end

endmodule
