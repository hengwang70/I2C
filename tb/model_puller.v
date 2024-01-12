`timescale 1ns/1ps

module model_puller #(
    parameter   T_R = 1,
    parameter   T_F = 1
)(
    input  wire     in,
    output reg      out
);
    wire    rise;
    wire    fall;
    assign #(0, 0, T_R) rise = in;
    assign #(0, T_F, 0) fall = in;
    initial #1 out = 1;
    always @(in) out = 1'bz;
    always @(posedge rise) out = 1'b1;
    always @(negedge fall) out = 1'b0;
endmodule
