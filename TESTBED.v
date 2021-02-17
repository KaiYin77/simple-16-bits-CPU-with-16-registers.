`timescale 1ns/10ps
`include "PATTERN.v"
`include "SCPU_0710851_submit.v"
`include "MEMORY.v"

module TESTBED();

// Connection wires
wire clk, rst_n, in_valid;
wire [18:0] instruction;
wire busy, out_valid;
wire signed [15:0] out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11, out12, out13, out14, out15;

wire WEN;
wire [12:0] ADDR;
wire signed [15:0] MEM_in, MEM_out;

initial begin
    @(negedge rst_n);
    $readmemh("golden/data.txt", I_MEMORY.mem);
    $dumpfile("SCPU_waveform_test.vcd");
    $dumpvars;
end

MEMORY I_MEMORY(
    .MEM_out(MEM_out),
    .CLK(clk),
    .CEN(1'b0),
    .WEN(WEN),
    .ADDR(ADDR),
    .MEM_in(MEM_in),
	.OEN(1'b0)
);

SCPU I_SCPU(
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .instruction(instruction),
    .busy(busy),
    .out_valid(out_valid),
    .out0(out0),
    .out1(out1),
    .out2(out2),
    .out3(out3),
    .out4(out4),
    .out5(out5),
    .out6(out6),
    .out7(out7),
    .out8(out8),
    .out9(out9),
    .out10(out10),
    .out11(out11),
    .out12(out12),
    .out13(out13),
    .out14(out14),
    .out15(out15),
    .MEM_out(MEM_out),
    .WEN(WEN),
    .ADDR(ADDR),
    .MEM_in(MEM_in)
);

PATTERN I_PATTERN(
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .instruction(instruction),
    .busy(busy),
    .out_valid(out_valid),
    .out0(out0),
    .out1(out1),
    .out2(out2),
    .out3(out3),
    .out4(out4),
    .out5(out5),
    .out6(out6),
    .out7(out7),
    .out8(out8),
    .out9(out9),
    .out10(out10),
    .out11(out11),
    .out12(out12),
    .out13(out13),
    .out14(out14),
    .out15(out15)
);

endmodule