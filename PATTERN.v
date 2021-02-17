`timescale 1ns/10ps
`define CYCLE_TIME 4.0

module PATTERN(
	// Output signals
	clk,
	rst_n,
	in_valid,
	instruction,
    // Input signals
    out_valid,
    busy,
    out0,
    out1,
    out2,
    out3,
    out4,
    out5,
    out6,
    out7,
    out8,
    out9,
    out10,
    out11,
    out12,
    out13,
    out14,
    out15
);

//---------------------------------------------------------------------
//   INPUT AND OUTPUT DECLARATION
//---------------------------------------------------------------------
output reg clk, rst_n, in_valid;
output reg [18:0] instruction;
input busy, out_valid;
input signed [15:0] out0, out1, out2, out3, out4, out5, out6, out7, out8, out9, out10, out11, out12, out13, out14, out15;

//---------------------------------------------------------------------
//   PARAMETER
//---------------------------------------------------------------------
parameter pattern_num = 982;

//---------------------------------------------------------------------
//   WIRE AND REG DECLARATION
//---------------------------------------------------------------------
reg	[15:0] data_mem[0:8191];
reg	[2:0] op_code;
reg	[3:0] rs, rt, rd, rl, func;
reg	signed [7:0] immed;
reg signed [15:0] r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12, r13, r14, r15;
reg signed [15:0] reg_s, reg_t, reg_d, reg_l;
reg	[18:0] ff   [0:pattern_num+17];
reg	[15:0] oo0  [0:pattern_num+17];
reg	[15:0] oo1  [0:pattern_num+17];
reg	[15:0] oo2  [0:pattern_num+17];
reg	[15:0] oo3  [0:pattern_num+17];
reg	[15:0] oo4  [0:pattern_num+17];
reg	[15:0] oo5  [0:pattern_num+17];
reg	[15:0] oo6  [0:pattern_num+17];
reg	[15:0] oo7  [0:pattern_num+17];
reg	[15:0] oo8  [0:pattern_num+17];
reg	[15:0] oo9  [0:pattern_num+17];
reg	[15:0] oo10 [0:pattern_num+17];
reg	[15:0] oo11 [0:pattern_num+17];
reg	[15:0] oo12 [0:pattern_num+17];
reg	[15:0] oo13 [0:pattern_num+17];
reg	[15:0] oo14 [0:pattern_num+17];
reg	[15:0] oo15 [0:pattern_num+17];
reg [3:0] put_1[0:4], put_2[0:4]; // For Data Hazard

//---------------------------------------------------------------------
//   INTEGER
//---------------------------------------------------------------------
integer seed = 1;
integer i, lat, total_latency;
integer patcount, patcount2;
integer patgen;
integer f, o0, o1, o2, o3, o4, o5, o6, o7, o8, o9, o10, o11, o12, o13, o14, o15;

//---------------------------------------------------------------------
//   CLOCK
//---------------------------------------------------------------------
real    CYCLE = `CYCLE_TIME;
always	#(CYCLE/2.0) clk = ~clk;
initial	clk = 0;

always	#(CYCLE) total_latency = total_latency + 1;

//---------------------------------------------------------------------
//   INITIAL
//---------------------------------------------------------------------
initial begin
	rst_n = 1; in_valid = 0; instruction = 19'bx; 
    r0  = 16'd0; r1  = 16'd0; r2  = 16'd0; r3  = 16'd0;
    r4  = 16'd0; r5  = 16'd0; r6  = 16'd0; r7  = 16'd0;
    r8  = 16'd0; r9  = 16'd0; r10 = 16'd0; r11 = 16'd0;
    r12 = 16'd0; r13 = 16'd0; r14 = 16'd0; r15 = 16'd0;
    
	force clk = 0;
	
	total_latency = 0;
	
    reset_signal_task;
	
	$readmemh("golden/instruction.txt" ,ff);
	$readmemh("golden/out0.txt", oo0);
	$readmemh("golden/out1.txt", oo1);
	$readmemh("golden/out2.txt", oo2);
	$readmemh("golden/out3.txt", oo3);
	$readmemh("golden/out4.txt", oo4);
	$readmemh("golden/out5.txt", oo5);
	$readmemh("golden/out6.txt", oo6);
	$readmemh("golden/out7.txt", oo7);
	$readmemh("golden/out8.txt", oo8);
	$readmemh("golden/out9.txt", oo9);
	$readmemh("golden/out10.txt", oo10);
	$readmemh("golden/out11.txt", oo11);
	$readmemh("golden/out12.txt", oo12);
	$readmemh("golden/out13.txt", oo13);
	$readmemh("golden/out14.txt", oo14);
	$readmemh("golden/out15.txt", oo15);

	fork
		for(patcount = 0; patcount < pattern_num + 16 + 2; patcount = patcount + 1) begin
			input_task;
			lat = 0;
			while(busy !== 0) begin
				lat = lat + 1;
				if(lat >= 10) begin
                    $display("-------------------------------------------------------------------------------");
                    $display("                                     FAIL!                                     ");
                    $display("                   The execution latency are over 10 cycles                    ");
                    $display("-------------------------------------------------------------------------------");
					repeat(2)@(negedge clk);
					$finish;
				end
				@(negedge clk);
			end	
		end
	
		for(patcount2 = 0; patcount2 < pattern_num + 16 + 2; patcount2 = patcount2 + 1) begin
			while(out_valid !== 1)	begin
				@(negedge clk);
			end
			check_output_task;
			$display("                      \033[0;34mPASS PATTERN NO.%5d\033[0m                      ", patcount2 + 1);
			@(negedge clk);
		end
	join
	
	YOU_PASS_task;
end

//---------------------------------------------------------------------
//   TASK
//---------------------------------------------------------------------
task reset_signal_task; begin
    #(0.5); rst_n = 0;
	#(2.0);
	if((busy !== 0) || 
    (out0 !== 16'd0) || (out1 !== 16'd0) || (out2 !== 16'd0) || (out3 !== 16'd0) || 
    (out4 !== 16'd0) || (out5 !== 16'd0) || (out6 !== 16'd0) || (out7 !== 16'd0) || 
    (out8 !== 16'd0) || (out9 !== 16'd0) || (out10 !== 16'd0) || (out11 !== 16'd0) || 
    (out12 !== 16'd0) || (out13 !== 16'd0) || (out14 !== 16'd0) || (out15 !== 16'd0)) begin
		$display("-------------------------------------------------------------------------------");
		$display("                                     FAIL!                                     ");
		$display("              Output signal should be 0 after initial RESET at %4t             ", $time);
		$display("-------------------------------------------------------------------------------");
		$finish;
	end
	#(10); rst_n = 1;
	#(3); release clk;
end endtask


task input_task; begin
	instruction = ff[patcount];
	op_code = instruction[18:16];
	rs = instruction[15:12];
	rt = instruction[11:8];
	rd = instruction[7:4];
	rl = instruction[3:0];
	
	func = rl;
	immed = {rd, rl};
	
	in_valid = 1;
	
	@(negedge clk);
	in_valid = 0; instruction = 19'bx;
end	endtask


task check_output_task;	begin
	r0  = oo0[patcount2];
	r1  = oo1[patcount2];
	r2  = oo2[patcount2];
	r3  = oo3[patcount2];
	r4  = oo4[patcount2];
	r5  = oo5[patcount2];
	r6  = oo6[patcount2];
	r7  = oo7[patcount2];
	r8  = oo8[patcount2];
	r9  = oo9[patcount2];
	r10 = oo10[patcount2];
	r11 = oo11[patcount2];
	r12 = oo12[patcount2];
	r13 = oo13[patcount2];
	r14 = oo14[patcount2];
	r15 = oo15[patcount2];
	
	check_reg0_task;
	check_reg1_task;
	check_reg2_task;
	check_reg3_task;
	check_reg4_task;
	check_reg5_task;
	check_reg6_task;
	check_reg7_task;
	check_reg8_task;
	check_reg9_task;
	check_reg10_task;
	check_reg11_task;
	check_reg12_task;
	check_reg13_task;
	check_reg14_task;
	check_reg15_task;
end	endtask


task check_reg0_task; begin
	if(out0 !== r0)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out0  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r0, out0, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg1_task; begin
	if(out1 !== r1)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out1  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r1, out1, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg2_task; begin
	if(out2 !== r2)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out2  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r2, out2, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg3_task; begin
	if(out3 !== r3)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out3  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r3, out3, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg4_task; begin
	if(out4 !== r4)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out4  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r4, out4, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg5_task; begin
	if(out5 !== r5)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out5  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r5, out5, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg6_task; begin
	if(out6 !== r6)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out6  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r6, out6, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg7_task; begin
	if(out7 !== r7)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out7  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r7, out7, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg8_task; begin
	if(out8 !== r8)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out8  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r8, out8, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg9_task; begin
	if(out9 !== r9)	begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out9  FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r9, out9, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg10_task; begin
	if(out10 !== r10) begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out10 FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r10, out10, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg11_task; begin
	if(out11 !== r11) begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out11 FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r11, out11, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg12_task; begin
	if(out12 !== r12) begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out12 FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r12, out12, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg13_task; begin
	if(out13 !== r13) begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out13 FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r13, out13, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg14_task; begin
	if(out14 !== r14) begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out14 FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r14, out14, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask

task check_reg15_task; begin
	if(out15 !== r15) begin
        $display("-------------------------------------------------------------------------------");
		$display("                               PATTERN NO.%5d                                  ", patcount2 + 1);
        $display("                                 out15 FAIL!                                   ");
		$display("                 Ans : %4h, Your output : %4h at %8t                           ", r15, out15, $time);
        $display("-------------------------------------------------------------------------------");
		repeat(2)@(negedge clk);
		$finish;
	end
end	endtask


task YOU_PASS_task; begin
	$display("-------------------------------------------------------------------");
	$display("                         Congratulations!                          ");
	$display("                   You have passed all patterns!                   ");
	$display("                 Your execution cycles = %5d cycles                ", total_latency);
	$display("                    Your clock period = %.1f ns                    ", CYCLE);
	$display("                    Your total latency = %.1f ns                   ", total_latency * CYCLE);
	$display("-------------------------------------------------------------------");
	$finish;
end endtask

endmodule