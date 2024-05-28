module PC( ///
    output reg [31:0] PC_out,		// instruction address
    input [31:0] branch_add,		// comes from exe/mem register from PC_branch
	 input PCSrc, jump,///						// comes from mem stage
	 input [25:0] jump_add,///		// comes from ID after decoding
    input clk,
    input rst
    );
	
	reg [31:0] temp_jump;
	reg [27:0] temp;
	
	always @(posedge clk or posedge rst)
		begin
			if (rst)
				begin
					PC_out <= 0;
				end
			else
				begin
					if (PCSrc)
						begin
							PC_out <= branch_add;
						end
					else if (jump)
						begin
							temp = {jump_add, 2'b00};
							temp_jump = {PC_out[31:28], temp};
							PC_out = temp_jump;
						end
					else
						begin
							PC_out <= PC_out + 1;
						end
				end			
		end

endmodule


module Instr_mem(
    output reg [31:0] instr_out,
    input [31:0] PC_out,
    input clk,
    input rst
    );

	reg [31:0] instructions [0:31];
	initial
		begin
			 instructions[0] = 32'b000000_00000_00000_00000_00000_000000; // rst
			 instructions[1] = 32'b000000_00011_00001_01000_00000_100000; // add $t0, $s0, $s1
			 instructions[2] = 32'b000000_10000_10001_01000_00000_100010; // subtract $t0, $s0, $s1
			 instructions[3] = 32'b000000_10000_10001_01000_00000_100100; // and $t0, $s0, $s1
			 instructions[4] = 32'b000000_10000_10001_01000_00000_100101; // or $t0, $s0, $s1
			 instructions[5] = 32'b000000_10000_10001_01000_00000_101010; // slt $t0, $s0, $s1
			 instructions[6] = 32'b100011_10001_01000_0000000000000100; // lw $t0, 4($s1)
			 instructions[7] = 32'b101011_10001_01000_0000000000000100; // sw $t0, 4($s1)
			 instructions[8] = 32'b000000_10000_10001_01000_00000_100100; // and $t0, $s0, $s1
			 instructions[9] = 32'b000100_00010_00010_0000000000000010; // beq $s0, $s1, 4
			 instructions[10] = 32'b000000_00000_00000_00000_00000_000000; // rst
			 instructions[11] = 32'b000000_10000_10001_01000_00000_100000; // add $t0, $s0, $s1
			 instructions[12] = 32'b000000_10000_10001_01000_00000_100010; // subtract $t0, $s0, $s1
			 instructions[13] = 32'b000000_10000_10001_01000_00000_100100; // and $t0, $s0, $s1
			 instructions[14] = 32'b000000_10000_10001_01000_00000_100101; // or $t0, $s0, $s1
			 instructions[15] = 32'b000000_10000_10001_01000_00000_101010; // slt $t0, $s0, $s1
			 instructions[16] = 32'b100011_10001_01000_0000000000000100; // lw $t0, 4($s1)
			 instructions[17] = 32'b101011_10001_01000_0000000000000100; // sw $t0, 4($s1)
			 instructions[18] = 32'b000100_10000_10001_0000000000000100; // beq $s0, $s1, 4
			 instructions[19] = 32'b000010_00000000000000000000001100; // jump to address 12
			 instructions[20] = 32'b000000_00000_00000_00000_00000_000000; // rst
			 instructions[21] = 32'b000000_10000_10001_01000_00000_100000; // add $t0, $s0, $s1
			 instructions[22] = 32'b000000_10000_10001_01000_00000_100010; // subtract $t0, $s0, $s1
			 instructions[23] = 32'b000000_10000_10001_01000_00000_100101; // or $t0, $s0, $s1
			 instructions[24] = 32'b000010_00000000000000000000000001; // jump to 4 
			 instructions[25] = 32'b000000_10000_10001_01000_00000_101010; // slt $t0, $s0, $s1
			 instructions[26] = 32'b100011_10001_01000_0000000000000100; // lw $t0, 4($s1)
			 instructions[27] = 32'b101011_10001_01000_0000000000000100; // sw $t0, 4($s1)
			 instructions[28] = 32'b000100_10000_10001_0000000000000100; // beq $s0, $s1, 4
			 instructions[29] = 32'b000010_00000000000000000000001100; // jump to address 12
			 instructions[30] = 32'b000000_10000_10001_01000_00000_100000; // add $t0, $s0, $s1
			 instructions[31] = 32'b000000_10000_10001_01000_00000_100010; // subtract $t0, $s0, $s1
		end
	
	always @(*)
		begin
			if (rst)
				begin
					instr_out <= instructions[0];
				end
			else
				begin
					instr_out <= instructions[PC_out];
					// Debug statement to trace PC_out and instruction
                    //$display("At time %0t: PC_out = %0d, instr_out = %b", $time, PC_out, instr_out);
				end
		end

endmodule


module IF_ID_reg(
	output reg [31:0] instr_out_reg,
	output reg [31:0] PC_out_reg,
	input [31:0] PC_out,
	input [31:0] instr_out, 
	input clk, rst
    );

	always@(posedge clk or posedge rst)
		begin 
			if (rst)
				begin
					instr_out_reg <= 0;
					PC_out_reg <= 0;
				end
			else 
				begin
					instr_out_reg <= instr_out;
					PC_out_reg <= PC_out;
				end
		end
endmodule



module IF(
	 output [31:0] instr_out_reg,
	 output [31:0] PC_out_reg,
    input [31:0] branch_add,		// comes from ex/mem register from PC_branch
	 input [25:0] jump_add,///
	 input PCSrc, Jump,///
	 input clk,
    input rst
    );
	 
	 wire [31:0] PC_out;		// instruction address
    wire [31:0] instr_out;
	 
	 PC PC_IF (
    .PC_out(PC_out),		
    .branch_add(branch_add),		
	 .PCSrc(PCSrc),	
	 .jump(Jump),
	 .jump_add(jump_add),
    .clk(clk),
    .rst(rst)
    );

	 Instr_mem instr_IF (
    .instr_out(instr_out),
    .PC_out(PC_out),
    .clk(clk),
    .rst(rst)
    );

	 IF_ID_reg reg1 (
	 .instr_out_reg(instr_out_reg),
	 .PC_out_reg(PC_out_reg),
	 .PC_out(PC_out),
	 .instr_out(instr_out), 
	 .clk(clk),
	 .rst(rst)
    );
	
endmodule
