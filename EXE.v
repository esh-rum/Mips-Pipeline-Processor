module PC_branch(
	 output reg [31:0] branchAddOut,		// address to which the PC will branch to
    input [31:0] addFromPC,		// output of PC from id/exe register
    input [31:0] branch_add,		// comes from ID after sign extending the 16 bits
    input clk,
    input rst
    );

	reg [31:0]	temp_branch;
	
	always @(posedge clk or posedge rst)
		begin
			if (rst)
				begin
					branchAddOut <= 0;
				end
			else
				begin
					temp_branch <= branch_add << 2;
					branchAddOut <= temp_branch + addFromPC;
				end			
		end
endmodule


module ALUcontrol(
	output reg [3:0] ALUoperation,
	input [1:0] ALUop_controlSignals,
	input [5:0] funct,
	input clk, rst
    );

	always @(*)
		begin
			if (rst) 
				begin
					ALUoperation <= 0;
				end
			else 
				begin
					case (ALUop_controlSignals)
						2'b00:
							begin
								ALUoperation <= 4'b0010;
							end
						2'b01: ALUoperation <= 4'b0110;
						2'b10:
							begin
								case (funct)
									6'b100000: ALUoperation <= 4'b0010;
									6'b100010: ALUoperation <= 4'b0110;
									6'b100100: ALUoperation <= 4'b0000;
									6'b100101: ALUoperation <= 4'b0001;
									6'b101010: ALUoperation <= 4'b0111;
									default: ALUoperation <= 4'b0010;
								endcase
							end
						default: ALUoperation <= 4'b0010;
					endcase
				end
		end

endmodule


module ALU(
	output reg [31:0] ALUresult,
	output reg zero,
	input [31:0] Rs_Data, Rt_Data,
	input [31:0] ext_imm,
	input [3:0] ALUoperation, 
	input ALUsrc, clk, rst
    );

	always @(*)
		begin
			if (rst)
				begin
					ALUresult <= 0;
					zero <= 0;
				end
			else 
				begin 
					case (ALUoperation)
						4'b0010:
							begin 
								if (ALUsrc)
									begin
										ALUresult <= Rs_Data + ext_imm;
									end
								else
									begin
										ALUresult <= Rs_Data + Rt_Data;
									end
							end
						4'b0110:
							begin 
								if (ALUsrc)
									begin
										ALUresult <= Rs_Data - ext_imm;
									end
								else
									begin
										ALUresult <= Rs_Data - Rt_Data;
									end
							end
						4'b0000: ALUresult <= Rs_Data & Rt_Data;
						4'b0001: ALUresult <= Rs_Data | Rt_Data;
						4'b0111: ALUresult <= (Rs_Data < Rt_Data) ? 1 : 0;
						default: ALUresult <= 0;
					endcase
					
					if (ALUresult == 0)
						begin
							zero <= 1;
						end
					else
						begin
							zero <= 0;
						end
				end
		end
endmodule 


module EXE_MEM_reg(
	 output reg Branch_regOut,
    output reg MemWrite_regOut,
    output reg MemRead_regOut,
    output reg MemToReg_regOut,
    output reg RegWriteFromCS_regOut,
    output reg zeroOut,
    output reg [4:0] DestinationOut,
	 output reg [31:0] branchAddOut, ALUresultOut, Rt_Data_out,
	 input Branch_regIn,
    input MemWrite_regIn,
    input MemRead_regIn,
    input MemToReg_regIn,
    input RegWriteFromCS_regIn,
    input zeroIn,
    input [4:0] DestinationIn,
	 input [31:0] branchAddIn, ALUresultIn, Rt_Data_in,
	 input clk, rst
    );

	always@(posedge clk or posedge rst)
		begin 
			if (rst)
				begin
					 Branch_regOut <= 0;
					 MemWrite_regOut <= 0;
					 MemRead_regOut <= 0;
					 MemToReg_regOut <= 0;
					 RegWriteFromCS_regOut <= 0;
					 zeroOut <= 0;
					 DestinationOut <= 0;
					 branchAddOut <= 0;
					 ALUresultOut <= 0;
					 Rt_Data_out <= 0;		
				end
			else 
				begin
					 Branch_regOut <= Branch_regIn;
					 MemWrite_regOut <= MemWrite_regIn;
					 MemRead_regOut <= MemRead_regIn;
					 MemToReg_regOut <= MemToReg_regIn;
					 RegWriteFromCS_regOut <= RegWriteFromCS_regIn;
					 zeroOut <= zeroIn;
					 DestinationOut <= DestinationIn;
					 branchAddOut <= branchAddIn;
					 ALUresultOut <= ALUresultIn;
					 Rt_Data_out <= Rt_Data_in;	
				end
		end
endmodule



module EXE(
	 output Branch_regOut,
    output MemWrite_regOut,
    output MemRead_regOut,
    output MemToReg_regOut,
    output RegWriteFromCS_regOut,
    output zero,
    output [4:0] Destination,
	 output [31:0] branchAddOut, ALUresult, Rt_Data_out,
	 input [31:0] PC_out_reg_out,
    input [31:0] Rs_Data_reg,
    input [31:0] Rt_Data_reg,
    input [31:0] ext_imm_reg,
    input [4:0] rt_add_reg,
    input [4:0] rd_add_reg,			//rd_add or rt_add(destination) is decided in exe stage but then goes agay and agay and comes from wb stage
    input [1:0] ALUop_reg,
    input RegDst_reg,
    input ALUsrc_reg,
    input Branch_reg,
    input MemWrite_reg,
    input MemRead_reg,
    input MemToReg_reg,
    input RegWriteFromCS_reg, clk, rst
    );

	// Internal signals
	wire [3:0] ALUoperation;
	wire [31:0] branch_address;
	wire [31:0] ALUresult_temp;
	wire zeroIn;
	wire [4:0] DestinationIn;

	// ALU Control
	ALUcontrol alu_control (
		.ALUoperation(ALUoperation),
		.ALUop_controlSignals(ALUop_reg),
		.funct(ext_imm_reg[5:0]),
		.clk(clk),
		.rst(rst)
	);

	// ALU
	ALU alu (
		.ALUresult(ALUresult_temp),
		.zero(zeroIn),
		.Rs_Data(Rs_Data_reg),
		.Rt_Data(Rt_Data_reg),
		.ext_imm(ext_imm_reg),	
		.ALUoperation(ALUoperation),
		.ALUsrc(ALUsrc_reg)
	);

	// Branch Address Calculation
	PC_branch pc_branch (
		.branchAddOut(branch_address),
		.addFromPC(PC_out_reg_out),
		.branch_add(ext_imm_reg),
		.clk(clk),
		.rst(rst)
	);

	// Destination Register Selection
	assign DestinationIn = (RegDst_reg) ? rd_add_reg : rt_add_reg;

	// Registering Outputs
	EXE_MEM_reg exe_mem_reg (
		.Branch_regOut(Branch_regOut),
		.MemWrite_regOut(MemWrite_regOut),
		.MemRead_regOut(MemRead_regOut),
		.MemToReg_regOut(MemToReg_regOut),
		.RegWriteFromCS_regOut(RegWriteFromCS_regOut),
		.zeroOut(zero),
		.DestinationOut(Destination),
		.branchAddOut(branchAddOut),
		.ALUresultOut(ALUresult),
		.Rt_Data_out(Rt_Data_out),
		.Branch_regIn(Branch_reg),
		.MemWrite_regIn(MemWrite_reg),
		.MemRead_regIn(MemRead_reg),
		.MemToReg_regIn(MemToReg_reg),
		.RegWriteFromCS_regIn(RegWriteFromCS_reg),
		.zeroIn(zeroIn),
		.DestinationIn(DestinationIn),
		.branchAddIn(branch_address),
		.ALUresultIn(ALUresult_temp),
		.Rt_Data_in(Rt_Data_reg),
		.clk(clk),
		.rst(rst)
	);


endmodule


//temp = regDst ? rd_add : rt_add;