`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   13:33:36 05/20/2024
// Design Name:   main
// Module Name:   E:/Solved lab tasks/ISE/PipelineProcessor/TBtake2.v
// Project Name:  PipelineProcessor
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: main
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TBtake2;

	// Inputs
	reg clk;
	reg rst;

	// Outputs
	wire [31:0] instr_out_reg_IF;
	wire [31:0] PC_out_reg_IF;
	wire [31:0] PC_out_reg_ID;
	wire [31:0] Rs_Data_reg_ID;
	wire [31:0] Rt_Data_reg_ID;
	wire [31:0] ext_imm_reg_ID;
	wire [4:0] rt_add_reg_ID;
	wire [4:0] rd_add_reg_ID;
	wire [1:0] ALUop_reg_ID;
	wire RegDst_reg_ID;
	wire ALUsrc_reg_ID;
	wire Branch_reg_ID;
	wire MemWrite_reg_ID;
	wire MemRead_reg_ID;
	wire MemToReg_reg_ID;
	wire RegWriteFromCS_reg_ID;
	wire [25:0] add_jType_ID;///
	wire Jump_reg_ID;///
	wire Branch_reg_EXE;
	wire MemWrite_reg_EXE;
	wire MemRead_reg_EXE;
	wire MemToReg_reg_EXE;
	wire RegWriteFromCS_reg_EXE;
	wire zero_EXE;
	wire [4:0] Destination_EXE;
	wire [31:0] branchAddOut_EXE;
	wire [31:0] ALUresult_EXE;
	wire [31:0] Rt_Data_out_EXE;
	wire [31:0] branch_out_MEM;
	wire [31:0] ALUresultOut_MEM;
	wire [31:0] readData_MEM;
	wire PCsrc_MEM;
	wire regWriteOut_MEM;
	wire MemToRegOut_MEM;
	wire [4:0] Destination_out_MEM;
	wire [31:0] writeData_WB;
	wire regWriteOut_WB;
	wire [4:0] Destination_out_WB;

	// Instantiate the Unit Under Test (UUT)
	main uut (
		.instr_out_reg_IF(instr_out_reg_IF), 
		.PC_out_reg_IF(PC_out_reg_IF), 
		.PC_out_reg_ID(PC_out_reg_ID), 
		.Rs_Data_reg_ID(Rs_Data_reg_ID), 
		.Rt_Data_reg_ID(Rt_Data_reg_ID), 
		.ext_imm_reg_ID(ext_imm_reg_ID), 
		.rt_add_reg_ID(rt_add_reg_ID), 
		.rd_add_reg_ID(rd_add_reg_ID), 
		.ALUop_reg_ID(ALUop_reg_ID), 
		.RegDst_reg_ID(RegDst_reg_ID), 
		.ALUsrc_reg_ID(ALUsrc_reg_ID), 
		.Branch_reg_ID(Branch_reg_ID), 
		.MemWrite_reg_ID(MemWrite_reg_ID), 
		.MemRead_reg_ID(MemRead_reg_ID), 
		.MemToReg_reg_ID(MemToReg_reg_ID), 
		.RegWriteFromCS_reg_ID(RegWriteFromCS_reg_ID), 
		.add_jType_ID(add_jType_ID),
	   .Jump_reg_ID(Jump_reg_ID),
		.Branch_reg_EXE(Branch_reg_EXE), 
		.MemWrite_reg_EXE(MemWrite_reg_EXE), 
		.MemRead_reg_EXE(MemRead_reg_EXE), 
		.MemToReg_reg_EXE(MemToReg_reg_EXE), 
		.RegWriteFromCS_reg_EXE(RegWriteFromCS_reg_EXE), 
		.zero_EXE(zero_EXE), 
		.Destination_EXE(Destination_EXE), 
		.branchAddOut_EXE(branchAddOut_EXE), 
		.ALUresult_EXE(ALUresult_EXE), 
		.Rt_Data_out_EXE(Rt_Data_out_EXE), 
		.branch_out_MEM(branch_out_MEM), 
		.ALUresultOut_MEM(ALUresultOut_MEM), 
		.readData_MEM(readData_MEM), 
		.PCsrc_MEM(PCsrc_MEM), 
		.regWriteOut_MEM(regWriteOut_MEM), 
		.MemToRegOut_MEM(MemToRegOut_MEM), 
		.Destination_out_MEM(Destination_out_MEM), 
		.writeData_WB(writeData_WB), 
		.regWriteOut_WB(regWriteOut_WB), 
		.Destination_out_WB(Destination_out_WB), 
		.clk(clk), 
		.rst(rst)
	);

	always #20 clk = ~clk;

	initial begin
		// Initialize Inputs
		clk = 1;
		rst = 1;
		
		#40;
		
		rst = 0;
		
		#50;

	end
      
endmodule

