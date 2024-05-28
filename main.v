module main(
	 output [31:0] instr_out_reg_IF,
    output [31:0] PC_out_reg_IF,
    output [31:0] PC_out_reg_ID,
    output [31:0] Rs_Data_reg_ID,
    output [31:0] Rt_Data_reg_ID,
    output [31:0] ext_imm_reg_ID,
    output [4:0] rt_add_reg_ID,
    output [4:0] rd_add_reg_ID,
    output [1:0] ALUop_reg_ID,
    output RegDst_reg_ID,
    output ALUsrc_reg_ID,
    output Branch_reg_ID,
    output MemWrite_reg_ID,
    output MemRead_reg_ID,
    output MemToReg_reg_ID,
    output RegWriteFromCS_reg_ID,
	 output [25:0] add_jType_ID,///
	 output Jump_reg_ID,///
    output Branch_reg_EXE,
    output MemWrite_reg_EXE,
    output MemRead_reg_EXE,
    output MemToReg_reg_EXE,
    output RegWriteFromCS_reg_EXE,
    output zero_EXE,
    output [4:0] Destination_EXE,
    output [31:0] branchAddOut_EXE,
    output [31:0] ALUresult_EXE,
    output [31:0] Rt_Data_out_EXE,
    output [31:0] branch_out_MEM,
    output [31:0] ALUresultOut_MEM,
    output [31:0] readData_MEM,
    output PCsrc_MEM,
    output regWriteOut_MEM,
    output MemToRegOut_MEM,
    output [4:0] Destination_out_MEM,
    output [31:0] writeData_WB,
    output regWriteOut_WB,
    output [4:0] Destination_out_WB,
    input clk,
    input rst
);

    // Signals for connecting modules
    /*wire [31:0] instr_out_reg_IF;
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
*/
    // Instantiate modules
    IF if_module (
        .instr_out_reg(instr_out_reg_IF),
        .PC_out_reg(PC_out_reg_IF),
        .branch_add(branch_out_MEM),
		  .jump_add(add_jType_ID),
        .PCSrc(PCsrc_MEM),
		  .Jump(Jump_reg_ID),
        .clk(clk),
        .rst(rst) 
    );

    ID id_module (
        .PC_out_reg_out(PC_out_reg_ID),
        .Rs_Data_reg(Rs_Data_reg_ID),
        .Rt_Data_reg(Rt_Data_reg_ID),
        .ext_imm_reg(ext_imm_reg_ID),
        .rt_add_reg(rt_add_reg_ID),
        .rd_add_reg(rd_add_reg_ID),
        .ALUop_reg(ALUop_reg_ID),
		  .add_jType_out(add_jType_ID),
		  .Jump_reg(Jump_reg_ID),
        .RegDst_reg(RegDst_reg_ID),
        .ALUsrc_reg(ALUsrc_reg_ID),
        .Branch_reg(Branch_reg_ID),
        .MemWrite_reg(MemWrite_reg_ID),
        .MemRead_reg(MemRead_reg_ID),
        .MemToReg_reg(MemToReg_reg_ID),
        .RegWriteFromCS_reg(RegWriteFromCS_reg_ID),
        .regWriteFromWB(regWriteOut_WB),
        .writeData(writeData_WB),
        .Destination(Destination_out_WB),
        .instr_out_reg(instr_out_reg_IF),
        .PC_out_reg_in(PC_out_reg_IF),
        .clk(clk),
        .rst(rst)
    );

    EXE exe_module (
        .Branch_regOut(Branch_reg_EXE),
        .MemWrite_regOut(MemWrite_reg_EXE),
        .MemRead_regOut(MemRead_reg_EXE),
        .MemToReg_regOut(MemToReg_reg_EXE),
        .RegWriteFromCS_regOut(RegWriteFromCS_reg_EXE),
        .zero(zero_EXE),
        .Destination(Destination_EXE),
        .branchAddOut(branchAddOut_EXE),
        .ALUresult(ALUresult_EXE),
        .Rt_Data_out(Rt_Data_out_EXE),
        .PC_out_reg_out(PC_out_reg_ID),
        .Rs_Data_reg(Rs_Data_reg_ID),
        .Rt_Data_reg(Rt_Data_reg_ID),
        .ext_imm_reg(ext_imm_reg_ID),
        .rt_add_reg(rt_add_reg_ID),
        .rd_add_reg(rd_add_reg_ID),
        .ALUop_reg(ALUop_reg_ID),
        .RegDst_reg(RegDst_reg_ID),
        .ALUsrc_reg(ALUsrc_reg_ID),
        .Branch_reg(Branch_reg_ID),
        .MemWrite_reg(MemWrite_reg_ID),
        .MemRead_reg(MemRead_reg_ID),
        .MemToReg_reg(MemToReg_reg_ID),
        .RegWriteFromCS_reg(RegWriteFromCS_reg_ID),
        .clk(clk),
        .rst(rst)
    );

    MEM mem_module (
        .branch_out(branch_out_MEM),
        .ALUresultOut(ALUresultOut_MEM),
        .readData(readData_MEM),
        .PCsrc(PCsrc_MEM),
        .regWriteOut(regWriteOut_MEM),
        .MemToRegOut(MemToRegOut_MEM),
        .Destination_out(Destination_out_MEM),
        .branch_in(branchAddOut_EXE),
        .ALUresult(ALUresult_EXE),
        .Rt_data(Rt_Data_out_EXE),
        .Destination_in(Destination_EXE),
        .memRead(MemRead_reg_EXE),
        .memWrite(MemWrite_reg_EXE),
        .zero(zero_EXE),
        .branch(Branch_reg_EXE),
        .regWriteIn(RegWriteFromCS_reg_EXE),
        .MemToRegIn(MemToReg_reg_EXE),
        .clk(clk),
        .rst(rst)
    );

    WB wb_module (
        .writeData(writeData_WB),
        .regWriteOut(regWriteOut_WB),
        .Destination_out(Destination_out_WB),
        .MemToReg(MemToRegOut_MEM),
        .regWriteIn(RegWriteFromCS_reg_EXE),
        .memData(readData_MEM),
        .ALUreseult(ALUresultOut_MEM),
        .Destination_in(Destination_EXE),
        .clk(clk),
        .rst(rst)
    );

endmodule