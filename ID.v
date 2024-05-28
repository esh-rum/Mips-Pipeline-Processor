module decoder(
	output reg [25:0] add_jType,///
	output reg [15:0] imm_val_iType,
	output reg [5:0] opcode_instr,	
	output reg [5:0] funct_instr,	
	output reg [4:0] rs_add,	
	output reg [4:0] rt_add,	
	output reg [4:0] rd_add,	
	output reg [4:0] shamt,
	input [31:0] instr_out_reg,	// output from IF/ID reg
	input clk, rst
    );

	always @(*)
		begin
			if(rst)
				begin
					add_jType = 0;
					imm_val_iType <= 0;
					opcode_instr <= 0;
					funct_instr <= 0;
					rs_add <= 0;
					rt_add <= 0;
					rd_add <= 0;
					shamt <= 0;
				end
			else
				begin 
					add_jType = instr_out_reg[25:0];
					imm_val_iType <= instr_out_reg[15:0];
					opcode_instr <= instr_out_reg[31:26];
					funct_instr <= instr_out_reg[5:0];
					rs_add <= instr_out_reg[25:21];
					rt_add <= instr_out_reg[20:16];
					rd_add <= instr_out_reg[15:11];
					shamt <= instr_out_reg[10:6];
				end
		end
endmodule


module controlSignals(
	output reg RegDst,
	output reg ExtOp,
	output reg ALUsrc,
	output reg Branch,
	output reg Jump,///
	output reg MemWrite,
	output reg MemRead,
	output reg MemToReg,
	output reg RegWriteFromCS,
	output reg [1:0]ALUop,
	input[5:0] opcode_instr,
	input clk, rst
    );

	always @(*)
		begin
			if(rst)
				begin 
					ALUop <= 2'b00;
					RegDst <= 0;
					ExtOp <= 0;	
					ALUsrc <= 0;
					Branch <= 0;
					Jump <= 0;
					MemWrite <= 0;
					MemRead <= 0;
					MemToReg <= 0;
					RegWriteFromCS <= 0; 
				end
			else
				begin
					case(opcode_instr)
						6'b100011:		// lw
							begin
								ALUop <= 2'b00;
								RegDst <= 0;
								ExtOp <= 1;	
								ALUsrc <= 1;
								Branch <= 0;
								Jump <= 0;
								MemWrite <= 0;
								MemRead <= 1;
								MemToReg <= 1;
								RegWriteFromCS <= 1; 
							end
						6'b101011:		//sw
							begin
								ALUop <= 2'b00;
								RegDst <= 0;
								ExtOp <= 1;
								ALUsrc <= 1;
								Branch <= 0;
								Jump <= 0;
								MemWrite <= 1;
								MemRead <= 0;
								MemToReg <= 0;
								RegWriteFromCS <= 0; 
							end
						6'b000100:		// beq
							begin
								ALUop <= 2'b01;
								RegDst <= 0;
								ExtOp <= 1;
								ALUsrc <= 0;
								Branch <= 1;
								Jump <= 0;
								MemWrite <= 0;	
								MemRead <= 0;
								MemToReg <= 0;
								RegWriteFromCS <= 0;
							end
						6'b000000:		// R-type
							begin
								ALUop <= 2'b10;
								RegDst <= 1;
								ExtOp <= 0;	
								ALUsrc <= 0;
								Branch <= 0;
								Jump <= 0;
								MemWrite <= 0;
								MemRead <= 0;
								MemToReg <= 1;
								RegWriteFromCS <= 1; 
							end
						6'b000010:
							begin
								ALUop = 2'b00;
								RegDst = 0;
								ExtOp = 0;	
								ALUsrc = 0;
								Branch = 0;
								Jump = 1;
								MemWrite = 0;
								MemRead = 0;
								MemToReg = 0;
								RegWriteFromCS = 0; 
							end
						default:
							begin
								ALUop <= 2'b01;
								RegDst <= 0;
								ExtOp <= 1;	
								ALUsrc <= 0;
								Branch <= 1;
								Jump <= 0;
								MemWrite <= 0;	
								MemRead <= 0;
								MemToReg <= 0;
								RegWriteFromCS <= 0;
							end
					endcase
				end
		end
endmodule


module regFile(
	output [31:0] Rs_Data, Rt_Data, 		// will be stored in ID/EXE reg
	output reg [31:0] Rd_data,
	input regWriteFromWB, regDst, extOp, clk, rst,
	input [4:0] rs_add, rt_add,
	input [31:0] writeData,			// is gonna come from wb stage
	input [4:0] Destination			// is found in EXE but comes from WB
    );
	//these inputs are coming from decoder except writeData
	reg [31:0] regFile [0:31];
	
	initial 
		begin
			regFile[0] = 32'h00000000; 
			regFile[1] = 32'h00000001; 
			regFile[2] = 32'h00000002;
			regFile[3] = 32'h00000003;
			regFile[4] = 32'h00000004;
			regFile[5] = 32'h00000005;
			regFile[6] = 32'h00000006;
			regFile[7] = 32'h00000007;
			regFile[8] = 32'h00000008;
			regFile[9] = 32'h00000009;
			regFile[10] = 32'h0000000A;
			regFile[11] = 32'h0000000B;
			regFile[12] = 32'h0000000C;
			regFile[13] = 32'h0000000D;
			regFile[14] = 32'h0000000E;
			regFile[15] = 32'h0000000F;
			regFile[16] = 32'h00000011;
			regFile[17] = 32'h00000012;
			regFile[18] = 32'h00000013;
			regFile[19] = 32'h00000014;
			regFile[20] = 32'h00000015;
			regFile[21] = 32'h00000016;
			regFile[22] = 32'h00000017;
			regFile[23] = 32'h00000018;
			regFile[24] = 32'h00000019;
			regFile[25] = 32'h0000001A;
			regFile[26] = 32'h0000001B;
			regFile[27] = 32'h0000001C;
			regFile[28] = 32'h0000001D;
			regFile[29] = 32'h0000001E;
			regFile[30] = 32'h0000001F;
			regFile[31] = 32'h00000010;
		end

	assign Rs_Data = regFile[rs_add];
	assign Rt_Data = regFile[rt_add];

	always @(posedge clk or posedge rst)
	begin
		if (rst)
			begin
				Rd_data <= 32'b0;
			end
		else
			begin
				//Rs_Data <= regFile[rs_add];
				//Rt_Data <= regFile[rt_add];
				if (regWriteFromWB)
					begin
						regFile[Destination] <= writeData;
						//Rd_data <= regFile[Destination];			// just shows what is being stored at rd_add in regFile
					end
			end
	end
endmodule


module ext_immediate(
	 output reg [31:0] ext_imm,
	 input [15:0] imm_val_iType,
	 input extOp,
	 input clk, rst
    );

	always @(*) 		
		begin
			if (rst)
				begin
					ext_imm <= 0;
				end
			else 
				begin 
					if (extOp) 
						begin		//sign extension
							ext_imm <= { {16{imm_val_iType[15]}}, imm_val_iType};
						end
					else 
						begin		//zero extension
							ext_imm <= {16'b0, imm_val_iType};
						end
				end
		end
endmodule



module ID_EXE_reg(
	output reg RegDst_reg,
	output reg ALUsrc_reg,
	output reg Branch_reg,
	output reg Jump_reg,///
	output reg MemWrite_reg,
	output reg MemRead_reg,
	output reg MemToReg_reg,
	output reg RegWriteFromCS_reg,
	output reg [1:0] ALUop_reg,	
	output reg [4:0] rt_add_reg,	
	output reg [4:0] rd_add_reg,
	output reg [31:0] Rs_Data_reg, Rt_Data_reg,
	output reg [31:0] ext_imm_reg,
	output reg [31:0] PC_out_reg_out, 
	output reg [25:0] add_jType_out,///
	input RegDst,
	input ALUsrc,
	input Branch,
	input Jump,///
	input MemWrite,
	input MemRead,
	input MemToReg,
	input RegWriteFromCS,
	input [1:0] ALUop,	
	input [4:0] rt_add,	
	input [4:0] rd_add,
	input [31:0] Rs_Data, Rt_Data,
	input [31:0] ext_imm,
	input [31:0] PC_out_reg_in, 
	input [25:0] add_jType_in,///
	input clk, rst
    );
	 
	 always@(posedge clk or posedge rst)
		begin 
			if (rst)
				begin
					RegDst_reg <= 0; 
					ALUsrc_reg <= 0;
					Branch_reg <= 0;
					Jump_reg <= 0;
					MemWrite_reg <= 0;
					MemRead_reg <= 0;
					MemToReg_reg <= 0;
					RegWriteFromCS_reg <= 0;
					ALUop_reg <= 0;
					rt_add_reg <= 0;
					rd_add_reg <= 0;
					Rs_Data_reg <= 0;
					Rt_Data_reg <= 0;
					ext_imm_reg <= 0;
					PC_out_reg_out <= 0;	
					add_jType_out <= 0;
				end
			else 
				begin
					RegDst_reg <= RegDst; 
					ALUsrc_reg <= ALUsrc;
					Branch_reg <= Branch;
					Jump_reg <= Jump;
					MemWrite_reg <= MemWrite;
					MemRead_reg <= MemRead;
					MemToReg_reg <= MemToReg;
					RegWriteFromCS_reg <= RegWriteFromCS;
					ALUop_reg <= ALUop;
					rt_add_reg <= rt_add;
					rd_add_reg <= rd_add;
					Rs_Data_reg <= Rs_Data;
					Rt_Data_reg <= Rt_Data;
					ext_imm_reg <= ext_imm;
					PC_out_reg_out <= PC_out_reg_in;	
					add_jType_out <= add_jType_in;
				end
		end
endmodule


module ID(
    output [31:0] PC_out_reg_out,
    output [31:0] Rs_Data_reg,
    output [31:0] Rt_Data_reg,
    output [31:0] ext_imm_reg,
    output [4:0] rt_add_reg,
    output [4:0] rd_add_reg,			//rd_add or rt_add(destination) is decided in exe stage but then goes agay and agay and comes from wb stage
    output [1:0] ALUop_reg,
	 output [25:0] add_jType_out,///
	 output Jump_reg,///
    output RegDst_reg,
    output ALUsrc_reg,
    output Branch_reg,
    output MemWrite_reg,
    output MemRead_reg,
    output MemToReg_reg,
    output RegWriteFromCS_reg,
	 input regWriteFromWB, 				// in main this will be equal to regWritefromCS output of WB
	 input [31:0] writeData,			// is gonna come from wb stage
	 input [4:0] Destination, 			// is found in EXE but comes from WB
    input [31:0] instr_out_reg, // output from IF/ID reg
    input [31:0] PC_out_reg_in, // comes from IF/ID reg, irrelevant here tho, not even sure if it should be written but its in the picture sooooo
    input clk, 
    input rst
    );

    // Internal signals
    wire [5:0] opcode_instr;
    wire [5:0] funct_instr;
    wire [4:0] rs_add;
    wire [4:0] rt_add;
    wire [4:0] rd_add;
    wire [4:0] shamt;
    wire [15:0] imm_val_iType;
    wire [31:0] Rs_Data;
    wire [31:0] Rt_Data;
    wire [31:0] Rd_Data;
    wire [31:0] ext_imm;
	 wire [25:0] add_jType;
    wire RegDst;
    wire ExtOp;
    wire ALUsrc;
    wire Branch;
    wire MemWrite;
    wire MemRead;
    wire MemToReg;
    wire RegWriteFromCS, jump;
    wire [1:0] ALUop;
    
    // Instantiate the decoder
    decoder decode (
        .add_jType(add_jType),
		  .imm_val_iType(imm_val_iType),
        .opcode_instr(opcode_instr),
        .funct_instr(funct_instr),
        .rs_add(rs_add),
        .rt_add(rt_add),
        .rd_add(rd_add),
        .shamt(shamt),
        .instr_out_reg(instr_out_reg),
        .clk(clk),
        .rst(rst)
    );

    // Instantiate the control signals
    controlSignals control (
        .RegDst(RegDst),
        .ExtOp(ExtOp),
        .ALUsrc(ALUsrc),
        .Branch(Branch),
		  .Jump(jump),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .MemToReg(MemToReg),
        .RegWriteFromCS(RegWriteFromCS),
        .ALUop(ALUop),
        .opcode_instr(opcode_instr),
        .clk(clk),
        .rst(rst)
    );

    // Instantiate the register file
    regFile registers (
        .Rs_Data(Rs_Data),
        .Rt_Data(Rt_Data),
        .Rd_data(Rd_data), 
        .regWriteFromWB(regWriteFromWB), 
        .regDst(RegDst),
        .extOp(ExtOp),
        .clk(clk),
        .rst(rst),
        .rs_add(rs_add),
		  .rt_add(rt_add), 
        .writeData(writeData), 
        .Destination(Destination)
    );

    // Instantiate the immediate extender
    ext_immediate extender (
        .ext_imm(ext_imm),
        .imm_val_iType(imm_val_iType),
		  .extOp(ExtOp),
        .clk(clk),
        .rst(rst)
    );

    // Instantiate the ID/EX register
    ID_EXE_reg reg2 (
        .RegDst_reg(RegDst_reg),
        .ALUsrc_reg(ALUsrc_reg),
        .Branch_reg(Branch_reg),
		  .Jump_reg(Jump_reg),
        .MemWrite_reg(MemWrite_reg),
        .MemRead_reg(MemRead_reg),
        .MemToReg_reg(MemToReg_reg),
        .RegWriteFromCS_reg(RegWriteFromCS_reg),
        .ALUop_reg(ALUop_reg),
        .rt_add_reg(rt_add_reg),
        .rd_add_reg(rd_add_reg),
        .Rs_Data_reg(Rs_Data_reg),
        .Rt_Data_reg(Rt_Data_reg),
        .ext_imm_reg(ext_imm_reg),
        .PC_out_reg_out(PC_out_reg_out),
		  .add_jType_out(add_jType_out),
        .RegDst(RegDst),
        .ALUsrc(ALUsrc),
        .Branch(Branch),
		  .Jump(jump),
        .MemWrite(MemWrite),
        .MemRead(MemRead),
        .MemToReg(MemToReg),
        .RegWriteFromCS(RegWriteFromCS),
        .ALUop(ALUop),
        .rt_add(rt_add),
        .rd_add(rd_add),
        .Rs_Data(Rs_Data),
        .Rt_Data(Rt_Data),
        .ext_imm(ext_imm),
        .PC_out_reg_in(PC_out_reg_in),
		  .add_jType_in(add_jType),
        .clk(clk),
        .rst(rst)
    );

endmodule


/* 
SIGNALS FROM CONTROL
regWrite, comes from WB but is used in ID
extOp, used in ID only
ALUop, used in EXE
ALUsrc, used in EXE
RegDst, used in EXE
branch, used in MEM
memRead, used in MEM
memWrite, used in MEM
memToReg, used in WB

EXTRA SIGNALS
PCsrc, generated in EXE but is used in IF
zero, generated from ALU in EXE but used in MEM
*/