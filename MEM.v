module dataMemory(
	output reg [31:0] readData,							// output of mem stored in the MEM/WB reg 
	input [31:0] ALUresult, writeData,		// both are EXE stage outputs that were stored in the register
	input memWrite, memRead, clk, rst		// input from EXE/MEM reg
    );

	reg [31:0] dataMemory [0:31];
	integer i;
	
	initial 
		begin
			dataMemory[0] = 32'h00000000; 
			dataMemory[1] = 32'h00000001; 
			dataMemory[2] = 32'h00000002;
			dataMemory[3] = 32'h00000003;
			dataMemory[4] = 32'h00000004;
			dataMemory[5] = 32'h00000005;
			dataMemory[6] = 32'h00000006;
			dataMemory[7] = 32'h00000007;
			dataMemory[8] = 32'h00000008;
			dataMemory[9] = 32'h00000009;
			dataMemory[10] = 32'h0000000A;
			dataMemory[11] = 32'h0000000B;
			dataMemory[12] = 32'h0000000C;
			dataMemory[13] = 32'h0000000D;
			dataMemory[14] = 32'h0000000E;
			dataMemory[15] = 32'h0000000F;
			dataMemory[16] = 32'h00000011;
			dataMemory[17] = 32'h00000012;
			dataMemory[18] = 32'h00000013;
			dataMemory[19] = 32'h00000014;
			dataMemory[20] = 32'h00000015;
			dataMemory[21] = 32'h00000001;
			dataMemory[22] = 32'h00000010;
			dataMemory[23] = 32'h00000018;
			dataMemory[24] = 32'h00000019;
			dataMemory[25] = 32'h0000001A;
			dataMemory[26] = 32'h0000001B;
			dataMemory[27] = 32'h0000001C;
			dataMemory[28] = 32'h0000001D;
			dataMemory[29] = 32'h0000001E;
			dataMemory[30] = 32'h0000001F;
			dataMemory[31] = 32'h00000010;
		end
		
	always @(posedge clk or posedge rst) 
		begin
			if (rst) 
				begin
					readData <= 0;
				end
			else
				begin
					if (memWrite) 
						begin
							dataMemory[ALUresult] <= writeData;
						end
				end
		end
		
	always@(*)
		begin
			if (memRead)
				begin
					readData <= dataMemory[ALUresult];
				end
		end
endmodule


module MEM_WB_reg(
	 output reg [31:0] ALUresultOut, readDataOut,
	 output reg regWriteOut, MemToRegOut,
	 output reg [4:0] Destination_out,
	 input [31:0] ALUresultIn, readDataIn,
	 input regWriteIn, MemToRegIn,
	 input [4:0] Destination_in, clk, rst
    );
	 
	 always @(posedge clk or posedge rst)
		begin
			if (rst)
				begin
					 ALUresultOut <= 0;
					 readDataOut <= 0;
					 regWriteOut <= 0;
					 MemToRegOut <= 0;
					 Destination_out <= 0;
				end
			else
				begin
					 ALUresultOut <= ALUresultIn;
					 readDataOut <= readDataIn;
					 regWriteOut <= regWriteIn;
					 MemToRegOut <= MemToRegIn;
					 Destination_out <= Destination_in;
				end
		end
endmodule


module MEM(
	 output [31:0] branch_out, ALUresultOut, readData,		//branch_out is the branch address
	 output PCsrc, regWriteOut, MemToRegOut,
	 output [4:0] Destination_out,
	 input [31:0] branch_in, ALUresult, Rt_data,
	 input [4:0] Destination_in,
	 input memRead, memWrite, zero, branch, regWriteIn, MemToRegIn, clk, rst
    );
	 
	 wire [31:0] readDataMemory;
    wire memWriteSignal, memReadSignal;

    // Memory read/write logic
    dataMemory mem (
        .readData(readDataMemory),      // Output from data memory
        .ALUresult(ALUresult),          // Address to read/write from ALU result
        .writeData(Rt_data),            // Data to write to memory
        .memWrite(memWrite),            // Control signal for writing
        .memRead(memRead),              // Control signal for reading
        .clk(clk), 
        .rst(rst)
    );

    // Branch calculation and PCsrc determination
    assign PCsrc = branch && zero;
    assign branch_out = branch_in;

    // Propagate values to MEM/WB register
    MEM_WB_reg mem_wb (
        .ALUresultOut(ALUresultOut), 
        .readDataOut(readData), 
        .regWriteOut(regWriteOut), 
        .MemToRegOut(MemToRegOut), 
        .Destination_out(Destination_out),
        .ALUresultIn(ALUresult), 
        .readDataIn(readDataMemory), 
        .regWriteIn(regWriteIn), 
        .MemToRegIn(MemToRegIn), 
        .Destination_in(Destination_in), 
        .clk(clk), 
        .rst(rst)
    );
endmodule
