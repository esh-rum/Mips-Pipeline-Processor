module WB(
	 output reg [31:0] writeData,		// to be written in reg File
	 output reg regWriteOut,
	 output reg [4:0] Destination_out,
	 input MemToReg, regWriteIn,	// from EXE/WB reg
	 input [31:0] memData, 	// data read from data memory
	 input [31:0] ALUreseult,	// alu result thats supposed to be stored in reg File, in prev stage it in used as an address for data memory
	 input [4:0] Destination_in,	// for destination in reg file
	 input clk, rst
    );
	 
	 always @(posedge clk or posedge rst)
		begin
			if(MemToReg)
				begin
					writeData <= ALUreseult;		// accidently did the typo, the whole code was done when noticed so we are gonna roll with it now
				end
			else
				begin
					writeData <= memData;
				end
		end
	 
	 always@ (posedge clk or posedge rst)
		begin
			if (rst)
				begin
					writeData <= 0;
					regWriteOut <= 0;
					Destination_out <= 0;
				end
			else
				begin
					regWriteOut <= regWriteIn;
					Destination_out <= Destination_in;
				end
				
		
		end
endmodule
