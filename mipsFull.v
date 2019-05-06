module mipsFull(input clk,reset,output[31:0] pc_out, alu_result);
	
	
	reg [31:0] PC;
	wire [31:0] pc_next,signExOut;
	wire [31:0] Instruction,WriteData,ReadData1,ReadData2,ALUin,ALUResult,MemoryData;
	wire RegDst,Branch,BranchTest,MemRead,Jump,MemtoReg,MemWrite,ALUSrc,RegWrite,zero;
	wire [1:0] ALUOp;
	wire [3:0] ALUCon;
	wire [4:0] WriteRegister;
	
	always @(posedge clk or posedge reset)  
	 begin   
		  if(reset)   
			   PC <= 0;  
		  else  
			   PC <= pc_next;  
	 end
	
	//calling ROM module;
	InstructionMemory InstructionMemory1(.ReadAddress(PC),.Instruction(Instruction));
	
	//calling Control module;
	control control1(.reset(reset),.instruction5(Instruction[31:26]),.RegDst(RegDst),.Branch(Branch),
						.Jump(Jump),.MemRead(MemRead),.MemtoReg(MemtoReg),.ALUOp(ALUOp),
						.MemWrite(MemWrite),.ALUSrc(ALUSrc),.RegWrite(RegWrite),.BranchTest(BranchTest));
	
	//if R type operation, RegDest is 1 and destination register is assigned, else the second operand register is assigned
	assign WriteRegister=RegDst?Instruction[15:11]:Instruction[20:16];
	
	//calling Registers module		
	Registers registers1(.clk(clk),.ReadRegister1(Instruction[25:21]),.ReadRegister2(Instruction[20:16]),
						.WriteRegister(WriteRegister),.WriteData(WriteData),.ReadData1(ReadData1),
						.ReadData2(ReadData2),.RegWrite(RegWrite),.reset(reset));
	
	//calling ALUControl module
	ALUControl ALUControl1(.instruction1(Instruction[5:0]),.ALUop(ALUOp),.ALUCon(ALUCon));
	
	//sign extending for I and L type operations to convert 16 bit offset/constant to 32 bits
	assign signExOut={{16{1'b0}},Instruction[15:0]};
	
	//2-to-1 mus following the Register module, for Branch and L type operations
	mux32 mux32ALU(.D0(ReadData2),.D1(signExOut),.select(ALUSrc),.Out(ALUin));
	
	//calling ALU module
	ALU ALU1(.A(ReadData1),.B(ALUin),.ALUCon(ALUCon),.Zero(zero),.ALUResult(ALUResult));
	
	//calling DataMemory module for L type operation
	DataMemory DataMemory1(.clk(clk),.Address(ALUResult),.WriteData(ReadData2),.ReadData(MemoryData),
							.MemRead(MemRead),.MemWrite(MemWrite),.reset(reset));
							
	//2-to-1 mux following the DataMemory module, chooses either ALUResult or data from Memory
	mux32 mux32DataMem(.D0(ALUResult),.D1(MemoryData),.select(MemtoReg),.Out(WriteData));
	
	//calling PCModule
	pcModule pcModule1(.Zero(zero),.Branch(Branch),.BranchTest(BranchTest),.Jump(Jump),.PC(PC),.instruction1(signExOut),.instruction6(Instruction[25:0]),.pc_next(pc_next));
	
	//assigning outputs
	assign pc_out=pc_next;
	assign alu_result=ALUResult;

endmodule 

//ALU module 
module ALU(A,B,ALUCon,Zero,ALUResult);
	
	input [31:0] A,B;
	input [3:0] ALUCon;
	
	output reg [31:0] ALUResult;
	output reg Zero;
	
	always@(*)
	begin
		case(ALUCon)
			4'b0010: ALUResult=A+B;
			4'b0110: ALUResult=A-B;
			4'b0000: ALUResult=A & B;
			4'b0001: ALUResult=A | B;
			4'b0111: ALUResult=(A<B);
			4'b0011: ALUResult=~(A | B);
			default: ALUResult=A+B;
		endcase 
		Zero =(ALUResult==0);	
	end
endmodule 

//Register module
module Registers(clk,ReadRegister1,ReadRegister2,WriteRegister,WriteData,ReadData1,ReadData2,RegWrite,reset);
	
	input [4:0] ReadRegister1,ReadRegister2,WriteRegister;
	input [31:0] WriteData;
	input RegWrite,reset,clk;
	
	output [31:0] ReadData1,ReadData2;
	
	reg [31:0] regBank [8:0]; //defined only 9 registers, $t0 to $t8, each 32 bits as per specifications
	integer i;
	
	always@ (posedge clk)
	begin
	if(reset) 
		for(i=0;i<9;i=i+1)  
			regBank[i] <= 0; //initializes each register to zero @reset
    else
    	begin
		if(RegWrite) //if we need to write to register
			regBank[WriteRegister]<=WriteData;
		end
	end
	
	assign ReadData1=regBank[ReadRegister1]; //assigning values of the register banks as per the given adresses to outputs
	assign ReadData2=regBank[ReadRegister2];
	
endmodule 


//DataMemory module
module DataMemory(clk,Address,WriteData,ReadData,MemRead,MemWrite,reset);
	
	input[31:0] Address,WriteData;
	input MemRead,MemWrite,reset,clk;
	
	output reg [31:0] ReadData;
	
	integer i;  
	reg [31:0] ram [7:0]; //declaring 32 bit 8 ram cells
    wire [2:0] ramAddress = Address[2:0]; //3 bit adressing system
	
	always@(posedge clk)
	begin
		if(reset)
		begin 
		   	for(i=0;i<8;i=i+1)
		        ram[i] <=0; //initializes each value to zero
		end
		else 
		begin
			if(MemWrite)
				ram[ramAddress] <= WriteData; //writes to memory if MemWrite is 1
		end
		ReadData = (MemRead==1'b1) ? ram[ramAddress]:0; //outputs data from memory if MemRead is 1
	end
				
endmodule

//InstructionMemory module
module InstructionMemory(ReadAddress,Instruction);
	
	input[31:0] ReadAddress;
	
	output reg [31:0] Instruction;
    
    reg [31:0] rom [15:0]; //declaring only 16 rom cells
    
    wire [3:0] romAddress = ReadAddress[5:2]; //since PC output is in multiples of 4, the 2 LSB's are zero and
											//  we only need 4 bit to address all 16 rom cells
											//  so we only need ReadAddress[5:2]
      initial
      begin //loading the converted machine code for instruction set-1 into the memory
		rom[1]=32'hA0210000;
		rom[2]=32'hA0420001;
		rom[3]=32'hA0630000;
		rom[4]=32'hA0840014;
		rom[5]=32'h222820;
		rom[6]=32'hA0410000;
		rom[7]=32'hA0A20000;
		rom[8]=32'hA0630002;
		rom[9]=32'h10640001;
		rom[10]=32'h98000005;
		rom[11]=32'hAC850005;
		rom[12]=32'h85302A;
		rom[13]=32'b00000000000000000000000000000000;
		rom[14]=32'b00000000000000000000000000000000;
		rom[15]=32'b00000000000000000000000000000000;
		rom[0]=32'b00000000000000000000000000000000;  
      end 
	
	always@(* )
		Instruction<=rom[romAddress]; 
				
endmodule 

//32bit MUX module
module mux32(D0,D1,select,Out);
	
	input [31:0]D0,D1;
	input select;
	
	output reg [31:0] Out;
	
	always@(*)
		if(select)
			Out<=D1;
		else 
			Out<=D0;
endmodule

//PC module
module pcModule(input Zero,Branch,BranchTest,Jump,input [31:0]PC,instruction1,
				input[25:0] instruction6,output [31:0]pc_next);
		wire [31:0] pc2,pc3,pc4,pc5,pc;
		wire pc6;
		
		assign pc4=PC+4;
		assign pc5=pc4+(instruction1<<2);
		assign pc2={pc4[31:28],instruction6[25:0],{2{1'b0}}};
		assign pc6=(BranchTest^Zero) & Branch;
		mux32 m2(pc4,pc5,pc6,pc3);
		mux32 m1(pc3,pc2,Jump,pc_next);
endmodule 

//ALUControl module
module ALUControl(instruction1,ALUop,ALUCon);

	input [5:0] instruction1; //lower 6 bits (function) from instruction
	input [1:0]ALUop;	//output from Control module
	
	output reg [3:0] ALUCon; //outputs ALUCon for ALU module
	
	wire [7:0] ALUControlIn;  
	assign ALUControlIn = {ALUop,instruction1};
	
	always@(*)
	begin
		casex(ALUControlIn)
			8'b10100000: ALUCon=4'b0010;//ADD
			8'b10100010: ALUCon=4'b0110;//SUB
			8'b10100100: ALUCon=4'b0000;//AND
			8'b10100101: ALUCon=4'b0001;//OR
			8'b10101010: ALUCon=4'b0111;//SLT
			8'b10101111: ALUCon=4'b0011;//NOR
			8'b01xxxxxx: ALUCon=4'b0110;//BEQ & BNE
			8'b00xxxxxx: ALUCon=4'b0010;//lw sw jmp addi
			default: ALUCon=4'b0010;
		endcase
	end
	
endmodule 

//Control module
module control(reset,instruction5,RegDst,Branch,Jump,MemRead,MemtoReg,
				ALUOp,MemWrite,ALUSrc,RegWrite,BranchTest);
	input reset;
	input [5:0] instruction5;
	
	output reg RegDst,Branch,BranchTest,MemRead,Jump,MemtoReg,MemWrite,ALUSrc,RegWrite;
	output reg [1:0] ALUOp;
	
	always@(*)
	begin
		if(reset)
		  begin  
				RegDst=0;
				Branch=0;
				Jump=0;
				BranchTest=0;
				MemRead=0;
				MemtoReg=0;
				ALUOp=0;
				MemWrite=0;
				ALUSrc=0;
				RegWrite=0;	 
		  end  
		  else begin  
			  case(instruction5)
				6'b000000:begin	//instruction for R-format instruction opcode 000000
					RegDst=1;
					Branch=0;
					Jump=0;
					BranchTest=0;
					MemRead=0;
					MemtoReg=0;
					ALUOp=2'b10;
					MemWrite=0;
					ALUSrc=0;
					RegWrite=1;
				end
				6'b100011:begin	//instruction for lw 
					RegDst=0;
					Branch=0;
					Jump=0;
					BranchTest=0;
					MemRead=1;
					MemtoReg=1;
					ALUOp=2'b00;
					MemWrite=0;
					ALUSrc=1;
					RegWrite=1;
				end
				6'b101011:begin	//instruction for sw
					RegDst=0;
					Branch=0;
					Jump=0;
					BranchTest=0;
					MemRead=0;
					MemtoReg=0;
					ALUOp=2'b00;
					MemWrite=1;
					ALUSrc=1;
					RegWrite=0;	
				end
				6'b101000:begin	//instruction for addi
					RegDst=0;
					Branch=0;
					Jump=0;
					BranchTest=0;
					MemRead=0;
					MemtoReg=0;
					ALUOp=2'b00;
					MemWrite=0;
					ALUSrc=1;
					RegWrite=1;	
				end
				6'b100110:begin	//instruction for jump
					RegDst=0;
					Branch=0;
					Jump=1;
					BranchTest=0;
					MemRead=0;
					MemtoReg=0;
					ALUOp=0;
					MemWrite=0;
					ALUSrc=0;
					RegWrite=0;	
				end
				6'b000100:begin	//instruction for Beq
					RegDst=0;
					Branch=1;
					Jump=0;
					BranchTest=0;
					MemRead=0;
					MemtoReg=0;
					ALUOp=2'b01;
					MemWrite=0;
					ALUSrc=0;
					RegWrite=0;	
				end
				6'b000110:begin	//instruction for Bne
					RegDst=0;
					Branch=1;
					Jump=0;
					BranchTest=1;
					MemRead=0;
					MemtoReg=0;
					ALUOp=2'b01;
					MemWrite=0;
					ALUSrc=0;
					RegWrite=0;	
				end
				default :begin
					RegDst=0;
					Branch=0;
					Jump=0;
					MemRead=0;
					MemtoReg=0;
					ALUOp=0;
					MemWrite=0;
					ALUSrc=0;
					RegWrite=0;
				end
			endcase
		end
	end
endmodule 




