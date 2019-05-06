module mipsTestbench;
	reg clk,reset;
	wire [31:0] pc_out, alu_result;

	mipsFull mips1(clk,reset,pc_out, alu_result);
	
	initial begin
		clk=1'b0;
		forever begin
			#5 clk=~clk;
		end
	end
	initial begin 
		$shm_open("shm.db",1);
		$shm_probe("AS");
		#700 $finish;
		#750 $shm_close();
	end
	initial begin 
		#0 reset <=1;
		#5 reset<=0;
	end

endmodule 
