module Bayesian_log2 #(parameter Narray = 2, Nword = 3, M = 2**Nword, N = Narray+Nword) //Narray: 1D array adress size, Nword: storage adr size
(	input logic clk, 
	input logic CBL0, CBLEN0, CSL0, CWL0,	// .........
	input logic [1:0] instructions_in,		// // "11" form and prog, "10" read_mem, "01" read_reg, "00" inference
	input logic [N-1:0] adr_full_col_in,		//adr for array_col + mem_col  
	input logic [N-1:0] adr_full_row_in,		//adr for array_row + mem_row
	//output logic [2**Nword-1:0] DATA_out [2**Narray-1:0] 		//data out
	output logic DATA_out [2**Narray-1:0]
); 

// input registers 
	reg [1:0] instructions;
	reg [N-1:0] adr_full_col;
	reg [N-1:0] adr_full_row;
	reg CBL, CBLEN, CSL, CWL;
	wire [2**Nword-1:0] reg_out [2**Narray-1:0];

// Connecting wires              
	wire [Nword+3:0] reg_lcs [2**Narray-1:0];// output likelihood colomn registers
	wire [Nword + 2**Narray*2-1:0] reg_lrs;	// output likelihood row registers 

// control signals	
	logic inference, read_mem, prog, read_out;			// states decoded from instructions  
	reg inference_d, read_mem_d, prog_d, read_out_d;		// d for delayed 1 clk
//	logic CBL, CBLEN, CSL, CWL;


// store inputs on registers (buffers)
	always_ff @(posedge clk) begin
		instructions <= instructions_in;
		adr_full_col <= adr_full_col_in;
		adr_full_row <= adr_full_row_in;
		CBL <= CBL0; CBLEN <= CBLEN0; CSL <= CSL0; CWL <= CWL0;
		inference_d <= inference; 
		read_mem_d <= read_mem; 
		prog_d <= prog; 
		read_out_d <= read_out;
	end

// decode instructions 
	assign prog = instructions[1] & instructions[0];
	assign read_mem = instructions[1] & ~instructions[0]; 
	assign inference = ~instructions[1] & ~instructions[0];
	assign read_out = ~instructions[1] & instructions[0]; 



// Likelihood Array Decoders 
 
	array_decoder_top #(Nword, Narray , N , M) ADT(		 
					   clk, CBL,
					   CBLEN,
					   CSL,
					   adr_full_col,
					   read_out,
					   reg_lcs);

	array_decoder_left #(Narray , Nword, N) ADL(			
				  		clk, CWL,
			  	  		adr_full_row,
					        read_out, inference,
				  		reg_lrs);

//  generate Likelihoods Array 
	/* verilator lint_off UNOPTFLAT */
	wire [2**Nword-1:0] DATA_next [2**Narray:1][2**Narray:0]; // 
	assign DATA_next[1][0] = 8'b0;
	assign DATA_next[2][0] = 8'b0;
	assign DATA_next[3][0] = 8'b0;
	assign DATA_next[4][0] = 8'b0;
  generate
	genvar i,j;  // 4*4 or 2**Narray * 2**Narray
	for (i=1; i<=2**Narray; i=i+1) begin : row
		
			//assign DATA_next[i][0] = 8'b0;       // prev 0
			assign reg_out[i-1] = DATA_next[i][2**Narray]; // output 

		for (j=1; j<=2**Narray; j=j+1) begin : col

			likelihood #(Nword) likelihood_cell_unit(
					clk,
					DATA_next[i][j-1], 
					reg_lcs[j-1], 
					reg_lrs[6+i],
					reg_lrs[Nword-1:0], 
					reg_lrs[2+i], // 6 = Nword + 2^Narray -1 
					read_out_d, prog_d, read_mem_d, inference_d,
					DATA_next[i][j]);

		end
	end
  endgenerate

// Output registers 
	//assign DATA_out = reg_out;
	register_out #( Narray, Nword) outputs ( clk, read_out_d, reg_out, DATA_out );

endmodule