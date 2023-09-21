module array_decoder_top #(parameter Nword = 3 , Narray = 2, N = Nword + Narray, M = 2**Nword) 
(       input logic clk, CBL, CBLEN, CSL,
        //input logic [1:0] instructions,   
        input logic [N-1:0] adr_full_col,
        input logic  read_out,
        //output logic [2**Narray-1:0] CBL_in,
        //output logic [2**Narray-1:0] CBLEN_in,
        //output logic [2**Narray-1:0] CSL_in,
        output logic [Nword+3:0] reg_lcs [2**Narray-1:0]);

// colomns registers 
	reg [Nword+3:0] reg_lc [2**Narray-1:0]; // likelihood colomns reg, size = Nword + 3 bits of CBL, CBLEN, CSL
  
// memory control signals 
        logic [2**Narray-1:0] CSL_in, CBLEN_in, CBL_in;

// Decoder Base
	logic [3:0] selected_top;
	assign selected_top = read_out ? 4'b0 : 4'b1 << adr_full_col[N-1:N-2];  // if read out disable selection else inference, prog_form or read_mem
	assign CBLEN_in = CBLEN ? selected_top: 4'b0; // better MUX or logic AND ? 
	assign CBL_in = CBL ? selected_top : 4'b0;
	assign CSL_in = CSL ? selected_top : 4'b0;


// Store colomns registers 
	assign reg_lcs = reg_lc;
	always_ff @(posedge clk) begin
		case (selected_top) 
			4'b0001 : begin
				  reg_lc[0] <= {selected_top[0], CBLEN_in[0] , CBL_in[0], CSL_in[0], adr_full_col[Nword-1:0]}; //
				  reg_lc[1][Nword+3] <= {selected_top[1]};
				  reg_lc[2][Nword+3] <= {selected_top[2]};
				  reg_lc[3][Nword+3] <= {selected_top[3]};
				end
 
			4'b0010 : begin
				  reg_lc[1] <= {selected_top[1], CBLEN_in[1] , CBL_in[1], CSL_in[1], adr_full_col[Nword-1:0]}; // 
				  reg_lc[2][Nword+3] <= {selected_top[2]};
				  reg_lc[3][Nword+3] <= {selected_top[3]};
				  reg_lc[0][Nword+3] <= {selected_top[0]};
				end

			4'b0100 : begin 
				  reg_lc[2] <= {selected_top[2], CBLEN_in[2] , CBL_in[2], CSL_in[2], adr_full_col[Nword-1:0]}; // 
				  reg_lc[1][Nword+3] <= {selected_top[1]};
				  reg_lc[3][Nword+3] <= {selected_top[3]};
				  reg_lc[0][Nword+3] <= {selected_top[0]};
				end

			4'b1000 : begin
				  reg_lc[3] <= {selected_top[3], CBLEN_in[3] , CBL_in[3], CSL_in[3], adr_full_col[Nword-1:0]}; //
				  reg_lc[1][Nword+3] <= {selected_top[1]};
				  reg_lc[2][Nword+3] <= {selected_top[2]};
				  reg_lc[0][Nword+3] <= {selected_top[0]};
				end
			4'b0000 : reg_lc <= '{default:0};
			default : reg_lc <= '{default:0}; 
		endcase

	end

endmodule