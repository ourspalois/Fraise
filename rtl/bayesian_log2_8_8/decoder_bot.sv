module decoder_bot #(parameter Nword = 3) // 
(	input logic [Nword-1:0] adr,
	input logic read,
	input logic CSL_in,
	output logic [2**Nword-1:0] CSL_out
);

	assign CSL_out = read ? (CSL_in ? 8'b11111111 : 8'b0) : (CSL_in ? 8'b1 << adr : 8'b0); //

endmodule