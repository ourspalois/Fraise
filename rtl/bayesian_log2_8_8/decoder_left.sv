module decoder_left #(parameter Nword = 2)
(	input logic [Nword-1:0] adr,
	input logic adr_0,
	input logic CWLE_in,
	output logic [2**Nword-1:0] CWLE_out
);

	assign CWLE_out = adr_0 ? (CWLE_in ? 4'b1 << adr : 4'b0) : 4'b0; //  

endmodule