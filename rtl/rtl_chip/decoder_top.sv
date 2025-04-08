module decoder_top #(parameter Nword = 3)
(	input logic [Nword-1:0] adr,
	input logic CBLEN_in,
	input logic CBL_in,
	output logic [2**Nword-1:0] CBLEN_out,
	output logic [2**Nword-1:0] CBL_out
);

	assign CBLEN_out = CBLEN_in ? 8'b1 << adr : 8'b0; //
	assign CBL_out = CBL_in ? 8'b1 << adr : 8'b0;   //

endmodule