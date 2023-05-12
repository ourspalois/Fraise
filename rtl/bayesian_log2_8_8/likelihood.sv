module likelihood #(parameter Nword = 3)
// WARNING : REVERSE THE DATA WHEN PROGRAMMING MEMORY AND COMPLEMENT IT!!
(	input logic clk,
	input logic [2**Nword-1:0] DATA_prev,
	input logic [Nword+3:0] reg_lcs, 
	input logic selected_left,
	input logic [Nword-1:0] reg_lrs,
	input logic CWL_in,
	input logic read_out, prog, read_mem, inference,
	output logic [2**Nword-1:0] DATA_next   // M = 2**Nword 
);

	logic [2**Nword-1:0] CBL, CBLEN, CSL, CWL; // outputs of top and bot decoders
	logic [2**Nword-1:0] DATA, DIN, DINb;  // inputs of memory array
	logic [2**(Nword-1)-1:0] CWLE, CWLO; // out of left and righ decoders
	logic CWL_in0, CBL_in0, CBLEN_in0, CSL_in0; // decoders control signals inputs 
	logic read, inference_en;       // read signal, for read_ mem and inference states
	logic adr_0;
	assign CWL_in0 = CWL_in & adr_0;
	assign adr_0 = selected_left & reg_lcs[Nword+3];
	assign CBL_in0 = reg_lcs[Nword+1] & adr_0;
	assign CBLEN_in0 = reg_lcs[Nword+2] & adr_0;
	assign CSL_in0 = reg_lcs[Nword] & adr_0;
	assign read = inference | (read_mem & adr_0);  // & adr_0;
	assign inference_en = inference & reg_lcs[Nword+3];
// adr coming from ADL is connected to DT and DB,
// adr coming from ADT is connected to DR and DL,
// control signals kept the same connections    
	decoder_top #(Nword) DT( .adr(reg_lrs[Nword-1:0]), .CBLEN_in(CBLEN_in0), .CBL_in(CBL_in0), .CBLEN_out(CBLEN), .CBL_out(CBL));
	decoder_bot #(Nword) DB( .adr(reg_lrs[Nword-1:0]), .read(read), .CSL_in(CSL_in0), .CSL_out(CSL));
	decoder_right #(Nword-1) DR( .adr(reg_lcs[Nword -1:1]), .adr_0(reg_lcs[0]), .CWLO_in(CWL_in0), .CWLO_out(CWLO));
	decoder_left #(Nword-1) DL( .adr(reg_lcs[Nword -1:1]), .adr_0(reg_lcs[0]), .CWLE_in(CWL_in0), .CWLE_out(CWLE));

	//assign CWL = {CWLE , CWLO};
	assign DIN = read ? 8'b11111111 : 8'b0; //
	assign DINb = 8'b0;

	//NEURONIC_4k_FULL_MOD rram(.CBL(CBL), .CBLEN(CBLEN), .CSL(CSL), .DIN(DIN), .DINb(DINb), .CWLE(CWLE), .CWLO(CWLO), .DATA(DATA));
	MC_8x8_FULL_upd rram(.CBL(CBL), .CBLEN(CBLEN), .CSL(CSL), .DIN(DIN), .DINb(DINb), .CWLE(CWLE), .CWLO(CWLO), .DOUT(DATA));
	//RRAM_array #(2**Nword,2**(Nword-1)) rram(.CBL(CBL), .CBLEN(CBLEN), .CSL(CSL), .DIN(DIN), .DINb(DINb), .CWLE(CWLE), .CWLO(CWLO), .DATA(DATA));

	bot_logic_log #(2**Nword) BT_log(clk, read_out, prog, read_mem, adr_0, inference_en, DATA, DATA_prev, DATA_next);

endmodule