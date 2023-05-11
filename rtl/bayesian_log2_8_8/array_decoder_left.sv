module array_decoder_left #(parameter Narray = 2, Nword = 3, N = Nword + Narray)
(
        input logic clk, CWL,
        input logic [N-1:0] adr_full_row,
        input logic read_out, inference, 
        //output logic [2**Narray-1:0] CWL_in,
        output logic [Nword + 2**Narray*2-1:0] reg_lrs
);
// rows registers 
	reg [Nword + 2**Narray*2-1:0] reg_lr;
// Decoder
	logic [3:0] selected_left;
	logic [3:0] CWL_in;
	assign selected_left = read_out ? 4'b0 : (inference ? 4'b1111 : 4'b1 << adr_full_row[N-1:N-2]); // 
	assign CWL_in = CWL ? selected_left : 4'b0; // better MUX or logic AND ? 


// Store rows registers 
	assign reg_lrs = reg_lr;
	always_ff @(posedge clk) begin
		if (read_out) reg_lr <= 7'b0;
		else reg_lr <= { selected_left, CWL_in, adr_full_row[Nword-1:0]}; // 
	end

endmodule