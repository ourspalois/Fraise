module bot_logic_log #(parameter M = 8)  
(	
	input logic clk,
	input logic read_out, prog, read_mem, adr_0, inference_en,
	input logic [M-1:0] DATA, DATA_prev,
	output logic [M-1:0] DATA_next
);
	reg [M-1:0] reg_mem;
	logic [M-1:0] DATA_auth;
	//logic [M-1:0] DATA_add, DATA_add_out,

	always_ff @(posedge clk) begin
	if (prog || (read_mem && ~adr_0)) begin
		if (~(read_out)) reg_mem <= 8'b0; // we can avoid these MUXs (already used DIN and read for reset mem output)
	end
	else if ((read_mem && adr_0) || inference_en) reg_mem <= DATA; 
	end
	assign DATA_auth = read_out ? reg_mem[M-1:0] : 8'b0;
	additions #(M) add(.proba1(DATA_auth), .proba2(DATA_prev), .proba_out(DATA_next)); // can modify the inputs based on control signals

endmodule