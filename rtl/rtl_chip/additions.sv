module additions #(parameter M = 8)
	(input logic [M-1:0] proba1, proba2,
	output logic [M-1:0] proba_out);
	logic [M:0] d1;
	logic [M-1:0] overflow;
	assign overflow = 8'b11111111;  
	
	assign d1 = proba1 + proba2;
	assign proba_out = d1[M] ? overflow : d1[M-1:0];

endmodule