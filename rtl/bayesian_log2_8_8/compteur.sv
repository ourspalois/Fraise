module compteur #(parameter Narray = 2, Nword = 3)
(	input logic clk, read_out,
	output [Nword:0] cpt);
	reg [Nword:0] cpt_reg;
	assign cpt = cpt_reg;
	always @(posedge clk ) 
	begin 
		if (read_out) cpt_reg <= cpt_reg+1;
		else cpt_reg <= 4'b1110;
	end
endmodule