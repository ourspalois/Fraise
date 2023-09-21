module comparator import comparator_pkg::* ; #(
    DataWidth = 32
) (
    input comparator_intr_e instruction,
    input logic [DataWidth-1:0] op_a,
    input logic [DataWidth-1:0] op_b,
    input logic [DataWidth-1:0] op_precision,

    output logic result
) ; 

    always_comb begin : blockName
        /* verilator lint_off CASEINCOMPLETE */
        case ( instruction)
            inf: begin
                result = (op_a < op_b);
            end 
            sup: begin
                result = (op_a > op_b);
            end
            inf_or_eq: begin
                result = (op_a <= op_b);
            end
            sup_or_eq: begin
                result = (op_a >= op_b);
            end
            eq: begin
                if (op_precision == 0) begin
                    result = (op_a == op_b);
                end else begin
                    result = (op_a >= (op_b - op_precision)) && (op_a <= (op_b + op_precision));
                end
            end
            neq: begin
                if (op_precision == 0) begin
                    result = (op_a != op_b);
                end else begin
                    result = (op_a < (op_b - op_precision)) || (op_a > (op_b + op_precision));
                end
            end
        endcase
    end

endmodule