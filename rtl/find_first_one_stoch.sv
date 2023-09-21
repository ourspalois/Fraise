module find_first_one_stoch
(
    input logic clk,
    input logic [255:0] array1,
    input logic [255:0] array2,
    input logic [255:0] array3,
    input logic [255:0] array4,
    output logic [31:0] pos_first_one,
    output logic [31:0] sum_stoch
);

    always_ff @(posedge clk) begin
        for (integer i = 255; i >= 0 ; i--) begin
            if (array1[i] == 1) begin
                pos_first_one[7:0] <= i[7:0];
                sum_stoch[7:0] <= sum_stoch[7:0] + 1;
            end
            if (array2[i] == 1) begin
                pos_first_one[15:8] <= i[7:0];
                sum_stoch[15:8] <= sum_stoch[15:8] + 1;
            end
            if (array3[i] == 1) begin
                pos_first_one[23:16] <= i[7:0];
                sum_stoch[23:16] <= sum_stoch[23:16] + 1;
            end
            if (array4[i] == 1) begin
                pos_first_one[31:24] <= i[7:0];
                sum_stoch[31:24] <= sum_stoch[31:24] + 1;
            end
        end

    end

endmodule