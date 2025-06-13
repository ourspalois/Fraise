module axi_read (
    ADAM_SEQ.Slave seq_port,
    AXI_LITE.Master axi_master, 

    input logic [31:0] adress_i, 
    input logic req_i, 
    output logic ready_o,
    output logic [31:0] data_o,
    output logic valid_o
);
    
    assign ready_o = axi_master.ar_ready;

    always_ff @(posedge seq_port.clk) begin
        if(seq_port.rst) begin
            valid_o <= 1'b0;
            data_o <= 32'h0;
        end else begin
            if(req_i && axi_master.ar_ready) begin
                axi_master.ar_addr <= adress_i;
                axi_master.ar_valid <= 1'b1;
                axi_master.r_ready <= 1'b1;
            end else  begin
                axi_master.ar_valid <= 1'b0;
            end

            if(axi_master.r_valid) begin
                data_o <= axi_master.r_data;
                valid_o <= 1'b1;
                axi_master.r_ready <= 1'b0;
            end else begin
                valid_o <= 1'b0;
            end
        end
    end

endmodule