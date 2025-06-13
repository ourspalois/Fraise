module axi_write (
    ADAM_SEQ.Slave seq_port,
    AXI_LITE.Master axi_master, 

    // Maestro side (priority side)
    input logic [31:0] maestro_adress_i,
    input logic [31:0] maestro_data_i,
    input logic maestro_req_i,
    output logic maestro_ack_o,

    // FSM side (low priority)
    input logic [31:0] fsm_adress_i,
    input logic [31:0] fsm_data_i,
    input logic fsm_req_i,
    output logic fsm_ack_o,

    input logic [31:0] adress_i, 
    input logic req_i, 
    output logic ready_o,
    output logic [31:0] data_o,
    output logic valid_o
 
);
    // structures : 
    // 1. input buffer and arbitter 
    // write and response management 

    logic req, ack, valid ;
    logic arbiter ;

    assign maestro_ack_o = ack && arbiter; 
    assign fsm_ack_o = ack && !(arbiter) ;

    assign req = maestro_req_i || fsm_req_i ;
    assign axi_master.aw_prot = 'b0;
    assign axi_master.ar_prot = 'b0;

    logic w_responded, aw_reponded ;


    always_ff @( posedge seq_port.clk) begin 
        if(seq_port.rst) begin
            valid <= 1'b0;
            axi_master.aw_valid <= 1'b0;
            axi_master.w_valid <= 1'b0;
            axi_master.w_strb <= 4'b1111;
            axi_master.aw_addr <= 32'h0;
            axi_master.b_ready <= 1'b0;
            axi_master.w_data <= 32'h0;
            aw_reponded <= 1'b0;
            w_responded <= 1'b0;
            ack <= 1'b0;
            arbiter <= 1'b0;
        end else begin
            if(req && (ack == 0) && (axi_master.aw_valid == 0 && axi_master.w_valid == 0) && !(w_responded && aw_reponded) && (axi_master.aw_ready && axi_master.w_ready)) begin
                if(maestro_req_i) begin
                    arbiter <= 1'b1;
                    axi_master.aw_addr <= maestro_adress_i;
                    axi_master.w_data <= maestro_data_i;
                end else if(fsm_req_i) begin
                    arbiter <= 1'b0;
                    axi_master.aw_addr <= fsm_adress_i;
                    axi_master.w_data <= fsm_data_i;
                end
                axi_master.aw_valid <= 1'b1;
                axi_master.w_valid <= 1'b1;
                axi_master.w_strb <= 4'b1111;
                axi_master.b_ready <= 1'b1;
            end

            if(axi_master.aw_ready && axi_master.aw_valid) begin
                axi_master.aw_valid <= 1'b0;
                aw_reponded <= 1'b1;            
            end

            if(axi_master.w_ready && axi_master.w_valid) begin
                axi_master.w_valid <= 1'b0;
                w_responded <= 1'b1;
            end

            if(w_responded && aw_reponded) begin
                ack <= 1'b1;
                w_responded <= 1'b0;
                aw_reponded <= 1'b0;
            end else begin
                ack <= 1'b0;
            end

            if(axi_master.b_valid && axi_master.b_resp == 2'b0) begin
                axi_master.b_ready <= 1'b0;
            end
        end
    end 

    assign ready_o = axi_master.ar_ready;

    always_ff @(posedge seq_port.clk) begin
        if(seq_port.rst) begin
            valid_o <= 1'b0;
            data_o <= 32'h0;
            axi_master.ar_addr <= 'b0;
            axi_master.ar_valid <= 1'b0;
            axi_master.r_ready <= 1'b0;
        end else begin
            if(req_i && axi_master.ar_ready) begin
                axi_master.ar_addr <= adress_i;
                axi_master.ar_valid <= 1'b1;
                axi_master.r_ready <= 1'b1;
            end else  begin
                axi_master.ar_valid <= 1'b0;
            end

            if(axi_master.r_valid && axi_master.r_resp == 0) begin
                data_o <= axi_master.r_data;
                valid_o <= 1'b1;
                axi_master.r_ready <= 1'b0;
            end else begin
                valid_o <= 1'b0;
            end
        end
    end

endmodule