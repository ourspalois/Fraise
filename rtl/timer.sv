module timer_obs # (
    int SIZE = 32, 
    int STOP = 2**22
)(
    input logic clk, 
    input logic rst, 
    input logic en, 
    output logic pulse
) ;
    logic [31:0] cnt ; 
    always_ff @( posedge(clk) ) begin : timer
        if (!rst) begin
            cnt <= '0;
            pulse <= '0;
        end else if (en) begin
            if (cnt >= STOP) begin
                cnt <= '0;
                pulse <= '1;
            end else begin
                cnt <= cnt + '1;
                pulse <= '0;
            end
        end
    end

endmodule