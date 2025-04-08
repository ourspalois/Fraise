`timescale 1ns/1ps

module tb_chip_control #(
    parameter CLK_PERIOD = 10,
    parameter string TEST_VECTOR = "test_vector.txt"    
) ();

    AXI_LITE #(
        .AXI_DATA_WIDTH(32),
        .AXI_ADDR_WIDTH(32)
    ) axi_port ();
    ADAM_SEQ seq () ; 

    chip_control #() dut(
        .seq_port(seq),
        .axi_port(axi_port)
    );

    logic clk, rst_n;
    assign seq.clk = clk;
    assign seq.rst = ~rst_n;
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end    

    task axi_read(input [31:0] address, output [31:0] data);
        axi_port.ar_valid = 1;
        axi_port.r_ready = 1;
        axi_port.ar_addr = address;
        while (!axi_port.ar_ready) begin
            #CLK_PERIOD;
        end
        axi_port.ar_valid = 0;
        while (!axi_port.r_valid) begin
            #CLK_PERIOD;
        end
        data = axi_port.r_data;
        axi_port.r_ready = 0;
        axi_port.ar_addr = 0;
        #CLK_PERIOD ;        
    endtask

    task axi_write(input [31:0] address, input [31:0] data);
        axi_port.b_ready = 1;
        axi_port.aw_addr = address;
        axi_port.w_data = data;
        axi_port.aw_valid = 1;
        axi_port.w_valid = 1;
        while (!(axi_port.aw_ready && axi_port.w_ready)) begin
            #CLK_PERIOD;
        end
        axi_port.aw_valid = 0;
        axi_port.w_valid = 0;
        while (!axi_port.b_valid) begin
            #CLK_PERIOD;
        end
        data = axi_port.b_resp;
        axi_port.b_ready = 0;
        #CLK_PERIOD ;
    endtask

    logic [31:0] data_in;
    logic [31:0] expected_data;
    logic [31:0] data_out;
    logic [31:0] address;
    integer file;
    string line;

    initial begin
        file = $fopen("test_vectors.txt", "r");
        if (file == 0) begin
            $display("Error: Could not open file");
            $finish;
        end
        
        rst_n = 0;
        #30 ;
        rst_n = 1;
        #30 ; 
        #(CLK_PERIOD/2);

        while (!$feof(file)) begin
            $fgets(line, file);

            if($sscanf(line, "write | %h | %h", address, data_in) == 2) begin
                $display("Writing to address %h, with data %h", address, data_in);
                axi_write(address, data_in);
            end
            else if($sscanf(line, "read | %h | %h", address, expected_data) == 2) begin
                axi_read(address, data_out);
                $display("Reading from address %h, data : %h", address, data_out);
                assert (data_out == expected_data ) else $display("Error: Data mismatch at address %h. Expected: %h, Got: %h", address, expected_data, data_out);
            end else if (line == "reset\n") begin
                $display("Resetting the system");
                rst_n = 0;
                #30 ;
                rst_n = 1;
            end else begin
                $display("Error: Invalid line in test vector file: %s", line);
            end
            
        end
        $finish;
    end


endmodule