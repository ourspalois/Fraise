interface chip_ports;
    logic clk, CBL0, CSL0, CBLEN0, CWL0 ; 
    logic [1:0] instructions_in ; 
    logic [4:0] addr_full_col_in, addr_full_row, seeds_in ; 
    logic DATA_out [3:0] ; 

    modport Master (
        output clk, CBL0, CSL0, CBLEN0, CWL0, instructions_in,
        output addr_full_col_in, addr_full_row, seeds_in, 
        input DATA_out
    );

    modport Slave (
        input clk, CBL0, CSL0, CBLEN0, CWL0, instructions_in,
        input addr_full_col_in, addr_full_row, seeds_in, 
        output DATA_out
    );
endinterface //interfacename