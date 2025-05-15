`include "macros.svh"

module Fraise_wrap #(
    `ADAM_CFG_PARAMS
) (
    ADAM_SEQ.Slave seq_port,
    AXI_LITE.Slave axi_slave[2], 
    AXI_LITE.Master axi_master  
    `ifdef SYNTHESIS
    ,chip_ports.Master chip_port
    `endif 
);
    `ADAM_AXIL_SLV_TIE_OFF(axi_slave[0]);
    `ADAM_AXIL_MST_TIE_OFF(axi_master);

    chip_control #() controls (
        .seq_port(seq_port),
        .axi_port(axi_slave[1])
        `ifdef SYNTHESIS 
        ,chip_port(chip_port)
        `endif 
    ) ; 

endmodule