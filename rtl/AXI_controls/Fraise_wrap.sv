`include "macros.svh"

module Fraise_wrap #(
    `ADAM_CFG_PARAMS
) (
    ADAM_SEQ.Slave seq_port,
    AXI_LITE.Slave axi_slave[2], 
    AXI_LITE.Master axi_master  
    `ifdef VIVADO
    ,chip_ports.Master chip_port
    `endif 
);

    chip_control #() controls (
        .seq_port(seq_port),
        .axi_port(axi_slave[1])
        `ifdef VIVADO
        ,.chip_port(chip_port)
        `endif 
    ) ; 
    `ADAM_AXIL_MST_TIE_OFF(axi_master);
    `ADAM_AXIL_SLV_TIE_OFF(axi_slave[0]);

    /*
    PWR_CTRL #() power_ctrl (
        .seq_port(seq_port),
        .axi_port(axi_slave[0]),
        .axi_master(axi_master)
    );*/

endmodule