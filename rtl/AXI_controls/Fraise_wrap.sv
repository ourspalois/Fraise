`include "macros.svh"

module Fraise_wrap #(
    `ADAM_CFG_PARAMS
) (
    ADAM_SEQ.Slave soc_seq_port,
    ADAM_SEQ.Slave accel_seq_port,
    AXI_LITE.Slave axi_slave[2], 
    AXI_LITE.Master axi_master  
    `ifdef VIVADO
    ,chip_ports.Master chip_port
    `endif 
);

    `ADAM_AXIL_I control_axi ();

    axi_lite_cdc_intf #(
        .AXI_ADDR_WIDTH    (ADDR_WIDTH), 
        .AXI_DATA_WIDTH    (DATA_WIDTH), 
        .LOG_DEPTH         (1          )
    ) control_cdc (
        .src_clk_i (soc_seq_port.clk     ),
        .src_rst_ni(~soc_seq_port.rst     ),
        .src       (axi_slave[1] ), 
        .dst_clk_i (accel_seq_port.clk   ),
        .dst_rst_ni(~accel_seq_port.rst  ),
        .dst       (control_axi)
    ) ; 

    chip_control #() controls (
        .seq_port(accel_seq_port),
        .axi_port(control_axi)
        `ifdef VIVADO 
        ,.chip_port(chip_port)
        `endif 
    ) ; 
    //`ADAM_AXIL_MST_TIE_OFF(axi_master);
    //`ADAM_AXIL_SLV_TIE_OFF(axi_slave[0]);
    
    PWR_CTRL #() power_ctrl (
        .seq_port(soc_seq_port),
        .axi_port(axi_slave[0]),
        .axi_master(axi_master)
    );

endmodule