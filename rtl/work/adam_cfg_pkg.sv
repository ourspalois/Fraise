
/*
 * ============================================================================
 * ADAM Configuration Package
 * ============================================================================
 *
 * This SystemVerilog package was auto-generated.
 *
 * Date   : 2025-04-30 13:45:28 UTC
 * Target : adam_nexys_video
 * Branch : small_bayes
 * Commit : b92ae9aef00dd24d070eeea308174a92f902f3aa (dirty)
 *
 * It is not recommended to modify this this file. 
 * ============================================================================
 */

`ifndef SYNTHESIS 
    `timescale 1ns/1ps
`endif

package adam_cfg_pkg;
    
    typedef logic [31:0] ADDR_T;

    typedef struct {
        ADDR_T start;
        ADDR_T end_;
        ADDR_T inc;
    } MMAP_T;

    typedef struct {
        int ADDR_WIDTH;
        int DATA_WIDTH;
        
        int GPIO_WIDTH;

        ADDR_T RST_BOOT_ADDR;

        int NO_CPUS;
        int NO_DMAS;
        int NO_MEMS;

        bit EN_LPCPU;
        bit EN_LPMEM;
        bit EN_DEBUG;
        bit EN_ACCEL;

        int NO_LSPA_GPIOS;
        int NO_LSPA_SPIS;
        int NO_LSPA_TIMERS;
        int NO_LSPA_UARTS;
        int NO_LSPA_APB;

        int NO_LSPB_GPIOS;
        int NO_LSPB_SPIS;
        int NO_LSPB_TIMERS;
        int NO_LSPB_UARTS;

        int EN_BOOTSTRAP_CPU0;
        int EN_BOOTSTRAP_MEM0;
        int EN_BOOTSTRAP_LPCPU;
        int EN_BOOTSTRAP_LPMEM;

        logic [31:0] DEBUG_IDCODE;
        ADDR_T       DEBUG_ADDR_HALT;
        ADDR_T       DEBUG_ADDR_EXCEPTION;
        
        int FAB_MAX_TRANS;

        MMAP_T MMAP_LPMEM;
        MMAP_T MMAP_SYSCFG;
        MMAP_T MMAP_LSPA;
        MMAP_T MMAP_LSPB;
        MMAP_T MMAP_ACCEL;
        MMAP_T MMAP_ACCEL_PWR;          

        ADDR_T MMAP_BOUNDRY;

        MMAP_T MMAP_DEBUG;
        MMAP_T MMAP_HSP;
        MMAP_T MMAP_MEM;
    } CFG_T;

    localparam CFG_T CFG = '{
        ADDR_WIDTH : 32,
        DATA_WIDTH : 32,

        GPIO_WIDTH : 8,
        
        RST_BOOT_ADDR : 32'h01000000,

        NO_CPUS : 1,
        NO_DMAS : 1,
        NO_MEMS : 2,
        
        EN_LPCPU : 1,
        EN_LPMEM : 1,
        EN_DEBUG : 0,
        EN_ACCEL : 1,
        
        NO_LSPA_GPIOS  : 1,
        NO_LSPA_SPIS   : 1,
        NO_LSPA_TIMERS : 1,
        NO_LSPA_UARTS  : 1,
        NO_LSPA_APB    : 1,

        NO_LSPB_GPIOS  : 0,
        NO_LSPB_SPIS   : 0,
        NO_LSPB_TIMERS : 0,
        NO_LSPB_UARTS  : 0,

        EN_BOOTSTRAP_CPU0  : 1,
        EN_BOOTSTRAP_MEM0  : 1,
        EN_BOOTSTRAP_LPCPU : 0,
        EN_BOOTSTRAP_LPMEM : 0,

        DEBUG_IDCODE         : 32'h249511c3,
        DEBUG_ADDR_HALT      : 32'h00080800,
        DEBUG_ADDR_EXCEPTION : 32'h00080808, 
        
        FAB_MAX_TRANS : 7,

        MMAP_LPMEM  : '{32'h00000000, 32'h00008000, '0},
        MMAP_SYSCFG : '{32'h00008000, 32'h00008400, '0},
        MMAP_LSPA   : '{32'h00010000, 32'h00018000, 32'h00000400},
        MMAP_LSPB   : '{32'h00018000, 32'h00020000, 32'h00000400},
        MMAP_ACCEL  : '{32'h00020000, 32'h00024000, '0},
        MMAP_ACCEL_PWR : '{32'h00024000, 32'h00024800, '0},

        MMAP_BOUNDRY : 32'h00080000,

        MMAP_DEBUG : '{32'h00080000, 32'h00088000, '0},
        MMAP_HSP   : '{32'h00090000, 32'h00098000, 32'h00000400},
        MMAP_MEM   : '{32'h01000000, 32'hffffffff, 32'h01000000}     
    };

`ifndef SYNTHESIS    

    typedef struct {
        time CLK_PERIOD;
        int  RST_CYCLES;

        time TA;
        time TT;
    } BHV_CFG_T;

    localparam BHV_CFG_T BHV_CFG = '{
        CLK_PERIOD : 40ns,
        RST_CYCLES : 5,

        TA : 2ns,
        TT : 18ns // CLK_PERIOD - TA       
    };

`endif

endpackage