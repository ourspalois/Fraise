`ifndef ADAM_MACROS_SVH_
`define ADAM_MACROS_SVH_

`define ADAM_COMMA ,
`define ADAM_SEMICOLON ;

// ADAM_CFG ===================================================================

`define ADAM_CFG_PARAMS_GENERIC(__opt, __sep) \
    __opt type CFG_T  = adam_cfg_pkg::CFG_T __sep \
    __opt CFG_T CFG   = adam_cfg_pkg::CFG __sep \
    \
    __opt ADDR_WIDTH   = CFG.ADDR_WIDTH __sep \
    __opt DATA_WIDTH   = CFG.DATA_WIDTH __sep \
    __opt STRB_WIDTH   = DATA_WIDTH/8 __sep \
    \
    __opt type ADDR_T = logic [ADDR_WIDTH-1:0] __sep \
    __opt type PROT_T = logic [2:0] __sep \
    __opt type DATA_T = logic [DATA_WIDTH-1:0] __sep \
    __opt type STRB_T = logic [STRB_WIDTH-1:0] __sep \
    __opt type RESP_T = logic [1:0] __sep \
    \
    __opt GPIO_WIDTH = CFG.GPIO_WIDTH __sep \
    \
    __opt type GPIO_T = logic [GPIO_WIDTH-1:0] __sep \
    \
    __opt type MMAP_T = adam_cfg_pkg::MMAP_T __sep \
    \
    __opt ADDR_T RST_BOOT_ADDR = CFG.RST_BOOT_ADDR __sep \
    \
    __opt NO_CPUS = CFG.NO_CPUS __sep \
    __opt NO_DMAS = CFG.NO_DMAS __sep \
    __opt NO_MEMS = CFG.NO_MEMS __sep \
    \
    __opt EN_LPCPU = CFG.EN_LPCPU __sep \
    __opt EN_LPMEM = CFG.EN_LPMEM __sep \
    __opt EN_DEBUG = CFG.EN_DEBUG __sep \
    __opt EN_ACCEL = CFG.EN_ACCEL __sep \
    \
    __opt NO_LSPA_GPIOS  = CFG.NO_LSPA_GPIOS __sep \
    __opt NO_LSPA_SPIS   = CFG.NO_LSPA_SPIS __sep \
    __opt NO_LSPA_TIMERS = CFG.NO_LSPA_TIMERS __sep \
    __opt NO_LSPA_UARTS  = CFG.NO_LSPA_UARTS __sep \
    \
    __opt NO_LSPB_GPIOS  = CFG.NO_LSPB_GPIOS __sep \
    __opt NO_LSPB_SPIS   = CFG.NO_LSPB_SPIS __sep \
    __opt NO_LSPB_TIMERS = CFG.NO_LSPB_TIMERS __sep \
    __opt NO_LSPB_UARTS  = CFG.NO_LSPB_UARTS __sep \
    \
    __opt EN_BOOTSTRAP_CPU0  = CFG.EN_BOOTSTRAP_CPU0 __sep \
    __opt EN_BOOTSTRAP_MEM0  = CFG.EN_BOOTSTRAP_MEM0 __sep \
    __opt EN_BOOTSTRAP_LPCPU = CFG.EN_BOOTSTRAP_LPCPU __sep \
    __opt EN_BOOTSTRAP_LPMEM = CFG.EN_BOOTSTRAP_LPMEM __sep \
    \
    __opt logic [31:0] DEBUG_IDCODE         = CFG.DEBUG_IDCODE __sep \
    __opt ADDR_T       DEBUG_ADDR_HALT      = CFG.DEBUG_ADDR_HALT __sep \
    __opt ADDR_T       DEBUG_ADDR_EXCEPTION = CFG.DEBUG_ADDR_EXCEPTION __sep \
    \
    __opt FAB_MAX_TRANS = CFG.FAB_MAX_TRANS __sep \
    \
    __opt MMAP_T MMAP_LPMEM  = CFG.MMAP_LPMEM __sep \
    __opt MMAP_T MMAP_SYSCFG = CFG.MMAP_SYSCFG __sep \
    __opt MMAP_T MMAP_LSPA   = CFG.MMAP_LSPA __sep \
    __opt MMAP_T MMAP_LSPB   = CFG.MMAP_LSPB __sep \
    __opt MMAP_T MMAP_ACCEL  = CFG.MMAP_ACCEL __sep \
    \
    __opt ADDR_T MMAP_BOUNDRY = CFG.MMAP_BOUNDRY __sep \
    \
    __opt MMAP_T MMAP_DEBUG = CFG.MMAP_DEBUG __sep \
    __opt MMAP_T MMAP_HSP   = CFG.MMAP_HSP __sep \
    __opt MMAP_T MMAP_MEM   = CFG.MMAP_MEM __sep \
    \
    __opt NO_LSPAS = NO_LSPA_GPIOS + NO_LSPA_SPIS + NO_LSPA_TIMERS + \
        NO_LSPA_UARTS __sep \
    \
    __opt EN_LSPA = (NO_LSPAS > 0) __sep \
    \
    __opt NO_LSPBS = NO_LSPB_GPIOS + NO_LSPB_SPIS + NO_LSPB_TIMERS + \
        NO_LSPB_UARTS __sep \
    \
    __opt EN_LSPB = (NO_LSPBS > 0) __sep \
    \
    __opt NO_HSPS = 0 __sep \
    __opt EN_HSP  = 0

`define ADAM_CFG_PARAMS \
    `ADAM_CFG_PARAMS_GENERIC(parameter, `ADAM_COMMA)

`define ADAM_CFG_LOCALPARAMS \
    `ADAM_CFG_PARAMS_GENERIC(localparam, `ADAM_SEMICOLON)

`define ADAM_CFG_PARAMS_MAP \
    .CFG_T (CFG_T), \
    .CFG (CFG)

`endif