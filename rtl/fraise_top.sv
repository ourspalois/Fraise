// This is the top module of the accelerator to control the bayesian array developped by INTEGNANO

// author : Ourspalois, INTEGNANO, 2023

module fraise_top  #(
    parameter int unsigned DataWidth = 32, 
    parameter int unsigned AddrWidth = 32,
    parameter int unsigned MatrixSize = 4, // I consider them square
    parameter int unsigned ArraySize = 8, 
    parameter int unsigned Nword_used = 3, //on lit 8 bits a la fois 
    parameter int unsigned NbrHostsLog2 = 1 
    ) (
    // basics 
    input logic clk_i,
    input logic reset_n,
    // interface with the on chip network 
    // request network interface
    input logic req_valid_i, // to tell if the request is valid 
    output logic ready_o, // to tell if the accelerator is ready to receive a new request 
    input logic [NbrHostsLog2-1:0] req_host_addr_i, // the adress of the host sending the request 
    input logic [AddrWidth-1:0] req_addr_i, // the adress to received 
    // address format : {1'h1, matrix_column, matrix_line, array_column, array_line}
    input logic req_wen_i, // write enable on 1 
    input logic [DataWidth-1:0] req_wdata_i, // the data to write 
    input logic [(DataWidth/8)-1:0] req_ben_i, // the bytes where the write is enabled 
    // response network interface
    output logic resp_valid_o, // to tell if the response is valid
    input logic resp_ready_i, // to tell if the network is ready for a response
    output logic [DataWidth-1:0] resp_data_o, // the data read
    output logic [NbrHostsLog2-1:0] resp_ini_addr_o ,// the adress of the initiator of the request
    output logic irq_o, // interrupt signal

    //master interface
    output logic Host_req_valid_o,
    input logic Host_req_ready_i, 
    input logic Host_gnt_i, 
    output logic [DataWidth-1:0] Host_tgt_addr_o,
    output logic Host_wen_o, 
    output logic [DataWidth/8 - 1 :0] Host_ben_o,
    output logic [DataWidth-1:0] Host_wdata_o,
    input logic Host_resp_valid_i, 
    output logic Host_resp_ready_o,
    input logic [DataWidth-1:0] Host_resp_data_i
) ;
    //pkg 
    import comparator_pkg::*;

    // logs 
    localparam int unsigned MatrixSizeLog2 = (MatrixSize == 1) ? 1 : $clog2(MatrixSize) ;
    localparam int unsigned ArraySizeLog2 = (ArraySize == 1) ? 1 : $clog2(ArraySize) ;
    

    // the accelerator resgisters adresses. 
    localparam int unsigned REG_START = 32'h00010 ; 
    localparam int unsigned ON_OFF_REG = 32'h000 + REG_START ; // 0 iddle, 1 inference (continuous)
    localparam int unsigned OBS_REG_1 = 32'h008 + REG_START ;
    localparam int unsigned OBS_REG_2 = 32'h00C + REG_START ;
    localparam int unsigned LAUNCH_REG = 32'h010 + REG_START ;
    localparam int unsigned RES_VALID_REG = 32'h014 + REG_START ;
    localparam int unsigned RES_REG = 32'h018 + REG_START ;
    localparam int unsigned IRQ_EN_REG = 32'h020 + REG_START ; // 0 : no irq, 1 : irq when res is valid
    localparam int unsigned PRECISION_REG = 32'h024 + REG_START ; 
    localparam int unsigned COMP_INSTRUCTION_REG = 32'h028 + REG_START ;
    localparam int unsigned COMP_REFERENCE = 32'h02C + REG_START ;
    localparam int unsigned COMP_RESULT = 32'h030 + REG_START ;
    localparam int unsigned COMP_BYPASS = 32'h034 + REG_START ;
    localparam int unsigned INFERENCE_WRITE_REG = 32'h038 + REG_START ;
    localparam int unsigned WRITING_SET_RESET = 32'h03C + REG_START ;
    localparam int unsigned HOST_EN = 32'h040 + REG_START ;
    localparam int unsigned OBS_ADDR_1 = 32'h044 + REG_START ;
    localparam int unsigned OBS_ADDR_2 = 32'h048 + REG_START ;
    localparam int unsigned OBS_ADDR_3 = 32'h04C + REG_START ;
    localparam int unsigned OBS_ADDR_4 = 32'h050 + REG_START ;

    localparam int unsigned MEM_ARRAY_START = 32'h800 ; // word size of this is 32 bits
    localparam int unsigned MEM_ARRAY_END = 32'hFFF ;

    logic [MatrixSize-1:0] new_obs ;
    logic [DataWidth-1:0] device_read_data;

    // broadcast byte enable to the 32 bits
    logic [DataWidth-1:0] req_ben_32 ;
    assign req_ben_32 = {{8{req_ben_i[3]}}, {8{req_ben_i[2]}}, {8{req_ben_i[1]}}, {8{req_ben_i[0]}}};

    typedef enum logic[DataWidth-1:0] { 
        Off, 
        On
    } ON_OFF_reg_e;
    
    /* verilator lint_off UNOPTFLAT */
    ON_OFF_reg_e ON_OFF_reg ; 

    logic [DataWidth-1:0] obs_reg ;
    logic [MatrixSize-1:0][2:0]Observation_vec ;
    assign Observation_vec = {obs_reg[26:24], obs_reg[18:16], obs_reg[10:8], obs_reg[2:0]} ; 
    logic launch_reg ; 
    logic res_valid, irq_en ; 
    logic stay_high ;

    logic [MatrixSize-1:0][2**Nword_used-1:0] results ;

    // write in the array 
    //TODO: make this only compiled with verilator (not used on quartus, dont want to break chips)
    logic [DataWidth-1:0] data_to_write ; 
    logic write_en ;
    logic set_reset ; // 1 if set 0 if reset 
    logic [DataWidth-1:0] write_addr ; 
    logic [4:0] write_counter ; 
    logic [3:0] internal_write_counter ; 

    typedef enum int { 
        set_CSL, 
        set_CBL, 
        set_CWL, 
        reset_CWL,
        loop
    } write_state_e;
    write_state_e write_state ; 

    // irq management
    localparam int unsigned res_size = 2**Nword_used;

    logic [MatrixSize-1:0] decision_result ; 
    logic comparator_bypass ; 

    logic [MatrixSize-1:0] comp_res ; 
    logic [MatrixSize-1:0][res_size-1:0] comp_precision ; // only usefull for == test 
    logic [MatrixSize-1:0][res_size-1:0] comp_reference ; 
    logic [MatrixSize-1:0][res_size-1:0] comp_instr; 

    genvar i ;
    generate
        for (i = 0; i< MatrixSize ; i = i+1) begin
            comparator #(.DataWidth(2**Nword_used)) u_comp (
                .instruction(comparator_intr_e'(comp_instr[i])) ,
                .op_a(results[i]),
                .op_b(comp_reference[i]),
                .op_precision(comp_precision[i]),
                .result(comp_res[i])
            ) ; 
        end
    endgenerate

    logic [MatrixSize-1:0] min_max_res ; 

    always_comb begin
        if(res_valid) begin
            /* verilator lint_off CASEINCOMPLETE */
            case(comp_instr[0])
            min: begin
                min_max_res[0] = (results[0] < results[1] & results[0] < results[2] & results[0] < results[3]) ? 1'b1 : 1'b0 ;
                min_max_res[1] = (results[1] < results[0] & results[1] < results[2] & results[1] < results[3]) ? 1'b1 : 1'b0 ;
                min_max_res[2] = (results[2] < results[0] & results[2] < results[1] & results[2] < results[3]) ? 1'b1 : 1'b0 ;
                min_max_res[3] = (results[3] < results[0] & results[3] < results[1] & results[3] < results[2]) ? 1'b1 : 1'b0 ; 
            end
            max: begin
                min_max_res[0] = (results[0] > results[1] & results[0] > results[2] & results[0] > results[3]) ? 1'b1 : 1'b0 ;
                min_max_res[1] = (results[1] > results[0] & results[1] > results[2] & results[1] > results[3]) ? 1'b1 : 1'b0 ;
                min_max_res[2] = (results[2] > results[0] & results[2] > results[1] & results[2] > results[3]) ? 1'b1 : 1'b0 ;
                min_max_res[3] = (results[3] > results[0] & results[3] > results[1] & results[3] > results[2]) ? 1'b1 : 1'b0 ;
            end
            endcase
        end 
    end

    always_ff @(posedge(clk_i)) begin
        if(res_valid) begin
            if(comp_instr[0] == min | comp_instr[0] == max) begin
                decision_result <= min_max_res ;
            end else begin
                decision_result <= comp_res ;
            end
            if(irq_en) begin
                if (comparator_bypass) begin
                    irq_o <= 1'b1 ;
                end else begin
                    irq_o <= (decision_result != 0) ? 1'b1 : 1'b0 ;
                end
            end
        end else begin
            irq_o <= 1'b0 ;
        end
    end 

    typedef enum int { 
            Host_idle,
            Host_send_request,
            Host_get_response
    } Host_managment_state_e ;

    Host_managment_state_e  Host_state ; 
    logic Host_en ; 
    logic [7:0] obs_index ;

    logic [MatrixSize-1:0][AddrWidth-1:0] Obs_address ; // TODO: 

    // controle machine

    typedef enum int { 
        Inference, 
        Writting
    } inference_write_e;

    inference_write_e inference_write ;

    typedef enum int { 
        Idle,
        Read_cycles_stoch,
        Run_stoch, 
        Read_cycles_log,
        Inf_cycles,
        Read_out,
        Run_log,
        Done
     } inference_state_e;

    inference_state_e inference_state ; 

    typedef enum int { 
        SL_WL_rise,
        Sl_fall,
        WL_high,
        WLfall
    } read_state_e ;

    read_state_e read_state ;

    always_ff @( posedge(clk_i) ) begin : Inference_machine
        if(reset_n) begin
        // interface management
        if(req_valid_i & ready_o) begin
            case(req_addr_i)
                ON_OFF_REG: begin 
                    if(req_wen_i) begin
                        /* verilator lint_off ALWCOMBORDER */
                        ON_OFF_reg <= ON_OFF_reg_e'(req_wdata_i) ;
                    end
                    device_read_data = ON_OFF_reg;
                end
                OBS_REG_1: begin 
                    if(req_wen_i) begin
                        obs_reg <= req_wdata_i & req_ben_32 ;
                    end
                    device_read_data = obs_reg ;
                end
                LAUNCH_REG: begin 
                    if(req_wen_i) begin
                        launch_reg <= req_wdata_i[0] ;
                    end
                    device_read_data = {31'b0, launch_reg} ;
                end 
                RES_VALID_REG: begin 
                    device_read_data = {31'b0, res_valid} ;
                end
                RES_REG: begin 
                    device_read_data = results ;
                    res_valid <= '0 ;
                end
                IRQ_EN_REG: begin 
                    if(req_wen_i) begin
                        irq_en <= req_wdata_i[0] ;
                    end
                    device_read_data = {31'b0, irq_en} ;
                end
                PRECISION_REG: begin
                    if(req_wen_i) begin
                        comp_precision <= req_wdata_i ;
                    end 
                    device_read_data = comp_precision ;
                end
                COMP_INSTRUCTION_REG: begin
                    if(req_wen_i) begin
                        comp_instr <= req_wdata_i ;
                    end 
                    device_read_data = comp_instr ;
                end
                COMP_REFERENCE: begin
                    if(req_wen_i) begin
                        comp_reference <= req_wdata_i ;
                    end 
                    device_read_data = comp_reference ;
                end
                COMP_RESULT: begin
                    device_read_data = {28'b0, comp_res} ;
                end
                COMP_BYPASS: begin
                    if(req_wen_i) begin
                        comparator_bypass <= req_wdata_i[0] ;
                    end 
                    device_read_data = {31'b0, comparator_bypass} ;
                end
                INFERENCE_WRITE_REG: begin
                    if(req_wen_i) begin
                        inference_write <= inference_write_e'(req_wdata_i[1:0]) ;
                    end 
                    device_read_data = inference_write ;
                end
                WRITING_SET_RESET: begin
                    if(req_wen_i)begin
                        set_reset <= req_wdata_i[0] ;
                    end
                end
                HOST_EN:begin
                    if(req_wen_i)begin
                        Host_en <= req_wdata_i[0] ;
                    end
                end
                OBS_ADDR_1:begin
                    if(req_wen_i)begin
                        Obs_address[0] <= req_wdata_i ;
                    end
                end
                OBS_ADDR_2:begin
                    if(req_wen_i)begin
                        Obs_address[1] <= req_wdata_i ;
                    end
                end
                OBS_ADDR_3:begin
                    if(req_wen_i)begin
                        Obs_address[2] <= req_wdata_i ;
                    end
                end
                OBS_ADDR_4:begin
                    if(req_wen_i)begin
                        Obs_address[3] <= req_wdata_i ;
                    end
                end
                default: begin
                    if(req_addr_i >= MEM_ARRAY_START & req_addr_i <= MEM_ARRAY_END) begin
                        device_read_data = '0 ; // not able to read for now
                        data_to_write <= req_wdata_i ;
                        write_addr <= req_addr_i ;
                        write_en <= 'b1 ; 
                    end else begin
                        `ifdef VERILATOR 
                            $display("FRAISE ERROR : address out of range of the memory : %h", req_addr_i);
                        `endif
                    end
                end
            endcase
            resp_valid_o <= 1'b1; 
            resp_ini_addr_o <= req_host_addr_i; 
            resp_data_o <= device_read_data;
            
        end else begin          
            resp_valid_o <= 1'b0;
        end
        // inference
        case (inference_write)
            Inference: begin        
            case(inference_state)
                Idle: begin
                    read_state <= SL_WL_rise ;
                    counter <= '0 ;
                    counter_run_reset <= '0 ;
                    if(ON_OFF_reg == On | launch_reg == '1) begin
                        inference_state <= Read_cycles_log ;
                        results <= '0 ;
                    end 
                end
                Read_cycles_log: begin
                    ready_o <= '0 ;
                    launch_reg <= '0 ; 
                    instructions <= 2'b00 ; // read_mem_mode
                    addr_col = {counter[1:0], 3'b0} ; 
                    addr_row = {2'b0, Observation_vec[counter[1:0]]} ;
                    case (read_state)
                        SL_WL_rise : begin
                            WL_signal <= '1 ;
                            SL_signal <= '1 ;
                            read_state <= Sl_fall ;
                        end 
                        Sl_fall : begin
                            SL_signal <= '0 ;
                            read_state <= WL_high ;
                        end
                        WL_high : begin
                            WL_signal <= '1 ;
                            read_state <= WLfall ;
                        end
                        WLfall : begin
                            WL_signal <= '0 ;
                            read_state <= SL_WL_rise ;
                            if(counter == 8'(MatrixSize-1)) begin
                                inference_state <= Read_out ;
                                counter_run_en <= '1 ;
                            end 
                            counter <= counter + 1 ;
                        end
                    endcase
                end
                /*
                Inf_cycles: begin
                    instructions <= b'00 ; // inference mode
                    if(counter_run >= 1) begin
                        inference_state <= Read_out ;
                    end
                end */
                Read_out:begin
                    instructions <= 2'b01 ;
                    if(counter_run >= 3) begin
                        inference_state <= Run_log ;
                    end
                end
                Run_log: begin
                    instructions <= 2'b01 ;
                    results[0] <= results[0] | ({7'b0, bit_out[0]} << (7-(counter_run-6))) ;
                    results[1] <= results[1] | ({7'b0, bit_out[1]} << (7-(counter_run-6))) ;
                    results[2] <= results[2] | ({7'b0, bit_out[2]} << (7-(counter_run-6))) ;
                    results[3] <= results[3] | ({7'b0, bit_out[3]} << (7-(counter_run-6))) ;

                    if(counter_run >= 13) begin
                        counter_run_en <= '0 ;
                        inference_state <= Done;
                    end
                end
                Done: begin
                    counter_run_reset <= '1 ;
                    res_valid <= '1 ; 
                    ready_o <= '1 ;
                    inference_state <= Idle;
                end
            endcase
            end
            Writting:begin
                if(write_en) begin
                CBLEN <= '1 ;
                instructions <= 2'b11 ; // write mode
                addr_col = {write_addr[4:3], write_counter[4:2]} ;
                addr_row = {write_addr[2:0], write_counter[1:0]} ;
                if(write_state == set_CSL) begin
                    internal_write_counter <= '0 ;
                end else begin
                    internal_write_counter <= internal_write_counter + 1 ;
                end
                case(write_state) 
                    set_CSL: begin
                        ready_o <= '0 ;
                        SL_signal <= set_reset ; 
                        write_state <= set_CBL ;
                    end
                    set_CBL: begin
                        CBL <= !(data_to_write[write_counter]); 
                        if (internal_write_counter == 2) begin
                            write_state <= set_CWL ;
                        end
                    end
                    set_CWL:begin
                        WL_signal <= '1 ; 
                        write_state <= reset_CWL ; 
                    end
                    reset_CWL:begin
                        WL_signal <= '0 ; 
                        if (internal_write_counter == 5) begin
                            write_state <= loop ;
                        end
                    end
                    loop:begin
                        if(write_counter == 31) begin
                            write_en <= '0 ;
                            CBLEN <= '0 ;
                            write_counter <= '0 ;
                            internal_write_counter <= '0 ;
                            inference_state <= Idle ;
                            ready_o <= '1 ;
                            write_state <= set_CSL ;
                        end else begin
                            write_counter <= write_counter + 1 ;
                            internal_write_counter <= '0 ;
                            write_state <= set_CSL ;
                        end
                    end
                endcase               
                end 
            end
        endcase
        // Host side control 
        
        case (Host_state)
            Host_idle: begin
                Host_req_valid_o <= '0 ;
                Host_resp_ready_o <= '0 ;
                if(Host_en & (new_obs != '0)) begin
                    Host_state <= Host_send_request ;
                end
             end
            Host_send_request: begin
                if(Host_req_ready_i) begin
                    case ({4'b0, new_obs})
                        1: obs_index = 0 ;
                        2: obs_index = 1 ;
                        4: obs_index = 2 ;
                        8: obs_index = 3 ;
                        default: obs_index = 0 ;
                    endcase  
                    Host_req_valid_o <= '1 ;
                    Host_tgt_addr_o <= Obs_address[obs_index] ;
                    Host_wen_o <= '0 ;
                    Host_ben_o <= 4'b1111;
                    Host_wdata_o <= '0 ;
                    if(Host_gnt_i == '1) begin
                        Host_state <= Host_get_response ;
                    end
                end                   
            end
            Host_get_response: begin
                Host_req_valid_o <= '0 ;
                   Host_tgt_addr_o <= '0 ;
                Host_resp_ready_o <= '1 ;
                if(Host_resp_valid_i == '1) begin
                    case (obs_index)
                        1 : obs_reg[2:0] <= Host_resp_data_i[2:0] ;
                        2 : obs_reg[5:3] <= Host_resp_data_i[2:0] ;
                        3 : obs_reg[8:6] <= Host_resp_data_i[2:0] ;
                        4 : obs_reg[11:9] <= Host_resp_data_i[2:0] ;
                    endcase
                    Host_state <= Host_idle ;
                end
            end
            endcase 
        end else begin
            inference_state <= Idle;
            read_state <= SL_WL_rise ; 
            inference_write <= Inference ;
            counter <= '0 ;
            write_counter <= '0 ;
            counter_run_reset <= '0 ;
            counter_run_en <= '0 ;
            res_valid <= '0 ;
            ready_o <= '1 ;
            Host_state <= Host_idle ;
            Host_en <= '0 ;
            obs_index = '0 ;
            Obs_address <= '0 ;
        end
    end

    // counter 

    logic [2**Nword_used-1:0] counter ;

    //run counter

    logic [2**Nword_used-1:0] counter_run ;
    logic counter_run_reset, counter_run_en ;

    always_ff @( posedge(clk_i) ) begin : compteur_run
        if(counter_run_reset) begin
            counter_run <= '0 ; 
        end else if(counter_run_en) begin
            counter_run <= counter_run + 1 ; 
        end
    end

    // read counter 

    logic [7:0] counter_read ;
    logic counter_read_reset, counter_read_en ;

    always_ff @( posedge(clk_i) ) begin : compteur_read
        if(counter_read_reset) begin
            counter_read <= '0 ; 
        end else if(counter_read_en) begin
            counter_read <= counter_read + 1 ; 
        end
    end

    // check new obs (timer based now) 
    logic timer_pulse ;

    timer_obs #(.SIZE(DataWidth), .STOP(32'(1<<25))) utimer_obs (
        .clk(clk_i),
        .rst(reset_n),
        .en(Host_en), 
        .pulse(timer_pulse)
    ) ;

    always_ff @( posedge(clk_i) ) begin : obs_new_control
        if(timer_pulse) begin
            if(new_obs == '0) begin
                new_obs = '1 ;
            end else begin
                new_obs = new_obs << 1 ;
            end
        end
    end

    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_col ;
    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_row ; 
    logic CBL, CBLEN ; 
    logic WL_signal, SL_signal ;
    logic [1:0] instructions ; 
    logic  bit_out_1 [MatrixSize-1:0];
    logic [MatrixSize-1:0] bit_out ;
    assign bit_out = {bit_out_1[3], bit_out_1[2], bit_out_1[1], bit_out_1[0]} ;

    Bayesian_log2 #(
        .Narray(MatrixSizeLog2),
        .Nword(ArraySizeLog2)
    ) e_Bayesian_log (
        .clk(clk_i),
        .CBL0(CBL),
        .CBLEN0(CBLEN), // 1 when pcsa connected 0 when not
        .CSL0(SL_signal), // 1 when CWlen IS ENABLED 
        .CWL0(WL_signal), 
        .instructions_in(instructions), 
        
        .adr_full_col_in(addr_col),
        .adr_full_row_in(addr_row),
        .DATA_out(bit_out_1)

    ) ;


    // interface with Bayesian_stoch_log TODO: add a proper interface 
        /*
    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_col ;
    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_row ; 
    logic stoch_log, inference, load_seed, read_1, read_8, load_mem, read_out ;
    logic [DataWidth-1:0] seeds ; 
    logic [2**Nword_used -1 :0] seed_input ;
    logic [MatrixSize-1:0] bit_out ;
    logic CBL, CBLEN ; 
    logic WL_signal, SL_signal, precharge_en ;

    `ifdef VERILATOR // not on chip 
        Bayesian_stoch_log #(
            .Narray(MatrixSizeLog2),
            .Nword(ArraySizeLog2),
            .Nword_used(Nword_used)
        ) e_Bayesian_stoch_log (
            .clk(clk_i),
            .CBL(CBL),
            .CBLEN(CBLEN), // 1 when pcsa connected 0 when not
            .CSL(SL_signal), // 1 when CWlen IS ENABLED 
            .CWL(WL_signal),  

            .inference(inference),
            .load_seed(load_seed),
            .read_1(read_1),
            .read_8(read_8), // put t 1 for 8 bits 
            .load_mem(load_mem),
            .read_out(read_out),
            
            .adr_full_col(addr_col),
            .adr_full_row(addr_row),
            .stoch_log(stoch_log), 
            .seeds(seed_input),
            .bit_out(bit_out)
        ) ; 
    `endif 
    */

endmodule
