// This is the top module of the accelerator to control the bayesian array developped by INTEGNANO

// author : Ourspalois, INTEGNANO, 2023

module fraise_top  #(
    parameter int unsigned DataWidth = 32, 
    parameter int unsigned AddrWidth = 32,
    parameter int unsigned MatrixSize = 4, // I consider them square
    parameter int unsigned ArraySize = 64, 
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
    output logic irq_o // interrupt signal
) ;
    //pkg 
    import comparator_pkg::*;

    // logs 
    localparam int unsigned MatrixSizeLog2 = (MatrixSize == 1) ? 1 : $clog2(MatrixSize) ;
    localparam int unsigned ArraySizeLog2 = (ArraySize == 1) ? 1 : $clog2(ArraySize) ;
    

    // the accelerator resgisters adresses. 
    localparam int unsigned REG_START = 32'h00010 ; 
    localparam int unsigned ON_OFF_REG = 32'h000 + REG_START ; // 0 iddle, 1 inference (continuous)
    localparam int unsigned SEED_REG = 32'h004 + REG_START ; 
    localparam int unsigned OBS_REG_1 = 32'h008 + REG_START ;
    localparam int unsigned OBS_REG_2 = 32'h00C + REG_START ;
    localparam int unsigned LAUNCH_REG = 32'h010 + REG_START ;
    localparam int unsigned RES_VALID_REG = 32'h014 + REG_START ;
    localparam int unsigned RES_REG = 32'h018 + REG_START ;
    localparam int unsigned MODE_REG = 32'h01C + REG_START ; // 0 : stachastic, 1 : logarithmic
    localparam int unsigned IRQ_EN_REG = 32'h020 + REG_START ; // 0 : no irq, 1 : irq when res is valid
    localparam int unsigned PRECISION_REG = 32'h024 + REG_START ; 
    localparam int unsigned COMP_INSTRUCTION_REG = 32'h028 + REG_START ;
    localparam int unsigned COMP_REFERENCE = 32'h02C + REG_START ;
    localparam int unsigned COMP_RESULT = 32'h030 + REG_START ;
    localparam int unsigned COMP_BYPASS = 32'h034 + REG_START ;


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

    logic [DataWidth-1:0] obs_reg_1 ;
    logic [DataWidth-1:0] obs_reg_2 ;
    logic [MatrixSize-1:0][8:0]Observation_vec ;
    assign Observation_vec = {obs_reg_2[24:16], obs_reg_2[8:0], obs_reg_1[24:16], obs_reg_1[8:0]} ; 
    logic launch_reg ; 
    logic res_valid, irq_en ; 
    logic mode ; // 0 : stachastic, 1 : logarithmic
    logic stay_high ;

    logic [MatrixSize-1:0][2**Nword_used-1:0] results ;

    // function to decode the input of the accelerator 
    always_ff @(posedge(clk_i)) begin : ReadInput
        if(req_valid_i) begin
            case(req_addr_i)
                ON_OFF_REG: begin 
                    if(req_wen_i) begin
                        /* verilator lint_off ALWCOMBORDER */
                        ON_OFF_reg <= ON_OFF_reg_e'(req_wdata_i) ;
                    end
                    device_read_data = ON_OFF_reg;
                end 
                SEED_REG: begin 
                    if(req_wen_i) begin
                        seeds <= req_wdata_i[2**Nword_used-1:0];
                    end 
                    device_read_data = {24'b0, seeds} ;
                end
                OBS_REG_1: begin 
                    if(req_wen_i) begin
                        obs_reg_1 <= req_wdata_i & req_ben_32 ;
                    end
                    device_read_data = obs_reg_1 ;
                end
                OBS_REG_2: begin 
                    if(req_wen_i) begin
                        obs_reg_2 <= req_wdata_i & req_ben_32 ;
                    end
                    device_read_data = obs_reg_2 ;
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
                MODE_REG: begin 
                    if(req_wen_i) begin
                        mode <= req_wdata_i[0] ;
                    end
                    device_read_data = {31'b0, mode} ;
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
                default: begin
                    `ifdef VERILATOR 
                        $display("FRAISE ERROR : addres out of range of the memory : %h", req_addr_i);
                    `endif
                end
            endcase
            resp_valid_o <= 1'b1; 
            resp_ini_addr_o <= req_host_addr_i; 
            resp_data_o <= device_read_data;
            
        end else begin          
            resp_valid_o <= 1'b0;
        end
    end

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
                .instruction(comp_instr[i]),
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


    // inference 

    typedef enum int { 
        Idle,
        Read_cycles,
        Run_stoch, 
        Read_out,
        Run_log,
        Done
     } inference_state_e;

    inference_state_e inference_state ; 

    typedef enum int { 
        SL_WL_rise,
        Sl_fall,
        WLfall
    } read_state_e ;

    read_state_e read_state ;

    always_ff @( posedge(clk_i) ) begin : Inference_machine
        
        case(inference_state)
            Idle: begin
                read_state <= SL_WL_rise ;
                counter <= '0 ;
                counter_run_reset <= '0 ;
                if(ON_OFF_reg == On | launch_reg == '1) begin
                    inference_state <= Read_cycles;
                    read_8 <= '1 ;
                end 
            end
            Read_cycles: begin
                ready_o <= '0 ; 
                launch_reg <= '0 ; 
                stoch_log <= mode ;
                inference <= '1 ;
                addr_col = {counter[1:0],Observation_vec[counter[1:0]][2:0] ,3'b0} ; 
                addr_row = {2'b0, Observation_vec[counter[1:0]][ArraySizeLog2 + 3 - 1:3]} ;
                case (read_state)
                    SL_WL_rise : begin
                        WL_signal <= '1 ;
                        SL_signal <= '1 ;
                        read_state <= Sl_fall ;
                    end 
                    Sl_fall : begin
                        SL_signal <= '0 ;
                        read_state <= WLfall ;
                    end
                    WLfall : begin
                        WL_signal <= '0 ;
                        read_state <= SL_WL_rise ;
                        if(counter == 8'(MatrixSize-1)) begin
                            inference_state <= (mode == 0) ? Run_stoch : Read_out ;
                            counter_run_en <= '1 ;
                        end 
                        counter <= counter + 1 ;
                    end
                endcase
                
            end
            Run_stoch: begin

                results[0] <= results[0] + 8'(bit_out[0]) ;
                results[1] <= results[1] + 8'(bit_out[1]) ;
                results[2] <= results[2] + 8'(bit_out[2]) ;
                results[3] <= results[3] + 8'(bit_out[3]) ;

                if(counter_run >= (2**8)-1) begin
                    counter_run_en <= '0 ;
                    inference_state <= Done;
                end 
            end
            Read_out:begin
                read_out <= '1 ;
                inference <= '0 ;
                inference_state <= Run_log ;
            end
            Run_log: begin

                results[0] <= results[0] | ({7'b0, bit_out[0]} << counter) ;
                results[1] <= results[1] | ({7'b0, bit_out[1]} << counter) ;
                results[2] <= results[2] | ({7'b0, bit_out[2]} << counter) ;
                results[3] <= results[3] | ({7'b0, bit_out[3]} << counter) ;

                if(counter_run >= 7) begin
                    counter_run_en <= '0 ;
                    inference_state <= Done;
                end
            end
            Done: begin
                ready_o <= '1 ;
                read_8 <= '0 ;
                read_out <= '0 ;
                inference <= '0 ;
                counter_run_reset <= '1 ;
                res_valid <= '1 ; 
                inference_state <= Idle;
            end
        endcase
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




    // interface with Bayesian_stoch_log TODO: add a proper interface 

    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_col ;
    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_row ; 
    logic stoch_log, inference, load_seed, read_1, read_8, load_mem, read_out ;
    logic [2**Nword_used-1:0] seeds ; 
    logic [MatrixSize-1:0] bit_out ;
    logic CBL, CBLEN ; 
    logic WL_signal, SL_signal, precharge_en ;
    
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
        .seeds(seeds),
        .bit_out(bit_out)
    ) ; 
    
    // reset management 

    always_ff @(posedge(clk_i)) begin : Reset_management
        if(!reset_n) begin
            inference_state <= Idle;
            counter <= '0 ;
            counter_run_reset <= '0 ;
            counter_run_en <= '0 ;
            ON_OFF_reg <= Off ;
            launch_reg <= '0 ;
            res_valid <= '0 ;
            ready_o <= '1 ;
            load_seed <= '0 ;
            inference <= '0 ;
            load_seed <= '0 ;
            read_1 <= '0 ;
            read_8 <= '0 ;
            load_mem <= '0 ;
            read_out <= '0 ;
            obs_reg_1 <= '0 ;
            obs_reg_2 <= '0 ;
        end
    end



endmodule
