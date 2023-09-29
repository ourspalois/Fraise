// This is the top module of the accelerator to control the bayesian array developped by INTEGNANO

// author : Ourspalois, INTEGNANO, 2023
// modification : C.Turck 2023; adaptation to Maxi Bayes (4x4 and 64x64 memory) 

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

    `ifndef VERILATOR
    , output logic CBL, 
    output logic CBLEN,
    output logic SL_signal,
    output logic WL_signal,
    output logic  inference, // Activation de l'inférence
    output logic  load_seed, // Chargement des seeds
    output logic  read_1, // Lecture de 1 bit
    output logic  read_8, // Lecture de 8 bits
    output logic  load_mem, // Programmation de la mémoire
    output logic  read_out, // Envoi de la sortie de lecture ou d'inférence 
    output logic  stoch_log, // Mode de calcul stochastique ou logarithmique
    output logic  [7:0] seeds, // Seeds pour le calcul stochastique
    output logic [7:0] addr_col,
    output logic [7:0] addr_row,
    input logic bit_out [MatrixSize-1:0]

    `endif
) ;
    //pkg 
    import comparator_pkg::*;

    // logs 
    localparam int unsigned MatrixSizeLog2 = (MatrixSize == 1) ? 1 : $clog2(MatrixSize) ;
    localparam int unsigned ArraySizeLog2 = (ArraySize == 1) ? 1 : $clog2(ArraySize) ;
    

    // the accelerator resgisters adresses. 
    localparam int unsigned REG_START = 32'h00010 ; // 
    localparam int unsigned ON_OFF_REG = 32'h000 + REG_START ;// // 0 iddle, 1 inference (continuous)
    localparam int unsigned PROG       = 32'h008 + REG_START ; 
    localparam int unsigned READ       = 32'h00C + REG_START ;
    localparam int unsigned INFERENCE  = 32'h010 + REG_START ;// // choix stoch/log/power conscious
    localparam int unsigned LAUNCH_INF = 32'h014 + REG_START ;//
    localparam int unsigned RECUP_RES  = 32'h018 + REG_START ;// // stoch
    localparam int unsigned RECUP_RES_2= 32'h01C + REG_START ;// // power conscious
    localparam int unsigned RECUP_RES_3= 32'h020 + REG_START ;// // log
    localparam int unsigned OBS_COL    = 32'h024 + REG_START ;// // observations pour l'inference
    localparam int unsigned OBS_ROW    = 32'h028 + REG_START ;// // observations pour l'inference
    localparam int unsigned WRITING_SET_RESET = 32'h02C + REG_START ;//


    localparam int unsigned MEM_ARRAY_START = 32'h00100 ; // word size of this is 32 bits
    localparam int unsigned MEM_ARRAY_END = 32'h200FF ;

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

    logic [DataWidth-1:0] obs_reg_col ;
    logic [DataWidth-1:0] obs_reg_row ;
    logic [MatrixSize-1:0][5:0]Observation_vec_row ;
    logic [MatrixSize-1:0][2:0]Observation_vec_col ;
    assign Observation_vec_col = {obs_reg_col[26:24], obs_reg_col[18:16], obs_reg_col[10:8], obs_reg_col[2:0]} ; 
    assign Observation_vec_row = {obs_reg_row[29:24], obs_reg_row[21:16], obs_reg_row[13:8], obs_reg_row[5:0]} ; 
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
    logic [7:0] adr_col_lfsr [3:0];
    logic [7:0] seeds_input [3:0];
    initial begin
        adr_col_lfsr[0] = 8'b00000000;
        adr_col_lfsr[1] = 8'b01000000;
        adr_col_lfsr[2] = 8'b10000000;
        adr_col_lfsr[3] = 8'b11000000;
        seeds_input[0] = 8'b11101011;
        seeds_input[1] = 8'b11111011;
        seeds_input[2] = 8'b01111111;
        seeds_input[3] = 8'b01011100;
    end

    typedef enum int { 
        Inference, 
        Writting, 
        Reading
    } inference_write_e;

    inference_write_e inference_write ;
    inference_write_e old_inference_write ; 

    typedef enum int { // TODO: clean up this SM
        Idle,
        Run_load_seeds,
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

    logic [NbrHostsLog2-1:0] read_host_addr ; 
    logic wait_read ; 
    logic [AddrWidth-1:0] read_addr ;
    logic read_is_done ; 

    logic [6:0] count_reads ; 
    logic [31:0] result_read ;
    logic [7:0] count_stoch;
    logic [255:0] results_stoch_1;
    logic [255:0] results_stoch_2;
    logic [255:0] results_stoch_3;
    logic [255:0] results_stoch_4;
    logic [31:0] results_stoch_sum;
    logic [31:0] results_stoch_first;
    logic type_inf;

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
                PROG: begin
                    if (req_wen_i) begin
                        inference_write <= Writting ;
                    end
                    device_read_data = inference_write ;
                end
                READ: begin
                    if(req_wen_i) begin
                        obs_reg_col <= req_wdata_i & req_ben_32 ;
                    end
                    device_read_data = obs_reg_col ;
                end
                INFERENCE: begin 
                    if(req_wen_i) begin
                        type_inf <= req_wdata_i[0] ;
                        inference_write <= Inference;
                    end
                end
                LAUNCH_INF: begin
                    if(req_wen_i) begin
                        launch_reg <= req_wdata_i[0] ;
                    end
                    device_read_data = {31'b0, launch_reg} ;
                end
                RECUP_RES: begin
                    device_read_data = results_stoch_sum ;
                    res_valid <= '0 ;
                    launch_reg <= '0;
                end
                RECUP_RES_2: begin
                    device_read_data = results_stoch_first ;
                    res_valid <= '0 ;
                    launch_reg <= '0;
                end
                RECUP_RES_3: begin
                    device_read_data = results ;
                    res_valid <= '0 ;
                    launch_reg <= '0;
                end
                OBS_COL: begin 
                    if(req_wen_i) begin
                        obs_reg_col <= req_wdata_i & req_ben_32 ;
                    end
                    device_read_data = obs_reg_col ;
                end
                OBS_ROW: begin 
                    if(req_wen_i) begin
                        obs_reg_row <= req_wdata_i & req_ben_32 ;
                    end
                    device_read_data = obs_reg_row ;
                end
                WRITING_SET_RESET: begin
                    if(req_wen_i)begin
                        set_reset <= req_wdata_i[0] ;
                    end
                end
                default: begin
                    if(req_addr_i >= MEM_ARRAY_START & req_addr_i <= MEM_ARRAY_END) begin
                        if(req_wen_i) begin
                            data_to_write <= req_wdata_i ;
                            write_addr <= (req_addr_i -  MEM_ARRAY_START)>>2;
                            write_en <= 'b1 ;
                        end else begin
                            wait_read = '1 ;
                            read_host_addr <= req_host_addr_i; 
                            read_addr <= (req_addr_i -  MEM_ARRAY_START)>> 2 ;
                            old_inference_write = inference_write ; 
                            inference_write <= Reading ; 
                            count_reads = 0 ;
                            result_read <= '0 ;
                        end
                    end else begin
                        `ifdef VERILATOR 
                            $display("FRAISE ERROR : address out of range of the memory : %h", req_addr_i);
                        `endif
                    end
                end
            endcase
            if(wait_read == '0) begin
                resp_valid_o <= 1'b1; 
                resp_ini_addr_o <= req_host_addr_i; 
                resp_data_o <= device_read_data;
            end
        end else if(wait_read == '1) begin
            if(read_is_done) begin
                resp_valid_o <= 1'b1 ;
                resp_ini_addr_o <= read_host_addr ;
                resp_data_o <= result_read ;
                wait_read = '0 ;
                read_is_done <= '0 ;
            end
        end else begin          
            resp_valid_o <= 1'b0;
        end
        // inference
        case (inference_write)
            Reading: begin
            case(inference_state)
                Idle: begin
                    CBLEN <= '0 ;
                    CBL <= '0 ; 
                    read_state <= SL_WL_rise ;
                    counter_run_reset <= '0 ;
                    inference_state <= Read_cycles_log ;
                    results <= '0 ;
                    inference <= '0; // Activation de l'inférence
                    load_seed <= '0; // Chargement des seeds
                    read_1 <= '0; // Lecture de 1 bit
                    read_8 <= '0; // Lecture de 8 bits
                    load_mem <= '0; // Programmation de la mémoire
                    read_out <= '0; // Envoi de la sortie de lecture ou d'inférence
                    stoch_log <= '0; // Mode de calcul stochastique ou logarithmique
                    seeds <= '0; // Seeds pour le calcul stochastique
                    ready_o <= '1 ;
                end
                Read_cycles_log: begin
                    ready_o <= '0 ;
                    launch_reg <= '0 ; 
                    read_8 <= '1;
                    stoch_log <= '1;
                    read_out <= '0;
                    //addr_col = {read_addr[4:3], read_addr[0] , count_reads[1:0] } ; 
                    //addr_row = {read_addr[2:1], 3'b0} ;
                    addr_col = {read_addr[2:1], 3'b0, read_addr[0], count_reads[1:0]}; // 2bits of likelihoods adress and 1 bit for the 32 half of the array and 5 for the 32 bits
                    addr_row = {read_addr[10:3]}; // 8 bits of adress : 2 for likelihoods and 6 for memory
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
                            inference_state <= Read_out ;
                            counter_run_en <= '1 ;
                        end
                    endcase
                end
                Read_out:begin
                    read_8 <= '0;
                    read_out <= '1;
                    inference <= '1;
                    if(counter_run >= 3) begin
                        inference_state <= Run_log ;
                    end
                end
                Run_log: begin
                    read_out <= '1;
                    inference <= '1;
                    case(read_addr[10:9]) 
                        2'b00 : result_read[count_reads*8 +: 8] <= result_read[count_reads*8 +: 8] | ({7'b0, bit_out[0]} << (7-(counter_run-4))) ;
                        2'b01 : result_read[count_reads*8 +: 8] <= result_read[count_reads*8 +: 8] | ({7'b0, bit_out[1]} << (7-(counter_run-4))) ;
                        2'b10 : result_read[count_reads*8 +: 8] <= result_read[count_reads*8 +: 8] | ({7'b0, bit_out[2]} << (7-(counter_run-4))) ;
                        2'b11 : result_read[count_reads*8 +: 8] <= result_read[count_reads*8 +: 8] | ({7'b0, bit_out[3]} << (7-(counter_run-4))) ;
                    endcase
                    if(counter_run >= 13) begin
                        counter_run_en <= '0 ;
                        inference_state <= Done;
                    end
                end
                Done: begin
                    read_out <= '0 ; 
                    count_reads = count_reads + 1 ;
                    if(count_reads >= 4) begin
                        inference_write <= old_inference_write ; 
                        read_is_done <= '1 ;
                        ready_o <= '1 ;
                    end
                    counter_run_reset <= '1 ;
                    inference_state <= Idle;
                end
            endcase
            end
            Inference: begin        
            case(inference_state)
                Idle: begin
                    read_state <= SL_WL_rise ;
                    inference <= '0; // Activation de l'inférence
                    load_seed <= '0; // Chargement des seeds
                    read_1 <= '0; // Lecture de 1 bit
                    read_8 <= '0; // Lecture de 8 bits
                    load_mem <= '0; // Programmation de la mémoire
                    read_out <= '0; // Envoi de la sortie de lecture ou d'inférence
                    stoch_log <= '0; // Mode de calcul stochastique ou logarithmique
                    seeds <= '0; // Seeds pour le calcul stochastique
                    counter <= '0 ;
                    counter_run_reset <= '0 ;
                    if(ON_OFF_reg == On & launch_reg == '1) begin
                        
                        // add a if to use either stoch or log inference
                        if (type_inf == '0) begin
                            inference_state <= Run_load_seeds;
                            results_stoch_first <= '0;
                            results_stoch_sum <= '0;
                            stoch_log <= '0;
                        end
                        else begin
                            inference_state <= Read_cycles_log;
                            results <= '0;
                            stoch_log <= '1;
                        end
                    end
                end
                Run_load_seeds: begin
                    load_seed <= '1;
                    CBLEN <= 1'b0;
					WL_signal <= 1'b0;
					SL_signal <= 1'b0;
					CBL <= 1'b0;
					//addr_row <= '0;
                    counter <= counter + 1;
                    if (counter == 0) begin
						addr_col = adr_col_lfsr[0];
						seeds <= seeds_input[0];
					end
					if (counter == 1) begin
						addr_col = adr_col_lfsr[1];
						seeds <= seeds_input[1];
					end
					if (counter == 2) begin
						addr_col = adr_col_lfsr[2];
						seeds <= seeds_input[2];
					end
					if (counter == 3) begin
						addr_col = adr_col_lfsr[3];
						seeds <= seeds_input[3];
						//counter <= '0;
						
					end
					if (counter == 4) begin
						counter <= '0;
						load_seed <= '0; // fin du load des seeds
                        inference_state <= Read_cycles_stoch;
					end

                end
                Read_cycles_stoch: begin
                    read_1 <= '1;
                    load_seed <= '0;
                    inference <= '0;
                    ready_o <= '0 ;
                    launch_reg <= '0 ;
                    addr_col = {counter[1:0], 3'b0, Observation_vec_col[counter[1:0]]} ; 
                    addr_row = {2'b0, Observation_vec_row[counter[1:0]]} ;
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
                                inference_state <= Run_stoch ;
                                //counter_run_en <= '1 ;
                                counter <= '0;
                            end 
                            counter <= counter + 1 ;
                        end
                    endcase
                end
                Run_stoch: begin
                    read_1 <= 0;
                    inference <= 1;
                    count_stoch <= count_stoch + 1;
                    results_stoch_1[count_stoch] <= bit_out[0];
                    results_stoch_2[count_stoch] <= bit_out[1];
                    results_stoch_3[count_stoch] <= bit_out[2];
                    results_stoch_4[count_stoch] <= bit_out[3];
                    if (count_stoch == 255) begin
                        count_stoch <= '0;
                        inference <= '0;
                        inference_state <= Idle;
                        ON_OFF_reg <= Off;
                        ready_o <= '1 ;
                    end
                end
                Read_cycles_log: begin
                    ready_o <= '0 ;
                    launch_reg <= '0 ; 
                    read_8 <= '1; // Lecture de 8 bits
                    addr_col = {counter[1:0], 3'b0, Observation_vec_col[counter[1:0]]} ; 
                    addr_row = {2'b0, Observation_vec_row[counter[1:0]]} ;
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
                                inference_state <= Inf_cycles ;
                                //counter_run_en <= '1 ;
                                counter <= '0;
                            end
                            else begin 
                                counter <= counter + 1 ;
                            end
                        end
                    endcase
                end
                Inf_cycles:begin
                    read_8 <= '0;
                    inference <= '1;
                    if(counter == 3) begin
                        inference_state <= Read_out ;
                        counter_run_en <= '1 ;
                    end 
                    counter <= counter + 1 ;
                end
                Read_out:begin
                    read_out <= '1;
                    if(counter_run >= 3) begin
                        inference_state <= Run_log ;
                    end
                end
                Run_log: begin
                    read_out <= '1;
                    results[0] <= results[0] | ({7'b0, bit_out[0]} << (7-(counter_run-4))) ;
                    results[1] <= results[1] | ({7'b0, bit_out[1]} << (7-(counter_run-4))) ;
                    results[2] <= results[2] | ({7'b0, bit_out[2]} << (7-(counter_run-4))) ;
                    results[3] <= results[3] | ({7'b0, bit_out[3]} << (7-(counter_run-4))) ;

                    if(counter_run >= 12) begin
                        counter_run_en <= '0 ;
                        inference_state <= Done;
                    end
                end
                Done: begin
                    counter_run_reset <= '1 ;
                    res_valid <= '1 ; 
                    ready_o <= '1 ;
                    inference_state <= Idle;
                    ON_OFF_reg <= Off;
                end
            endcase
            end
            Writting:begin
                if(write_en) begin
                CBLEN <= '1 ;
                load_mem <= '1; // write mode
                //addr_col = {write_addr[4:3], write_addr[0], write_counter[4:3]} ;
                //addr_row = {write_addr[2:1], write_counter[2:0]} ;
                addr_col = {write_addr[2:1], write_addr[0], write_counter[4:0]}; // [2:1] 2bits of likelihoods adress and [0] 1 bit for the 32 half of the array
                addr_row = {write_addr[10:3]}; // 8 bits of adress : [10:9] 2 for likelihoods and [8:3] 6 for memory
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
    `ifndef VERILATOR
        logic WL_signal, SL_signal ;
    `endif
    `ifdef VERILATOR // not on chip 

        logic CBL, CBLEN ; 
        logic CWL, CSL ; 
        logic  inference; // Activation de l'inférence
        logic  load_seed; // Chargement des seeds
        logic  read_1; // Lecture de 1 bit
        logic  read_8; // Lecture de 8 bits
        logic  load_mem; // Programmation de la mémoire
        logic  read_out; // Envoi de la sortie de lecture ou d'inférence 
        logic  stoch_log; // Mode de calcul stochastique ou logarithmique
        logic  [7:0] seeds; // Seeds pour le calcul stochastique
        //logic  bit_out_1 [MatrixSize-1:0];
        logic [MatrixSize-1:0] bit_out ;
    
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
    `endif 
endmodule
