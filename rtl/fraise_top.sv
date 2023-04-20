// This is the top module of the accelerator to control the bayesian array developped by INTEGNANO

// author : Ourspalois, INTEGNANO, 2023

module fraise_top #(
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
    output logic [NbrHostsLog2-1:0] resp_ini_addr_o // the adress of the initiator of the request
) ;
    // logs 
    localparam int unsigned MatrixSizeLog2 = (MatrixSize == 1) ? 1 : $clog2(MatrixSize) ;
    localparam int unsigned ArraySizeLog2 = (ArraySize == 1) ? 1 : $clog2(ArraySize) ;
    

    // the accelerator resgisters adresses. 
    localparam int unsigned REG_START = 32'h0010 ; 
    localparam int unsigned MODE_REG = 32'h000 + REG_START ; // 0 iddle, 1 inference (continuous)
    localparam int unsigned SEED_REG = 32'h004 + REG_START ; 
    localparam int unsigned OBS_REG_1 = 32'h008 + REG_START ;
    localparam int unsigned OBS_REG_2 = 32'h00C + REG_START ;
    localparam int unsigned LAUNCH_REG = 32'h010 + REG_START ;
    localparam int unsigned RES_VALID_REG = 32'h014 + REG_START ;
    localparam int unsigned RES_REG = 32'h018 + REG_START ;

    logic [DataWidth-1:0] device_read_data;

    // broadcast byte enable to the 32 bits
    logic [DataWidth-1:0] req_ben_32 ;
    assign req_ben_32 = {{8{req_ben_i[3]}}, {8{req_ben_i[2]}}, {8{req_ben_i[1]}}, {8{req_ben_i[0]}}};

    typedef enum logic[DataWidth-1:0] { 
        Off, 
        On
    } mode_reg_e;
    
    /* verilator lint_off UNOPTFLAT */
    mode_reg_e mode_reg ; 

    logic [DataWidth-1:0] obs_reg_1 ;
    logic [DataWidth-1:0] obs_reg_2 ;
    logic [MatrixSize-1:0][8:0]Observation_vec ;
    assign Observation_vec = {obs_reg_2[24:16], obs_reg_2[8:0], obs_reg_1[24:16], obs_reg_1[8:0]} ; 
    logic launch_reg ; 
    logic res_valid ; 
    logic stay_high ;

    logic [MatrixSize-1:0][2**Nword_used-1:0] results ;

    // function to decode the input of the accelerator 
    always_ff @(posedge(clk_i)) begin : ReadInput
        if(req_valid_i) begin
            case(req_addr_i)
                MODE_REG: begin 
                    if(req_wen_i) begin
                        /* verilator lint_off ALWCOMBORDER */
                        mode_reg = mode_reg_e'(req_wdata_i) ;
                    end
                    device_read_data = mode_reg;
                end 
                SEED_REG: begin 
                    if(req_wen_i) begin
                        seeds = req_wdata_i[2**Nword_used-1:0];
                        // TODO: write seeds 
                    end 
                    device_read_data = {24'b0, seeds} ;
                end
                OBS_REG_1: begin 
                    if(req_wen_i) begin
                        obs_reg_1 = req_wdata_i & req_ben_32 ;
                    end
                    device_read_data = obs_reg_1 ;
                end
                OBS_REG_2: begin 
                    if(req_wen_i) begin
                        obs_reg_2 = req_wdata_i & req_ben_32 ;
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
                default: begin
                    `ifdef VERILATOR 
                        $display("FRAISE ERROR : addres out of range of the control registers : %h", req_addr_i);
                    `endif 
                end
            endcase
            resp_valid_o = 1'b1; 
            resp_ini_addr_o = req_host_addr_i; 
            resp_data_o = device_read_data;
            stay_high = 1'b1 ;
        end else begin
            if(stay_high ) begin 
                stay_high ='0 ;
                resp_valid_o = 1'b1;
            end else begin
            resp_valid_o = 1'b0;
            end


        end
    end

    // inference 

    typedef enum int { 
        Idle,
        Write_obs, 
        Run, 
        Done
     } inference_state_e;

    inference_state_e inference_state ; 
    genvar i ;

    always_ff @( posedge(clk_i) ) begin : Inference_machine
        case(inference_state)
            Idle: begin
                counter_reset = '0 ;
                if(mode_reg == On | launch_reg == '1) begin
                    inference_state = Write_obs;
                end 
            end
            Write_obs: begin
                ready_o <= '0 ; 
                launch_reg <= '0 ; 
                counter_en <= '1 ; 

                inference <= '1 ;
                addr_col <= {counter[1:0],Observation_vec[counter[1:0]][2:0], 3'b0} ; 
                addr_row <= {2'b0, Observation_vec[counter[1:0]][ArraySizeLog2 + 3 - 1:3]} ;

                if(counter == 8'(MatrixSize-1)) begin
                    counter_reset = '1 ;
                    inference_state = Run;
                end

            end
            Run: begin
                ready_o <= '1 ;
                inference <= '0 ;
                counter_reset = '0 ;
                counter_en <= '1 ;

                results[0] <= results[0] + 8'(bit_out[0]) ;
                results[1] <= results[1] + 8'(bit_out[1]) ;
                results[2] <= results[2] + 8'(bit_out[2]) ;
                results[3] <= results[3] + 8'(bit_out[3]) ;

                if(counter >= (2**8)-1) begin
                    counter_en <= '0 ;
                    inference_state = Done;
                end
            end
            Done: begin
                counter_reset = '1 ;
                res_valid <= '1 ; 
                inference_state = Idle;
            end
        endcase
    end

    // counter 

    logic [2**Nword_used-1:0] counter ;
    logic counter_reset, counter_en ; 

    always_ff @( posedge(clk_i) ) begin : compteur
        if(counter_reset) begin
            counter = '0 ; 
        end else if(counter_en) begin
            counter = counter + 1 ; 
        end
    end


    // interface with Bayesian_stoch_log TODO: add a proper interface 

    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_col ;
    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_row ; 
    logic stoch_log, inference, load_seed, read_1, read_8, load_mem, read_out ;
    logic [2**Nword_used-1:0] seeds ; 
    logic [MatrixSize-1:0] bit_out ;

    Bayesian_stoch_log #(
        .Narray(MatrixSizeLog2),
        .Nword(ArraySizeLog2),
        .Nword_used(Nword_used)
    ) e_Bayesian_stoch_log (
        .clk(clk_i),
        .CBL(1'b0),
        .CBLEN(1'b0),
        .CSL(1'b0),
        .CWL(1'b0),

        .inference(inference),
        .load_seed(load_seed),
        .read_1(read_1),
        .read_8(read_8),
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
            inference_state = Idle;
            counter_reset = '0 ;
            counter_en <= '0 ;
            mode_reg = Off ;
            launch_reg <= '0 ;
            res_valid <= '0 ;
            ready_o <= '1 ;
            inference <= '0 ;
            load_seed <= '0 ;
            read_1 <= '0 ;
            read_8 <= '0 ;
            load_mem <= '0 ;
            read_out <= '0 ;
            obs_reg_1 = '0 ;
            obs_reg_2 = '0 ;
        end
    end



endmodule