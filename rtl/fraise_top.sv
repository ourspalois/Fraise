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

    // TODO: get rid of this ready shinanigan 
    always_comb begin : ready
        ready_o = '1 ;
    end
    // the accelerator resgisters adresses. 
    localparam int unsigned OBS_START = 32'h1000 ; 
    localparam int unsigned REG_START = 32'h0 ; 
    localparam int unsigned MODE_REG = 32'h000 + REG_START ; // 0 iddle, 1 inference (continuous)
    localparam int unsigned RESULT_PTR_REG = 32'h004 + REG_START;

    logic [DataWidth-1:0] device_read_data;
    /* verilator lint_off UNOPTFLAT */
    logic [AddrWidth-1:0] result_ptr;
    
    typedef enum logic[DataWidth-1:0] { 
        Off, 
        On
    } mode_reg_e;
    
    /* verilator lint_off UNOPTFLAT */
    mode_reg_e mode_reg ; 

    localparam int unsigned MatrixSizeLog2 = (MatrixSize == 1) ? 1 : $clog2(MatrixSize) ;
    localparam int unsigned ArraySizeLog2 = (ArraySize == 1) ? 1 : $clog2(ArraySize) ;
    logic [MatrixSizeLog2-1:0] matrix_column ; 
    logic [MatrixSizeLog2-1:0] matrix_line ; 
    logic [ArraySizeLog2-1:0] array_column ;
    logic [ArraySizeLog2-1:0] array_line ;
    logic [MatrixSize-1:0][ArraySizeLog2 + 3-1:0] Observation_vec ; // the vector of observations

    // function to decode the input of the accelerator 
    always_ff @(posedge(clk_i)) begin : ReadInput
        if(req_valid_i) begin
            case(req_addr_i)
                RESULT_PTR_REG: begin
                    
                    if(req_wen_i) begin
                        /* verilator lint_off ALWCOMBORDER */
                        result_ptr = req_wdata_i ; //TODO : respect BEN
                    end 
                    device_read_data = result_ptr;
                end
                MODE_REG: begin
                    
                    if(req_wen_i) begin
                        /* verilator lint_off ALWCOMBORDER */
                        mode_reg = mode_reg_e'(req_wdata_i) ;
                    end
                    device_read_data = mode_reg;
                end
                default: begin
                    // add error flag in verilator

                    // sel if in array or in observations 
                    // for now everything is an obs at this point
                    if(req_wen_i) begin
                        // an observation takes 2Bytes 
                        Observation_vec[32'(req_addr_i[MatrixSizeLog2:1])>>1] = req_wdata_i[ArraySizeLog2 + 3-1:0];
                    end
                end
            endcase
            resp_valid_o = 1'b1; 
            resp_ini_addr_o = req_host_addr_i; 
            resp_data_o = device_read_data;
        end else begin
            resp_valid_o = 1'b0;
        end
    end

    // state machine for taking in observations 
    
    typedef enum int { 
        Idle,
        send_obs_addr,
        average_output, // averages the output of the array (full stoch for now)
        send_obs_data
     } observation_state_e;    

    observation_state_e observation_state = Idle; 
    logic [MatrixSizeLog2-1:0] compteur_observation ; 
    logic count_obs_reset, count_obs_enable;
    logic [Nword_used-1:0] compteur_result ;
    logic count_result_reset, count_result_enable;
    logic [Nword_used**2-1:0] result ;

    always_ff @( posedge(clk_i) ) begin : Inference
        case(observation_state) 
            Idle: begin
                count_obs_reset = 1'b1;
                if(mode_reg == On) begin
                    observation_state = send_obs_addr;
                end
            end
            send_obs_addr: begin
                count_obs_reset = 1'b0;
                count_obs_enable = 1'b1; 
                inference = 1'b1;
                count_result_reset = 1'b1;

                addr_col = {compteur_observation,Observation_vec[compteur_observation][2:0], 3'b0} ; 
                addr_row = {2'b0, Observation_vec[compteur_observation][ArraySizeLog2 + 3 - 1:3]} ;

                if(compteur_observation == 2'(ArraySize-1)) begin
                    observation_state = average_output;
                end
            end
            average_output: begin
                count_obs_enable = 1'b0;
                count_result_reset = 1'b0;
                count_result_enable = 1'b1;

                result = (result + 8'(bit_out)); 

                if(compteur_result == 3'(Nword_used-1)) begin
                    observation_state = send_obs_data;
                end
            end
            send_obs_data : begin
                count_result_enable = 1'b0;
                
                if(compteur_result == 3'(Nword_used-1)) begin
                    observation_state = Idle;
                end
            end

        endcase
        
    end

    always_ff @( posedge(clk_i) ) begin : count
        if(count_obs_reset) begin
            compteur_observation <= 0;
        end
        else if(count_obs_enable) begin
            compteur_observation <= compteur_observation + 1;
        end
    end

    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_col ;
    logic [ArraySizeLog2 + MatrixSizeLog2 -1 :0] addr_row ; 
    logic stoch_log, inference, load_seed, read_1, read_8, load_mem, read_out ;
    logic [2**Nword_used-1:0] seeds ; 
    logic [MatrixSize-1:0] bit_out ;

    Bayesian_stoch_log #(
        .Narray(MatrixSizeLog2),
        .Nword(ArraySizeLog2),
        .Nword_used(Nword_used), 
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
    

endmodule