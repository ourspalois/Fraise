`include "macros.svh"

module PWR_CTRL #(
    parameter int reg_num = 16,
    `ADAM_CFG_PARAMS
)(
    ADAM_SEQ.Slave seq_port,
    AXI_LITE.Slave axi_port, 
    AXI_LITE.Master axi_master
  );
  // TODO : add MEMs for clk gating 

  logic clk ; 
  assign clk = seq_port.clk;

  reg [31:0] registers [0:reg_num-1]; // 16 registers of 32 bits
  reg [31:0] registers_14 ; 
  //0 : state 
  // for the pwr registers I use 4bits for components adressing, and 2 bit for the state (so 5 updates on state change max )

  //1 IDLE to WAIT
  //1 HSDOM off, LPCPU off, LPMEM off, (NVMaccels)
  //2 WAIT to FETCH
  //2 
  //3 FETCH toCOMPUTE 
  //3 (NVMaccels)
  //4 COMPUTE to WAIT
  //4 
  //5 COMPUTE to IDLE
  //5 HSDOM on, LPCPU on, LPMEM on, (NVMaccels)

  //6 .. A : observation adresses 

  //10 wait counter lenght 

  //11 .. 14 : observation sources
  //15 : alarm levels
  
  // AXIl slave management
  logic read_regs ;
  logic [10:0] read_addr ;// western reading style, likelihood arrays are continious in memory

  logic write_regs ;
  logic [10:0] write_addr ;// western reading style, likelihood arrays are continious in memory
  logic [31:0] write_data ;

  logic ready ; 
  assign ready = ~(read_regs || write_regs) ;
  logic pwr_launch ;  

  typedef enum int { IDLE, WAIT, FETCH, COMPUTE } state_t;
  state_t state ;

  always_ff @( posedge clk) begin 
    if(seq_port.rst) begin
      registers[0] <= 32'h0;
      //transitions
      registers[1] <= 32'hffff_ffff;
      registers[1][0+:6] <= 6'b10_0000; // CUT CPU
      registers[1][6+:6] <= 6'b10_0001; // CUT LS RAM 
      registers[1][12+:6] <= 6'b11_1111;
      registers[2] <= 32'hffff_ffff;
      registers[3] <= 32'hffff_ffff;
      registers[4] <= 32'hffff_ffff;
      registers[5] <= 32'hffff_ffff;
      registers[5][0+:6] <= 6'b11_0000; // WAKE CPU 
      registers[5][6+:6] <= 6'b11_0001; // WAKE LS RAM
      //obs destinations
      registers[6] <= MMAP_ACCEL.start + 32'h2000 + 3*4 ; 
      registers[7] <= MMAP_ACCEL.start + 32'h2000 + 4*4 ;
      registers[8] <= MMAP_ACCEL.start + 32'h2000 + 5*4 ;
      registers[9] <= MMAP_ACCEL.start + 32'h2000 + 6*4 ;

      //wait lenght
      registers[10] <= 32'h0;

      //obs sources 
      registers[11] <= 32'h0 ;
      registers[12] <= 32'h1 ;
      registers[13] <= 32'h2 ;

      registers[15] <= 32'h00_00_00_00; 


      axi_port.r_valid <= 1'b0;
      axi_port.b_valid <= 1'b0;
      axi_port.ar_ready <= 1'b0;
      axi_port.aw_ready <= 1'b0;
      axi_port.w_ready <= 1'b0;
      read_regs <= 1'b0;
      write_regs <= 1'b0;
      axi_port.b_resp <= 'b0;
      axi_port.r_resp <= 0;
      axi_port.r_data <= 32'h0;
      pwr_launch <= 'b0;
      read_addr <= 'b0;
      write_addr <= 'b0;
      write_data <= 32'h0;
    end
    else begin
      // read management
      if(axi_port.ar_valid && ready) begin
        if(axi_port.ar_addr < 32'h2000) begin
          read_regs <= 1'b1;
        end 
        axi_port.ar_ready <= 1'b1;
        read_addr <= axi_port.ar_addr[2+:11];
      end else begin
        axi_port.ar_ready <= 1'b0;
      end

      // read response management
      if(axi_port.r_ready) begin
        if(read_regs) begin
          if(read_addr == 14 )begin
            axi_port.r_data <= registers_14;
          end else begin
            axi_port.r_data <= registers[read_addr[0+:4]];
          end
          axi_port.r_valid <= 1'b1;
          axi_port.r_resp <= 0;
          read_regs <= 1'b0;
        end else begin
          axi_port.r_valid <= 1'b0;
          axi_port.r_resp <= 0;
        end
      end

      // write management
      if(axi_port.aw_valid && axi_port.w_valid && ready) begin
        if(axi_port.aw_addr < 31'h2000) begin
          write_regs <= 1'b1;
        end
        axi_port.aw_ready <= 1'b1;
        axi_port.w_ready <= 1'b1;
        write_addr <= axi_port.aw_addr[2+:11];
        write_data <= axi_port.w_data;
      end else begin
        axi_port.aw_ready <= 1'b0;
        axi_port.w_ready <= 1'b0;
      end

      // write response management
      if(write_regs && axi_port.b_ready) begin
        if (write_addr[0+:3] == 3'b0) begin
          $display("PWR_CTRL : launching PWR FSM");
          pwr_launch <= 1;
        end 
        registers[write_addr[0+:4]] <= write_data;
        axi_port.b_resp <= 0;
        axi_port.b_valid <= 1'b1;
        write_regs <= 1'b0;
      end else begin
        axi_port.b_valid <= 1'b0;
        if(state != IDLE) begin
          pwr_launch <= 0;
        end
      end
    end
  end

  //PWR FSM 
  typedef enum int { FETCH_IDLE, GET_SEND, GET_RECEIVE, WRITE } FETCH_t;
  FETCH_t fetch_state ;
  logic [31:0] fetch_counter ;
  logic [31:0] fetch_data ;

    // wait counter
  logic counter_done ;
  logic [31:0] counter ;
  assign counter_done = (counter == registers[10]) ;

  //axi read
  logic [31:0] axi_read_addr, axi_read_data ;
  logic axi_read_valid, axi_read_req, axi_read_ready ;

  //axi fsm write
  logic [31:0] fsm_adress, fsm_data ;
  logic fsm_req, fsm_ack ; // TODO:VALID is useless here

  //COMPUTE 
  logic compute_done, alert ;
  typedef enum int { COMPUTE_IDLE, GET, RECEIVE } read_ctrl_t;
  read_ctrl_t compute_read ;
  logic [31:0] result ;
  always_comb begin
    alert = 'b0;
  end

  logic [1:0] min_index;
  logic [7:0] v0, v1, v2, v3;

  always_ff @( posedge clk) begin 
    if(seq_port.rst) begin
      state <= IDLE;
      registers_14 <= '0 ;
    end else begin
      case (state)
        IDLE: begin
         if(pwr_launch) begin
           state <= WAIT;
            $display("PWR_CTRL : WAIT state");
         end
         
        end
        WAIT: begin
          if(counter_done) begin
            state <= FETCH;
            $display("PWR_CTRL : FETCH state");
          end
        end
        FETCH: begin
          if(fetch_counter == 32'h1) begin
            state <= COMPUTE;
            $display("PWR_CTRL : COMPUTE state");
          end
        end
        COMPUTE: begin
          if(compute_done) begin
            $display("PWR_CTRL : COMPUTE done, result = %h", result);
            v0 = result[7:0];
            v1 = result[15:8];
            v2 = result[23:16];
            v3 = result[31:24];

            // Equality case: if all values are equal, set min_index to 0
            if ((v0 == v1) && (v1 == v2) && (v2 == v3)) begin
              min_index = 2'd3;
            end else if (v0 <= v1 && v0 <= v2 && v0 <= v3) begin
              min_index = 2'd0;
            end else if (v1 <= v0 && v1 <= v2 && v1 <= v3) begin
              min_index = 2'd1;
            end else if (v2 <= v0 && v2 <= v1 && v2 <= v3) begin
              min_index = 2'd2;
            end else begin
              min_index = 2'd3;
            end
            $display("PWR_CTRL : min_index = %d", min_index);
            if(min_index != 0) begin
              state <= IDLE ;
              registers_14 <= 32'h00_00_00_01 ; 
              $display("PWR_CTRL : triger WAKEUP, going IDLE");
            end else begin
              state <= WAIT ; 
              $display("PWR_CTRL : WAIT state");
            end
          end
        end
        default: begin
          state <= IDLE;
          $display("PWR_CTRL : IDLE state, waiting for launch signal");
        end
      endcase
    end
  end

  always_ff @( posedge clk ) begin 
    if(seq_port.rst) begin
      fetch_state <= FETCH_IDLE;
      compute_read <= COMPUTE_IDLE;
      fetch_data <= 32'h0;
      result <= 32'h0;
      fetch_counter <= 32'h0;
      compute_done <= 0;
    end else begin
      case (state)
        FETCH : begin
          case (fetch_state)
            FETCH_IDLE: begin
              if(axi_read_ready && fetch_counter != 32'h1) begin
                fetch_state <= GET_SEND;
              end
            end
            GET_SEND: begin
              fetch_state <= GET_RECEIVE;
            end
            GET_RECEIVE : begin
              if(axi_read_valid) begin
                fetch_state <= WRITE;
                fetch_data <= axi_read_data;
              end
            end
            WRITE: begin
              if(fsm_ack) begin
                fetch_state <= FETCH_IDLE;
                fetch_counter <= fetch_counter + 32'h1;
              end
            end
            default: begin
              fetch_state <= FETCH_IDLE;
            end
          endcase
        end 

        COMPUTE : begin
          case (compute_read)
            COMPUTE_IDLE: begin
              if(axi_read_ready && !compute_done) begin
                compute_read <= GET;
              end
            end
            GET: begin
              compute_read <= RECEIVE;
            end
            RECEIVE: begin
              if(axi_read_valid) begin
                compute_read <= COMPUTE_IDLE;
                result <= axi_read_data;
                compute_done <= 1;
              end
            end
            default: begin
              compute_read <= GET;
            end
          endcase
        end

        default: begin
          fetch_state <= FETCH_IDLE;
          compute_read <= GET;
          compute_done <= 0;
          fetch_counter <= 32'h0;
        end
      endcase
    end
  end

  always_comb begin 
    case (state)
      FETCH : begin
        case (fetch_state)
          FETCH_IDLE: begin
            axi_read_req <= 0;
            axi_read_addr <= 32'h0;
            fsm_req <= 0;
            fsm_adress <= 32'h0;
            fsm_data <= 32'h0;
          end
          GET_SEND : begin
            axi_read_req <= 1;
            axi_read_addr <= registers[fetch_counter+11] ; 
            fsm_req <= 0;
            fsm_adress <= 32'h0;
            fsm_data <= 32'h0;
          end
          GET_RECEIVE : begin
            axi_read_req <= 0;
            axi_read_addr <= 32'b0;
            fsm_req <= 0;
            fsm_adress <= 32'h0;
            fsm_data <= 32'h0;
          end
          WRITE : begin
            axi_read_req <= 0;
            axi_read_addr <= 32'h0;
            fsm_req <= 1;
            fsm_adress <= registers[fetch_counter+6] ;
            fsm_data <= fetch_data;
          end
          default: begin
            axi_read_req <= 0;
            axi_read_addr <= 32'h0;
            fsm_req <= 0;
            fsm_adress <= 32'h0;
            fsm_data <= 32'h0;
          end
        endcase
      end

      COMPUTE : begin
        case (compute_read)
          COMPUTE_IDLE: begin
            axi_read_req <= 0;
            axi_read_addr <= 32'h0;
            fsm_req <= 0;
            fsm_adress <= 32'h0;
            fsm_data <= 32'h0;
          end
          GET: begin
            axi_read_req <= 1;
            axi_read_addr <= MMAP_ACCEL.start + 32'h88 ;
            fsm_req <= 0;
            fsm_adress <= 32'h0;
            fsm_data <= 32'h0;
          end
          RECEIVE: begin
            axi_read_req <= 0;
            axi_read_addr <= 32'h0;
            fsm_req <= 0;
            fsm_adress <= 32'h0;
            fsm_data <= 32'h0;
          end
          default: begin
            axi_read_req <= 0;
            axi_read_addr <= 32'h0;
            fsm_req <= 0;
            fsm_adress <= 32'h0;
            fsm_data <= 32'h0;
          end
        endcase
      end

      default: begin
        axi_read_req <= 0;
        axi_read_addr <= 32'h0;
        fsm_req <= 0;
        fsm_adress <= 32'h0;
        fsm_data <= 32'h0;
      end
    endcase
    
  end

  // wait counter
  always_ff @( posedge clk) begin
    if(seq_port.rst) begin
      counter <= 32'h0;
    end else begin
      if(state == WAIT) begin
        if(counter_done) begin
          counter <= 32'h0;
        end else begin
          counter <= counter + 32'h1;
        end
      end
    end
  end

  //maestro management
  state_t old_state ; 
  logic [3:0] pwr_counter ;
  logic pwr_ctrl_en ; 

  logic [31:0] maestro_address, maestro_data ;
  logic maestro_req, maestro_ack ;
  logic [7:0] transition_number, pwr_transition ; 

  // adress lookup table 
  logic [31:0] maestro_adress_lut [0:15] ;
  assign maestro_adress_lut[0] = MMAP_SYSCFG.start + (4 * (5) + 1) * 4   ; //CPU
  assign maestro_adress_lut[1] = MMAP_SYSCFG.start + (4 * (6) + 1) * 4   ; //LPRAM

  always_comb begin 
      case (old_state)
          IDLE: transition_number <= 8'h0;
          WAIT: transition_number <= 8'h1;
          FETCH: transition_number <= 8'h2;
          COMPUTE: if(state == WAIT) begin transition_number <= 8'h3; end else begin transition_number <= 8'h4; end
          default: transition_number <= 8'h0;
      endcase
  end

  always_ff @( posedge clk) begin
    if(seq_port.rst) begin
      old_state <= IDLE;      
      pwr_ctrl_en <= 0;
      pwr_counter <= 32'h0;
      pwr_transition <= 8'h0;
      maestro_address <= 32'h0;
      maestro_data <= 32'h0;
      maestro_req <= 0;
    end else begin 
      old_state <= state;
      if((old_state != state) && (registers[1 + transition_number][0+:6] != 6'b11_1111)) begin
        pwr_ctrl_en <= 1;
        case (old_state)
          IDLE: pwr_transition <= 8'h0;
          WAIT: pwr_transition <= 8'h1;
          FETCH: pwr_transition <= 8'h2;
          COMPUTE: if(state == WAIT) begin pwr_transition <= 8'h3; end else begin pwr_transition <= 8'h4; end
        endcase
      end else if(pwr_ctrl_en && registers[1 + pwr_transition][6*(pwr_counter+1)+:6] == 6'b11_1111) begin
        pwr_ctrl_en <= 0;
      end

      if(pwr_ctrl_en) begin        
        if (maestro_ack) begin
          pwr_counter <= pwr_counter + 4'b1;
          maestro_req <= 0;
        end else begin
          maestro_address <= maestro_adress_lut[registers[1 + pwr_transition][(6*pwr_counter)+:4]] ; // register adress management
          maestro_data <= {30'b0, registers[1 + pwr_transition][(6*pwr_counter)+4 +:2]} +1  ; // register data management
          maestro_req <= 1;
        end
      end else begin
        maestro_req <= 0;
        pwr_counter <= 32'h0;
      end
    end
  end

  axi_write write_port(
    .seq_port(seq_port),
    .axi_master(axi_master),
    .maestro_adress_i(maestro_address),
    .maestro_data_i(maestro_data),
    .maestro_req_i(maestro_req),
    .maestro_ack_o(maestro_ack),
    .fsm_adress_i(fsm_adress),
    .fsm_data_i(fsm_data),
    .fsm_req_i(fsm_req),
    .fsm_ack_o(fsm_ack),
    .adress_i(axi_read_addr),
    .req_i(axi_read_req),
    .ready_o(axi_read_ready),
    .data_o(axi_read_data),
    .valid_o(axi_read_valid)
  ) ;

endmodule