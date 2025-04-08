`include "macros.svh"

module chip_control #(
    `ADAM_CFG_PARAMS
  )(
    ADAM_SEQ.Slave seq_port,
    AXI_LITE.Slave axi_port
    `ifdef SYNTHESIS 
    ,chip_ports.Master chip_port
    `endif 
  ) ;

    // clock management utility 
    logic clk, chip_clk;
    assign clk = seq_port.clk;
    assign chip_clk = seq_port.clk;
    logic rst_n ;
    assign rst_n = ~seq_port.rst;
    // TODO signal and clock shift as in banzAI

    // Chip signals 
    logic CBL0, CBLEN0, CSL0, CWL0 ; 
    logic [1:0] instructions_in;
    logic [4:0] adr_full_col_in, adr_full_row_in;
    logic DATA_out [3:0];

    // control signals 
    logic read_mem, read_regs, read_result, write_mem, write_regs;
    logic [31:0] read_addr, write_addr;
    logic [31:0] write_data;
    logic [7:0] read_data;

    // registers 
    typedef struct packed {
    logic [7:0] empty_2 ;
    logic [7:0] empty_1 ;
    logic [7:0] empty_0 ;
    logic [7:0] obs_3 ;
    logic [7:0] obs_2 ; 
    logic [7:0] obs_1 ;
    logic [7:0] obs_0 ;
    logic [7:0] set_reset_mode ; // Now LSB (bit [7:0])
    } control_registers_t;
    control_registers_t control_registers; // written by AXI

    typedef struct packed {
        logic [7:0] inference_result_4 ; 
        logic [7:0] inference_result_3 ;
        logic [7:0] inference_result_2 ;
        logic [7:0] inference_result_1 ;
    } status_registers_t;
    status_registers_t status_registers; // written by FSM

    // Memory map : 
    // 0x0000 - 0x007F : RRAM arrays 
    // 0x0080 - 0x0087 : Control registers
    // 0x0088 - 0x008B : Status registers
    // => 8 bit memory adresses, word size 8 bit

    // signals 
    logic ready, fsm_ready, fsm_done; 
    assign ready = ~(read_mem || read_regs || read_result || write_mem || write_regs || ~fsm_ready) ;

    typedef enum int {
        IDLE, 
        WRITE_SETUP, WRITE_ASSERT_CWL, WRITE_BUFFER, 
        READ_SETUP, READ_ASSERT_SL_WL, READ_SL_FALL, READ_WL_BUFFER, READ_WL_FALL, READ_INFERENCE, READ_OUT,
        INF_SETUP, INF_ASSERT_SL_WL, INF_SL_FALL, INF_WL_BUFFER, INF_WL_FALL, INF_INFERENCE, INF_READ_OUT
    } state_t;
    state_t state;

    //stuff not yet used 
    logic [15:0] read_counter ;
    logic read_pulse_counter ;
    logic [10:0]  read_output_count ; 
    logic [15:0] write_counter, write_pulse_counter, write_precharge_counter ;
        
    // AXI interface process 
    always_ff @(posedge clk) begin
    if (!rst_n) begin
      control_registers <= 'b0;

      axi_port.r_valid <= 1'b0;
      axi_port.b_valid <= 1'b0;
      axi_port.ar_ready <= 1'b0;
      axi_port.aw_ready <= 1'b0;
      axi_port.w_ready <= 1'b0;
      axi_port.b_resp <= 2'b00;
      axi_port.r_resp <= 2'b00;
      axi_port.r_data <= 'b0;

      read_mem <= 1'b0;
      read_regs <= 1'b0;
      read_result <= 1'b0;
      write_mem <= 1'b0;
      write_regs <= 1'b0;
      read_data <= 'b0;
      read_addr <= 'b0;
      write_data <= 'b0;
      write_addr <= 'b0;
    end
    else begin
      // read management
      if(axi_port.ar_valid && ready) begin
        if(axi_port.ar_addr < 32'h80) begin
          read_mem <= 1'b1;
        end else if(axi_port.ar_addr == 32'h88) begin
          read_result <= 1'b1;
        end else begin
          read_regs <= 1'b1;
        end 
        axi_port.ar_ready <= 1'b1;
        read_addr <= axi_port.ar_addr[0+:8];
      end else begin
        axi_port.ar_ready <= 1'b0;
      end

      // read response management
      if(axi_port.r_ready) begin
        if(read_regs) begin
          if(read_addr < 32'h0088) begin
            axi_port.r_data <= control_registers[ ({3'b000, read_addr[0+:3]} <<3 ) +:8];
          end else begin
            axi_port.r_data <= status_registers[ (read_addr[0+:2] << 3) +:8];
          end
          axi_port.r_valid <= 1'b1;
          axi_port.r_resp <= 2'b00;
          read_regs <= 1'b0;
        end else if(read_result && state==IDLE && read_output_count > 0) begin
          axi_port.r_data <= status_registers;
          axi_port.r_valid <= 1'b1;
          axi_port.r_resp <= 2'b00;
          read_result <= 1'b0;
        end else if(read_mem && read_output_count >= 12) begin
          axi_port.r_data <= read_data;
          axi_port.r_valid <= 1'b1;
          axi_port.r_resp <= 2'b00;
          read_mem <= 1'b0;
        end else begin
          axi_port.r_valid <= 1'b0;
        end
      end

      // write management
      if(axi_port.aw_valid && axi_port.w_valid && ready) begin
        if(axi_port.aw_addr < 31'h80) begin
          write_mem <= 1'b1;
        end else begin
          write_regs <= 1'b1;
        end
        axi_port.aw_ready <= 1'b1;
        axi_port.w_ready <= 1'b1;
        write_addr <= axi_port.aw_addr[0+:8];
        write_data <= axi_port.w_data;
      end else begin
        axi_port.aw_ready <= 1'b0;
        axi_port.w_ready <= 1'b0;
      end

      // write response management
      if(write_regs && axi_port.b_ready) begin
        if (write_addr !=0) begin
          control_registers[({3'b000, write_addr[0+:4]} << 3) +: 8] <= write_data[0+:8];
        end
        axi_port.b_valid <= 1'b1;
        axi_port.b_resp <= 2'b00;
        write_regs <= 1'b0;
      end else if(write_mem && write_counter == 7) begin
        axi_port.b_resp <= 2'b00;
        axi_port.b_valid <= 1'b1;
        write_mem <= 1'b0;
      end else begin
        axi_port.b_valid <= 1'b0;
      end
      end
    end

    state_t next_state ;
    assign fsm_ready = (state == IDLE) ? 1'b1 : 1'b0;
    logic [1:0] inference_read_count ; 

    always_ff @(posedge clk) begin
        if(!rst_n) begin
            write_counter <= 0;
            read_output_count <= 0;
            inference_read_count <= 0;
            status_registers <= 'b0;
        end else begin
          case(state)
            IDLE : begin
              write_counter <= 0;
              read_output_count <= 0;
              inference_read_count <= 0;
            end
            WRITE_BUFFER : begin
              write_counter <= write_counter + 1;
            end
            READ_INFERENCE : begin
              read_output_count <= read_output_count + 1;
            end
            READ_OUT : begin
              read_output_count <= read_output_count + 1;
              read_data <= (read_data<<1) | 1'(DATA_out[read_addr[6:5]]);
            end

            INF_WL_FALL : begin 
              inference_read_count <= inference_read_count + 1;
            end

            INF_INFERENCE : begin
              read_output_count <= read_output_count + 1;
            end
            INF_READ_OUT : begin
              read_output_count <= read_output_count + 1;
              status_registers.inference_result_1 <= (status_registers.inference_result_1<<1) | 1'(DATA_out[0]);
              status_registers.inference_result_2 <= (status_registers.inference_result_2<<1) | 1'(DATA_out[1]);
              status_registers.inference_result_3 <= (status_registers.inference_result_3<<1) | 1'(DATA_out[2]);
              status_registers.inference_result_4 <= (status_registers.inference_result_4<<1) | 1'(DATA_out[3]); 
            end

          endcase
        end
      end

    always_ff @(posedge clk) begin
        if(!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin 
      case (state) 
        IDLE : begin 
          if( read_output_count >= 12) begin
            next_state <= IDLE;
          end else if(write_mem) begin
            next_state <= WRITE_SETUP;                    
          end else if(read_mem) begin
            next_state <= READ_SETUP;
          end else if(read_result) begin
            next_state <= INF_SETUP;
          end else begin
            next_state <= IDLE;
          end
        end 
        WRITE_SETUP : begin
          next_state <= WRITE_ASSERT_CWL;
        end
        WRITE_ASSERT_CWL : begin
          next_state <= WRITE_BUFFER;
        end
        WRITE_BUFFER : begin
          if(write_counter == 7) begin
            next_state <= IDLE;
          end else begin
            next_state <= WRITE_SETUP ; 
          end
        end

        READ_SETUP : begin
          next_state <= READ_ASSERT_SL_WL;
        end
        READ_ASSERT_SL_WL : begin
          next_state <= READ_SL_FALL;
        end
        READ_SL_FALL : begin
          next_state <= READ_WL_BUFFER;
        end
        READ_WL_BUFFER : begin
          next_state <= READ_WL_FALL;
        end
        READ_WL_FALL : begin
          next_state <= READ_INFERENCE;
        end
        READ_INFERENCE : begin
          next_state <= READ_OUT;
        end
        READ_OUT : begin
          if(read_output_count >= 11) begin
            next_state <= IDLE;
          end else begin
            next_state <= READ_OUT;
          end
        end

        INF_SETUP : begin
          next_state <= INF_ASSERT_SL_WL;
        end
        INF_ASSERT_SL_WL : begin
          next_state <= INF_SL_FALL;
        end
        INF_SL_FALL : begin
          next_state <= INF_WL_BUFFER;
        end
        INF_WL_BUFFER : begin
          next_state <= INF_WL_FALL;
        end
        INF_WL_FALL : begin
          if(inference_read_count == 3) begin
            next_state <= INF_INFERENCE;
          end else begin
            next_state <= INF_SETUP ;
          end
        end
        INF_INFERENCE : begin
          next_state <= INF_READ_OUT;
        end
        INF_READ_OUT : begin
          if(read_output_count >= 11) begin
            next_state <= IDLE;
          end else begin
            next_state <= INF_READ_OUT;
          end
        end
      endcase
    end

    always_comb begin
      case (state)
        IDLE : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b00;
          adr_full_col_in <= 5'b0;
          adr_full_row_in <= 5'b0;
        end
        WRITE_SETUP : begin
          CBL0 <= write_data[write_counter[2:0]];
          CBLEN0 <= 1'b1;
          CSL0 <= control_registers.set_reset_mode[0];
          CWL0 <= 1'b0;
          instructions_in <= 2'b11;
          adr_full_col_in <= {write_addr[4:3], write_addr[0+:3]};
          adr_full_row_in <= {write_addr[6:5], write_counter[0+:3]};
        end
        WRITE_ASSERT_CWL : begin
          CBL0 <= write_data[write_counter[2:0]];
          CBLEN0 <= 1'b1;
          CSL0 <= control_registers.set_reset_mode[0];
          CWL0 <= 1'b1;
          instructions_in <= 2'b11;
          adr_full_col_in <= {write_addr[4:3], write_addr[0+:3]};
          adr_full_row_in <= {write_addr[6:5], write_counter[0+:3]};
        end
        WRITE_BUFFER : begin
          CBL0 <= write_data[write_counter[2:0]];
          CBLEN0 <= 1'b1;
          CSL0 <= control_registers.set_reset_mode[0];
          CWL0 <= 1'b0;
          instructions_in <= 2'b11;
          adr_full_col_in <= {write_addr[4:3], write_addr[0+:3]};
          adr_full_row_in <= {write_addr[6:5], write_counter[0+:3]};
        end

        READ_SETUP : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b10;
          adr_full_col_in <= {read_addr[4:3], read_addr[0+:3]};
          adr_full_row_in <= {read_addr[6:5], 3'b0};
        end
        READ_ASSERT_SL_WL : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b1;
          CWL0 <= 1'b1;
          instructions_in <= 2'b10;
          adr_full_col_in <= {read_addr[4:3], read_addr[0+:3]};
          adr_full_row_in <= {read_addr[6:5], 3'b0};
        end
        READ_SL_FALL : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b1;
          instructions_in <= 2'b10;
          adr_full_col_in <= {read_addr[4:3], read_addr[0+:3]};
          adr_full_row_in <= {read_addr[6:5], 3'b0};
        end
        READ_WL_BUFFER : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b1;
          instructions_in <= 2'b10;
          adr_full_col_in <= {read_addr[4:3], read_addr[0+:3]};
          adr_full_row_in <= {read_addr[6:5], 3'b0};
        end
        READ_WL_FALL : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b01; 
          adr_full_col_in <= {read_addr[4:3], read_addr[0+:3]};
          adr_full_row_in <= {read_addr[6:5], 3'b0};
        end
        READ_INFERENCE : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b01;
          adr_full_col_in <= '0;
          adr_full_row_in <= '0;
        end
        READ_OUT : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b01;
          adr_full_col_in <= '0;
          adr_full_row_in <= '0;
        end

        INF_SETUP : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b00;
          case (inference_read_count)
            2'b00 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_0[0+:3]};
            end
            2'b01 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_1[0+:3]};
            end
            2'b10 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_2[0+:3]};
            end
            2'b11 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_3[0+:3]};
            end
            default: begin
              adr_full_col_in <= 5'b0 ; 
            end
          endcase
          adr_full_row_in <= {2'b0, 3'b0};
        end
        INF_ASSERT_SL_WL : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b1;
          CWL0 <= 1'b1;
          instructions_in <= 2'b00;
          case (inference_read_count)
            2'b00 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_0[0+:3]};
            end
            2'b01 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_1[0+:3]};
            end
            2'b10 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_2[0+:3]};
            end
            2'b11 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_3[0+:3]};
            end
            default: begin
              adr_full_col_in <= 5'b0 ; 
            end
          endcase
          adr_full_row_in <= {2'b0, 3'b0};
        end
        INF_SL_FALL : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b1;
          instructions_in <= 2'b00;
          case (inference_read_count)
            2'b00 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_0[0+:3]};
            end
            2'b01 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_1[0+:3]};
            end
            2'b10 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_2[0+:3]};
            end
            2'b11 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_3[0+:3]};
            end
            default: begin
              adr_full_col_in <= 5'b0 ; 
            end
          endcase
          adr_full_row_in <= {2'b0, 3'b0};
        end
        INF_WL_BUFFER : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b1;
          instructions_in <= 2'b00;
          case (inference_read_count)
            2'b00 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_0[0+:3]};
            end
            2'b01 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_1[0+:3]};
            end
            2'b10 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_2[0+:3]};
            end
            2'b11 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_3[0+:3]};
            end
            default: begin
              adr_full_col_in <= 5'b0 ; 
            end
          endcase
          adr_full_row_in <= {2'b0, 3'b0};
        end
        INF_WL_FALL : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b01; 
          case (inference_read_count)
            2'b00 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_0[0+:3]};
            end
            2'b01 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_1[0+:3]};
            end
            2'b10 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_2[0+:3]};
            end
            2'b11 : begin
              adr_full_col_in <= {inference_read_count, control_registers.obs_3[0+:3]};
            end
            default: begin
              adr_full_col_in <= 5'b0 ; 
            end
          endcase
          adr_full_row_in <= {2'b0, 3'b0};
        end
        INF_INFERENCE : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b01;
          adr_full_col_in <= '0;
          adr_full_row_in <= '0;
        end
        INF_READ_OUT : begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b01;
          adr_full_col_in <= '0;
          adr_full_row_in <= '0;
        end


        default: begin
          CBL0 <= 1'b0;
          CBLEN0 <= 1'b0;
          CSL0 <= 1'b0;
          CWL0 <= 1'b0;
          instructions_in <= 2'b00;
          adr_full_col_in <= 5'b0;
          adr_full_row_in <= 5'b0;
        end
      endcase
    end

    // Chip instanciation
    `ifndef SYNTHESIS
        Bayesian_log2 chip (
            .clk(chip_clk), 
            .CBL0(CBL0),
            .CBLEN0(CBLEN0),
            .CSL0(CSL0),
            .CWL0(CWL0),	
            .instructions_in(instructions_in),
            .adr_full_col_in(adr_full_col_in),
            .adr_full_row_in(adr_full_row_in),
            .DATA_out(DATA_out)
        )
    `else 
        assign chip_port.clk = clk;
        assign chip_port.CBL0 = CBL0;
        assign chip_port.CBLEN0 = CBLEN0;
        assign chip_port.CSL0 = CSL0;
        assign chip_port.CWL0 = CWL0;
        assign chip_port.instructions_in = instructions_in;
        assign chip_port.adr_full_col_in = adr_full_col_in;
        assign chip_port.adr_full_row_in = adr_full_row_in;
        assign DATA_out = chip_port.DATA_out ;
    `endif ;

endmodule