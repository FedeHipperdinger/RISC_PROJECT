// -----------------------------------------------------------------------------
// Description:
// ------------
//
//Control unit for one cycle RISC
//
// -----------------------------------------------------------------------------


module control_unit_mod ( 
  // ----------------------------------
  // Data Inputs
  // ----------------------------------
  input  wire      	 	   clk,
  input  wire      	 	   rst_n,
  input  wire      	 	   [1:0]T_i,  
  input  wire      	 	   [2:0]OPC_i,  
  input  wire      	 	   [33:0]reg_i,  //Rc
  input  wire      	 	   C_flag_i,  
  input  wire      	 	   V_flag_i, 
  input  wire      	 	   [1:0]addr_bits_i, //two least significant bits of the data memory input address  
  input  wire      	 	   [1:0]pc_bits_i,   //two least significant bits of the program counter input address  


  // ----------------------------------
  // Outputs
  // ----------------------------------
  output reg      	   	 [1:0]pc_sel_o,
  output reg      	   	 data_mem_w_ctrl_o,
  output reg      	   	 data_mem_w_sel_o,
  output reg      	   	 [2:0]alu_ctrl_o,
  output reg      	   	 alu_srcA_sel_o,
  output reg      	   	 [1:0]alu_srcB_sel_o,
  output reg      	   	 reg_file_w_ctrl_o,
  output reg      	   	 [1:0]reg_file_w_sel_o,
  output reg      	   	 extend_ctrl_o,
  output reg      	   	 [1:0]status_flags_o,
  output reg      	   	 [2:0]exception_flags_o // index 0: Invalid instruction, index 1: Invalid memory address, index 2: Invalid program counter address

);

  
  // Instruction types (T)
  localparam MOVEMENT 		    = 2'b00;
  localparam LOGIC 			      = 2'b10;
  localparam ARITHMETIC 	    = 2'b11;
  localparam FLOW_CONTROL 	  = 2'b01;
  
  //Movement Instructions
  localparam LOAD 		        = 3'b000;
  localparam STORE 			      = 3'b010;
  localparam LOADI 	          = 3'b001;
  localparam STOREI 	        = 3'b011;
  localparam MOV 	            = 3'b100;

  //Logic Instructions
  localparam OR 		          = 3'b000;
  localparam INV 			        = 3'b001;
  localparam AND 	            = 3'b010;

  //Arithmetic Instructions
  localparam ADD 		          = 3'b000;
  localparam SUB 			        = 3'b001;
  localparam ADDC 	          = 3'b010;
  localparam SUBC 	          = 3'b011;

  //Flow control Instructions
  localparam JUMP 		        = 3'b000;
  localparam BZ 			        = 3'b001;
  localparam BNZ 	            = 3'b010;
  localparam BC 	            = 3'b011;
  localparam BV 	            = 3'b100;
  localparam JAL 	            = 3'b101;
  localparam JRAL 	          = 3'b110;
  localparam RET 	            = 3'b111;

  //Extra signal
  localparam STOP = 3'b111; // Belongs to movement instructions

  reg exception_0;
  reg exception_1;
  reg exception_2;
  reg halt;
  
  assign exception_flags_o[0] = exception_0; // Invalid instruction
  assign exception_flags_o[1] = exception_1; // Invalid memory address
  assign exception_flags_o[2] = exception_2; // Invalid program counter address
  assign halt = (exception_flags_o != 3'd0 || (T_i == MOVEMENT & OPC_i == STOP)); // Halt if any exception or STOP instruction is detected
  
  always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
      data_mem_w_ctrl_o <= 1'b0;   // disable write data mem  by default
      data_mem_w_sel_o  <= 1'b0;   // data_mem_w_sel_o = Rc.data by default
      status_flags_o    <= 2'b00;  // V and C zero by default
      reg_file_w_ctrl_o <= 1'b0;   // disable write reg file by default
      reg_file_w_sel_o  <= 2'd1;   // Ra.data = alu_res by default
      pc_sel_o          <= 2'b00;  // PC + 4 by default
      alu_ctrl_o        <= 3'd0;   // alu addition by default
      alu_srcA_sel_o    <= 1'b0;   // srcA = Rb.data by default
      alu_srcB_sel_o    <= 2'd0;   // srcB = Rc.data by default
      extend_ctrl_o     <= 1'b1;   // ext_o = Sign_ext(imm,Rc) by default
      exception_0 		  <= 1'b0; 
      exception_1 		  <= 1'b0;
      exception_2 		  <= 1'b0;
    end
    else begin
      if(T_i == MOVEMENT && (OPC_i == LOAD || OPC_i == STORE || OPC_i == STOREI)) begin
        if (addr_bits_i != 2'b00) begin
          exception_1 <=  1'b1; // Invalid memory address
        end
      end

    end
  end

  
  always @(*) begin
    data_mem_w_ctrl_o = 1'b0;   // disable write data mem
    data_mem_w_sel_o  = 1'b0;   // data_mem_w_sel_o = Rc.data by default
    status_flags_o    = 2'b00;  // V and C zero by default
    reg_file_w_ctrl_o = 1'b0;   // disable write reg file
    reg_file_w_sel_o  = 2'd1;   // Ra.data = alu_res by default
    alu_ctrl_o        = 3'd0;   // alu addition by default
    alu_srcA_sel_o    = 1'b0;   // srcA = Rb.data by default
    alu_srcB_sel_o    = 2'd0;   // srcB = Rc.data by default
    extend_ctrl_o     = 1'b1;   // ext_o = Sign_ext(imm,Rc) by default
    
    if(halt == 1'b1) begin // Hold exception flags if halt
      exception_0 		= exception_0; 
      exception_1 		= exception_1;
      exception_2 		= exception_2;
      pc_sel_o        = 2'd3;  // PC = PC
    end
    else begin
      exception_2 = (pc_bits_i[0] | pc_bits_i[1]);     // Invalid program counter address
      exception_0 = 1'b0;
      pc_sel_o          = 2'b00;  // PC + 4 by default

      if(exception_2 == 1'b0) begin
        case(T_i)
          MOVEMENT: begin
            case(OPC_i)
              LOAD: begin
                reg_file_w_ctrl_o = 1'b1;  // enable write reg file
                alu_srcA_sel_o    = 1'b0;  // srcA = Rb.data
                alu_srcB_sel_o    = 2'd1;  // srcB = ext_o
                extend_ctrl_o     = 1'b1;
              end
              STORE: begin
                data_mem_w_ctrl_o = 1'b1;  // enable write data mem
                data_mem_w_sel_o  = 1'b0;  // data_mem_w_sel_o = Rc.data
                alu_srcA_sel_o    = 1'b0;  // srcA = Rb.data
                alu_srcB_sel_o    = 2'd1;  // srcB = ext_o
                extend_ctrl_o     = 1'b0; // ext_o = Sign_ext(imm)
              end
              LOADI: begin
                reg_file_w_ctrl_o = 1'b1;   // enable write reg file
                reg_file_w_sel_o  = 2'd2;   // Ra.data = Sign_ext(imm,Rc)
                alu_srcA_sel_o    = 1'b1;   // srcA = 0
                alu_srcB_sel_o    = 2'd1;   // srcB = ext_o
                extend_ctrl_o     = 1'b1;   // ext_o = Sign_ext(imm,Rc)
              end
              STOREI: begin
                data_mem_w_ctrl_o = 1'b1;  // enable write data mem
                data_mem_w_sel_o  = 1'b1;  // data_mem_w_sel_o = ext_o
                alu_srcA_sel_o    = 1'b0;  // srcA = Rb.data
                alu_srcB_sel_o    = 2'd2;  // srcB = 0
                extend_ctrl_o     = 1'b1;
              end
              MOV: begin
                alu_ctrl_o        = 3'd7;  // pass the value of Rb to Ra
                reg_file_w_ctrl_o = 1'b1;  // enable write reg file
                reg_file_w_sel_o  = 2'd2;  // Ra.data = Rb.data
                alu_srcA_sel_o    = 1'b0;  // srcA = Rb.data
                alu_srcB_sel_o    = 2'd2;  // srcB = 0
                status_flags_o    = {V_flag_i, C_flag_i};// V and C
              end
              STOP: begin
                exception_0 = 1'b0; // halt without exception
              end
              default:
                exception_0 = 1'b1; // Invalid instruction

            endcase
          end
          LOGIC: begin
            //status_flags_o    = 2'b00; // V and C
            reg_file_w_ctrl_o = 1'b1;  // enable write reg file
            reg_file_w_sel_o  = 2'd2;

            case(OPC_i)
              OR: begin
                alu_ctrl_o  = 3'd4;
              end
              INV: begin
                alu_ctrl_o  = 3'd5;
              end
              AND: begin
                alu_ctrl_o  = 3'd6;
              end
              default:
                exception_0 = 1'b1; // Invalid instruction
              
            endcase
          end
          ARITHMETIC: begin
            reg_file_w_ctrl_o = 1'b1;  // enable write reg file
            reg_file_w_sel_o  = 2'd2;  // Ra.data = alu_result_o
            alu_srcA_sel_o    = 1'b0;  // srcA = Rb.data
            alu_srcB_sel_o    = 2'd0;  // srcB = Rc.data
            status_flags_o    = {V_flag_i, C_flag_i}; // V and C

            case(OPC_i) 
              ADD: begin
                alu_ctrl_o        = 3'd0; // already setted by default
              end

              SUB: begin
                alu_ctrl_o        = 3'd1;
              end

              ADDC: begin
                alu_ctrl_o        = 3'd2;
              end

              SUBC: begin
                alu_ctrl_o        = 3'd3;
              end
              default:
                exception_0 = 1'b1; // Invalid instruction

            endcase
            
            
          end
          FLOW_CONTROL: begin
            case(OPC_i)
              JUMP: begin
                pc_sel_o          = 2'd1; // PC' = PC + alu_result_o
                alu_srcA_sel_o    = 1'b0; // srcA = Rb.data
                alu_srcB_sel_o    = 2'd2; // srcB = 0
                                          // alu_result_o = Rb.data
              end
              BZ: begin
                if (~|reg_i[31:0]) begin         // if(reg_i[31:0] == 32'd0)
                  pc_sel_o          = 2'd1; // PC' = PC + alu_result_o
                  alu_srcA_sel_o    = 1'b0; // srcA = Rb.data
                  alu_srcB_sel_o    = 2'd1; // srcB = ext_o
                  extend_ctrl_o     = 1'b0; // ext_o = Sign_ext(imm)
                end
              end
              BNZ: begin
                if (|reg_i[31:0]) begin          // if(reg_i[31:0] != 32'd0)
                  pc_sel_o          = 2'd1; // PC' = PC + alu_result_o
                  alu_srcA_sel_o    = 1'b0; // srcA = Rb.data
                  alu_srcB_sel_o    = 2'd1; // srcB = ext_o
                  extend_ctrl_o     = 1'b0; // ext_o = Sign_ext(imm)
                end
              end
              BC: begin
                if(reg_i[32] == 1'b1) begin
                  pc_sel_o          = 2'd1; // PC' = PC + alu_result_o
                  alu_srcA_sel_o    = 1'b0; // srcA = Rb.data
                  alu_srcB_sel_o    = 2'd1; // srcB = ext_o
                  extend_ctrl_o     = 1'b0; // ext_o = Sign_ext(imm)
                end
              end
              BV: begin
                if(reg_i[33] == 1'b1) begin
                  pc_sel_o          = 2'd1; // PC' = PC + alu_result_o
                  alu_srcA_sel_o    = 1'b0; // srcA = Rb.data
                  alu_srcB_sel_o    = 2'd1; // srcB = ext_o
                  extend_ctrl_o     = 1'b0; // ext_o = Sign_ext(imm)
                end
              end
              JAL: begin
                reg_file_w_ctrl_o = 1'b1;  // enable write reg file
                reg_file_w_sel_o  = 2'd0;  // Ra.data = PC + 4
                pc_sel_o          = 2'd1; // PC' = PC + alu_result_o
                alu_srcA_sel_o    = 1'b1; // srcA = 0
                alu_srcB_sel_o    = 2'd1; // srcB = ext_o
                extend_ctrl_o     = 1'b1; // ext_o = Sign_ext(imm,Rc)
              end
              JRAL: begin
                reg_file_w_ctrl_o = 1'b1;  // enable write reg file
                reg_file_w_sel_o  = 2'd0;  // Ra.data = PC + 4
                pc_sel_o          = 2'd1; // PC' = PC + alu_result_o
                alu_srcA_sel_o    = 1'b0; // srcA = Rb.data
                alu_srcB_sel_o    = 2'd1; // srcB = ext_o
                extend_ctrl_o     = 1'b1; // ext_o = Sign_ext(imm,Rc)
              end
              RET: begin
                pc_sel_o          = 2'd2; // PC' = alu_result_o
                alu_srcA_sel_o    = 1'b0; // srcA = Rb.data
                alu_srcB_sel_o    = 2'd2; // srcB = 0
              end
              default:
                exception_0 = 1'b1; // Invalid instruction
            endcase
          end
          default:
            exception_0 = 1'b1; // Invalid instruction  

          
          
          
        endcase
      end
    end
  end

endmodule  