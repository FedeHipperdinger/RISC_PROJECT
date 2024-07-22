// -----------------------------------------------------------------------------
// Description:
// ------------

// -----------------------------------------------------------------------------


module hazard_unit_mod #(parameter N = 10)( 
  input  wire      	 	clk,
  input  wire      	 	rst_n,
  // ----------------------------------
  // Data Inputs
  // ----------------------------------
  input  wire      	 	[31:0]instr_i,
  input  wire      	  [1:0]pc_selD_i,
  input  wire      	  [1:0]pc_selE_i,


  // ----------------------------------
  // Outputs
  // ----------------------------------
  output reg      	   	 stallF_o,
  output reg      	   	 stallD_o,
  output reg      	   	 flushD_o,
  output reg      	   	 flushE_o,
  output reg      	   	 [1:0]bypA_sel_o,
  output reg      	   	 [1:0]bypB_sel_o,
  output reg      	   	 [1:0]byp_CU_sel_o
);
  // Instruction types (T)
  localparam MOVEMENT 		    = 2'b00;    // Data hazard --> Stall
  localparam LOGIC 			      = 2'b10;    // RAW data hazard --> Bypass
  localparam ARITHMETIC 	    = 2'b11;    // RAW data hazard --> Bypass
  localparam FLOW_CONTROL 	  = 2'b01;    // Control hazard --> Flush/stall

  //Arithmetic Instructions
  localparam NOP              = 3'b111;   // No operation

  //Movement Instructions
  localparam LOAD 		        = 3'b000;  // Stall
  localparam STORE 			      = 3'b010;
  localparam LOADI 	          = 3'b001;
  localparam STOREI 	        = 3'b011;
  localparam MOV 	            = 3'b100;

  //Flow control Instructions
  localparam JUMP 		        = 3'b000;   // Stall
  localparam BZ 			        = 3'b001;   // Flush
  localparam BNZ 	            = 3'b010;   // Flush
  localparam BC 	            = 3'b011;   // Flush
  localparam BV 	            = 3'b100;   // Flush
  localparam JAL 	            = 3'b101;   // Stall
  localparam JRAL 	          = 3'b110;   // Stall
  localparam RET 	            = 3'b111;   // Stall

  ////////////////////
  // Internal wires //
  ////////////////////
  wire [1:0] T;
  wire [2:0] OPC;
  wire [11:0] Imm;
  wire [4:0] Rc;
  wire [4:0] Rb;
  wire [4:0] Ra;

  reg  [1:0] clk_count1;
  reg  [1:0] clk_count2;
  reg  [1:0] clk_count3;
  reg  [4:0] Ra_mem [2:0];
  reg  [1:0] T_mem [2:0];
  reg  Ra_changed_mem [2:0];
  reg  movement_stall_flag;
  reg  flow_stall_flag;
  reg  flow_ctrl_flush_flag;

  // Assignments
  assign T = instr_i[31:30];
  assign OPC = instr_i[29:27];
  assign Imm = instr_i[26:15];
  assign Rc = instr_i[14:10];
  assign Rb = instr_i[9:5];
  assign Ra = instr_i[4:0];

  
  always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
      stallF_o <= 1'b0;
      stallD_o <= 1'b0;
      flushD_o <= 1'b0;
      flushE_o <= 1'b0;
      bypA_sel_o <= 2'b00;
      bypB_sel_o <= 2'b00;
      clk_count1 <= 0;
      clk_count2 <= 0;
      Ra_mem <= {5'd0, 5'd0, 5'd0};
      T_mem <= {MOVEMENT, MOVEMENT, MOVEMENT};  
      Ra_changed_mem <= {1'b0, 1'b0, 1'b0};
      movement_stall_flag <= 1'b0;
      flow_ctrl_flush_flag <= 1'b0;

    end
    else begin
      
      Ra_mem <= {Ra_mem[1], Ra_mem[0], Ra};
      
      if(T == MOVEMENT) begin
        if(OPC == LOADI || OPC == MOV) begin
          Ra_changed_mem <= {Ra_changed_mem[1], Ra_changed_mem[0], 1'b1};
        end
        else begin
          Ra_changed_mem <= {Ra_changed_mem[1], Ra_changed_mem[0], 1'b0};
        end
      end
      else if(T == LOGIC || T == ARITHMETIC) begin
        if(OPC != NOP) begin
          Ra_changed_mem <= {Ra_changed_mem[1], Ra_changed_mem[0], 1'b1};
        end
        else begin
          Ra_changed_mem <= {Ra_changed_mem[1], Ra_changed_mem[0], 1'b0};
        end
      end
      else begin // T == FLOW_CONTROL
        Ra_changed_mem <= {Ra_changed_mem[1], Ra_changed_mem[0], 1'b0};
      end


      if(movement_stall_flag) begin
        clk_count1 <= clk_count1 + 2'd1;
      end
      else begin
        clk_count1 <= 2'b00;
      end

      if(flow_ctrl_flush_flag) begin
        clk_count2 <= clk_count2 + 2'd1;
      end
      else begin
        clk_count2 <= 2'b00;
      end

      if(flow_stall_flag) begin
        clk_count3 <= clk_count3 + 2'd1;
      end
      else begin
        clk_count3 <= 2'b00;
      end


    end
  end


  always @(*) begin
      // Default values
      bypA_sel_o    = 2'b00;
      bypB_sel_o    = 2'b00;
      byp_CU_sel_o  = 2'b00;
      
      if(Ra_changed_mem[0] && Rb == Ra_mem[0]) begin
        bypA_sel_o = 2'd1;
      end
      else if(Ra_changed_mem[1] && Rb == Ra_mem[1]) begin
        bypA_sel_o = 2'd2;
      end

      if(Ra_changed_mem[0] && Rc == Ra_mem[0]) begin
        bypB_sel_o = 2'd1;
      end
      else if(Ra_changed_mem[1] && Rc == Ra_mem[1]) begin
        bypB_sel_o = 2'd2;
      end
      
      if(movement_stall_flag) begin
        case(clk_count1)
          2'b01: begin
            flushD_o = 1'b1; 
            flushE_o = 1'b0;
          end
          2'b10: begin
            flushD_o = 1'b1; 
            flushE_o = 1'b1;
          end
          2'b11: begin
            flushD_o = 1'b0; 
            flushE_o = 1'b0;
            movement_stall_flag = 1'b0;
            stallF_o = 1'b0;
            stallD_o = 1'b0;
          end
        endcase
      end
      else if(flow_stall_flag) begin
        case(clk_count3)
          2'b01: begin
            flushD_o = 1'b0; 
            flushE_o = 1'b0;
          end
          2'b10: begin
            flushD_o = 1'b0; 
            flushE_o = 1'b0;
            flow_stall_flag = 1'b0;
            stallF_o = 1'b0;
            stallD_o = 1'b0;
          end
        endcase
      end


      else begin

        if(clk_count2 == 2'b01) begin
          if(pc_selE_i == 2'b01) begin // condition matched
            flushD_o = 1'b1; // Flush must introduce a bubble in the pipeline
            flushE_o = 1'b1;
            movement_stall_flag = 1'b0; // Reset the stall flag just in case
            Ra_changed_mem <= {1'b0, 1'b0, 1'b0};
          end
          else begin // condition didn't match
            flow_ctrl_flush_flag = 1'b0;
            flushD_o = 1'b0;
            flushE_o = 1'b0;
          end
        end
        else if(clk_count2 == 2'b10) begin // condition didn't match
          flow_ctrl_flush_flag = 1'b0;
          flushD_o = 1'b0;
          flushE_o = 1'b0;
          Ra_changed_mem <= {1'b0, 1'b0, 1'b0};
        end

        
        

        case(T)
          MOVEMENT: begin
            if(OPC == LOAD) begin
              movement_stall_flag = 1'b1;
              stallF_o = 1'b1;
              stallD_o = 1'b1;
            end
          end
          FLOW_CONTROL: begin
            if(OPC == JUMP || OPC == JAL || OPC == JRAL || OPC == RET)begin 
              stallF_o = 1'b0;
              stallD_o = 1'b1; // Only stall Decode stage
              flow_stall_flag = 1'b1;
              flushD_o = 1'b1; 
              flushE_o = 1'b0;
            end
            else begin
              if(Ra_changed_mem[0] && Rc == Ra_mem[0]) begin
                byp_CU_sel_o = 2'd1;
              end
              else if(Ra_changed_mem[1] && Rc == Ra_mem[1]) begin
                byp_CU_sel_o = 2'd2;
              end
              if(pc_selD_i == 2'd1) begin
                flow_ctrl_flush_flag = 1'b1;  
              end
              else begin
                flow_ctrl_flush_flag = 1'b0;
              end
            end
          end
        endcase

      end
  end
  
endmodule  
