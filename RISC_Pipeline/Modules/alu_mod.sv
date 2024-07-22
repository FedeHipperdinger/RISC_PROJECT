// -----------------------------------------------------------------------------
// Description:
// ------------
//
// ALU for one cycle RISC processor
// -----------------------------------------------------------------------------


module alu_mod ( 
  input  wire      	 	clk,
  // ----------------------------------
  // Data Inputs
  // ----------------------------------
  input  wire      	 	[33:0]alu_srcA_i,
  input  wire      	 	[33:0]alu_srcB_i,
  input  wire      	 	 [2:0]alu_ctrl_i,
  

  // ----------------------------------
  // Outputs
  // ----------------------------------
  output reg      	   	 [31:0]alu_result_o,
  output reg      	   	 alu_C_flag_o,
  output reg      	   	 alu_V_flag_o
);

  //Alu modes of operation - Not the same values as the control unit
  localparam ADD  = 3'd0;
  localparam SUB  = 3'd1;
  localparam ADDC = 3'd2;
  localparam SUBC = 3'd3;
  localparam OR   = 3'd4;
  localparam INV  = 3'd5;
  localparam AND  = 3'd6;
  localparam PASS = 3'd7;

  reg [31:0]srcB;
  always @(*) begin //testear
    srcB = (alu_ctrl_i == ADD | alu_ctrl_i == SUB) ? alu_srcB_i[31:0] : alu_srcB_i[31:0] + {31'd0, alu_srcB_i[32]};
    
    case(alu_ctrl_i)
      ADD:
        begin
          {alu_C_flag_o , alu_result_o} = {1'b0,alu_srcA_i[31:0]}+{1'b0,srcB};
          alu_V_flag_o = ((alu_srcA_i[31] == 1'b1) && (alu_srcB_i[31] == 1'b1) && (alu_result_o[31] == 1'b0))
                          || ((alu_srcA_i[31] == 1'b0) && (alu_srcB_i[31] == 1'b0) && (alu_result_o[31] == 1'b1)); 
        end
      SUB:
        begin
          {alu_C_flag_o , alu_result_o} = {1'b0,alu_srcA_i[31:0]}-{1'b0,srcB};
          alu_V_flag_o = (alu_srcA_i[31] == 1'b1) && (alu_srcB_i[31] == 1'b0) && (alu_result_o[31] == 1'b0)
                          || (alu_srcA_i[31] == 1'b0) && (alu_srcB_i[31] == 1'b1) && (alu_result_o[31] == 1'b1); 
        end
      ADDC:
        begin
          {alu_C_flag_o , alu_result_o} = {1'b0,alu_srcA_i[31:0]}+{1'b0,srcB};
          alu_V_flag_o = ((alu_srcA_i[31] == 1'b1) && (alu_srcB_i[31] == 1'b1) && (alu_result_o[31] == 1'b0))
                          || ((alu_srcA_i[31] == 1'b0) && (alu_srcB_i[31] == 1'b0) && (alu_result_o[31] == 1'b1)); 
        end
      SUBC:
        begin
          {alu_C_flag_o , alu_result_o} = {1'b0,alu_srcA_i[31:0]}-{1'b0,srcB};
          alu_V_flag_o = (alu_srcA_i[31] == 1'b1) && (alu_srcB_i[31] == 1'b0) && (alu_result_o[31] == 1'b0)
                          || (alu_srcA_i[31] == 1'b0) && (alu_srcB_i[31] == 1'b1) && (alu_result_o[31] == 1'b1); 
        end
      OR:
        begin
          alu_result_o = alu_srcA_i[31:0] | alu_srcB_i[31:0];
          alu_C_flag_o = 1'b0;
          alu_V_flag_o = 1'b0;
        end
      INV:
        begin
          alu_result_o = ~alu_srcA_i[31:0];
          alu_C_flag_o = 1'b0;
          alu_V_flag_o = 1'b0;
        end
      AND:
        begin
          alu_result_o = alu_srcA_i[31:0] & alu_srcB_i[31:0];
          alu_C_flag_o = 1'b0;
          alu_V_flag_o = 1'b0;
        end       
      PASS:
        begin
          alu_result_o = alu_srcA_i[31:0];
          alu_C_flag_o = alu_srcA_i[32];
          alu_V_flag_o = alu_srcA_i[33];
        end
    endcase
  end

endmodule  
