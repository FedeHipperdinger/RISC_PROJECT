// -----------------------------------------------------------------------------
// Description:
// ------------
//
//Extend module for one cycle RISC
// ext_ctrl_i = 0 >>> Sign_ext(Imm) is the Immediate field extended to 32 bits while keeping its sign.
// ext_ctrl_i = 0 >>> Sign_ext(Imm, Rc) is the concatenation of Immediate and Rc fields and extended to 32 bits keeping the sign of Immediate.
//
// -----------------------------------------------------------------------------


module extender_mod ( 
  // ----------------------------------
  // Data Inputs
  // ----------------------------------
  input  wire      	 	[11:0]dataA_i,  //imm
  input  wire      	 	[4:0]dataB_i,  //Rc
  input  wire      	 	ext_ctrl_i,  


  // ----------------------------------
  // Outputs
  // ----------------------------------
  output reg      	   	 [31:0]ext_data_o
);

assign ext_data_o = (ext_ctrl_i==1'b1) ? {{16{dataA_i[11]}}, dataA_i[10:0], dataB_i} : {{21{dataA_i[11]}}, dataA_i[10:0]};

endmodule  
