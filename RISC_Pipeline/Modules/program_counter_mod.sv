// -----------------------------------------------------------------------------
// Description:
// ------------
//
// Program counter for one cycle RISC
// For 256 addresses (N=10) (byte addressable)
//
// -----------------------------------------------------------------------------


module program_counter_mod #(parameter N = 10)( 
  input  wire      	 	clk,
  input  wire      	 	rst_n,
  input  wire      	 	enable_i,
  // ----------------------------------
  // Data Inputs
  // ----------------------------------
  input  wire      	 	[31:0]next_pc_i,
  

  // ----------------------------------
  // Outputs
  // ----------------------------------
  output reg      	   	 [N-1:0]pc_o
);

  always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
      pc_o <= {N{1'b0}};
    end
    else begin
      if(enable_i) begin
        pc_o <= next_pc_i[N-1:0];
      end
    end
  end

  
endmodule  

