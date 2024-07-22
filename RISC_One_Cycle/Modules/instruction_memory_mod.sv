// -----------------------------------------------------------------------------
// Description:
// ------------
//
//Instruction Memory for one cycle RISC
// -----------------------------------------------------------------------------


module instruction_memory_mod #(parameter N = 10, parameter M = 1024)( 
  input  wire      	 	clk,
  input  wire      	 	rst_n,
  // ----------------------------------
  // Data Inputs
  // ----------------------------------
  input  wire      	 	[0:N-1]address_i,  

  // ----------------------------------
  // Outputs
  // ----------------------------------
  output wire      	   	 [31:0]read_data_o
);
  // ----------------------------------
  // Internal Data
  // ----------------------------------
  reg [0:7]reg_mem[0:M-1];                         // each word is 32 bits wide. This means there are M/4 valid memory directions (256)
  parameter   MEM_INIT_FILE   = "output.bin";     // Path to the file with the initial memory values   


  always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
      if (MEM_INIT_FILE != "") begin
          $readmemb(MEM_INIT_FILE, reg_mem);
      end
    end
  end
  
  assign read_data_o = {reg_mem[address_i],reg_mem[{address_i[0:N-3],address_i[N-2:N-1]+2'd1}],reg_mem[{address_i[0:N-3],address_i[N-2:N-1]+2'd2}],reg_mem[{address_i[0:N-3],address_i[N-2:N-1]+2'd3}]};

endmodule  
