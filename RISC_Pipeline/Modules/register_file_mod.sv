// -----------------------------------------------------------------------------
// Description:
// ------------
//
// Register file for one cycle RISC
//
// -----------------------------------------------------------------------------


module register_file_mod ( 
  input  wire      	 	clk,
  input  wire      	 	rst_n,
  // ----------------------------------
  // Data Inputs
  // ----------------------------------
  input  wire      	 	[0:4]address1_i,
  input  wire      	 	[0:4]address2_i,
  input  wire      	 	[0:4]address3_i,
  input  wire      	 	[0:33]write_data3_i,
  input  wire      	 	write_en3_i,
  

  // ----------------------------------
  // Outputs
  // ----------------------------------
  output wire      	   	 [0:33]read_data1_o,
  output wire      	   	 [0:33]read_data2_o
);
  // ----------------------------------
  // Internal Data
  // ----------------------------------
  reg [0:33]reg_mem[0:31]; 
  
  parameter   MEM_INIT_FILE   = "reg_memory.bin"; // Path to the file with the initial memory values   


  assign read_data1_o = reg_mem[address1_i];
  
  assign read_data2_o = reg_mem[address2_i];
  
  
    
  always @(negedge clk or negedge rst_n) begin
    if(~rst_n) begin
      if (MEM_INIT_FILE != "") begin
          $readmemb(MEM_INIT_FILE, reg_mem);
      end
    end
    else begin
      if (write_en3_i) begin
        reg_mem[address3_i] <= write_data3_i;
      end
    end
  end
  
  

  
endmodule  
