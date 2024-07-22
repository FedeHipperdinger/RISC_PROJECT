// -----------------------------------------------------------------------------
// Description:
// ------------
//
// Data Memory for one cycle RISC
// 8 bits wide -- M = 1024 addresses (N= 10)
// -----------------------------------------------------------------------------


module data_memory_mod #(parameter N = 10, parameter M = 1024)( 
  input  wire      	 	clk,
  input  wire      	 	rst_n,
  // ----------------------------------
  // Data Inputs
  // ----------------------------------
  input  wire      	 	[0:N-1]address_i,
  input  wire      	 	[0:31]write_data_i,
  input  wire      	 	write_en_i,    

  // ----------------------------------
  // Outputs
  // ----------------------------------
  output wire      	   	 [0:31]read_data_o
);
  // ----------------------------------
  // Internal Data
  // ----------------------------------
  reg [0:7]reg_mem[0:M-1]; 
  parameter   MEM_INIT_FILE   = "data_memory.hex"; // Path to the file with the initial memory values   

  assign read_data_o = {reg_mem[address_i], reg_mem[address_i + {{N-2{1'b0}}, 2'd1}], reg_mem[address_i + {{N-2{1'b0}}, 2'd2}], reg_mem[address_i + {{N-2{1'b0}}, 2'd3}]};

    
  always @(posedge clk or negedge rst_n) begin
    if(~rst_n) begin
      if (MEM_INIT_FILE != "") begin
          $readmemh(MEM_INIT_FILE, reg_mem);
      end
    end
    else begin
      if (write_en_i) begin
        reg_mem[address_i]      <= write_data_i[0:7];
        reg_mem[address_i+8'd1] <= write_data_i[8:15];
        reg_mem[address_i+8'd2] <= write_data_i[16:23];
        reg_mem[address_i+8'd3] <= write_data_i[24:31];
      end
    end
    
  end
  
  

  
endmodule  
