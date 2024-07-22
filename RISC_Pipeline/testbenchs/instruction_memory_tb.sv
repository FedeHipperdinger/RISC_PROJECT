

module instruction_memory_tb #(parameter N = 10, parameter M = 1024)  (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;

  // Input signals.
  reg [0:N-1] address_tb;
  // Output signals.
  wire [0:31] read_data_tb;





  // ----------------------------------
  // Clock model.
  // ----------------------------------
  `define T_CLK         20     // Clock Period
  initial begin
   clk = 1'b0;
   forever # (`T_CLK/2) clk = ~clk;
  end
  
  // ----------------------------------
  // DUT instantiation.
  // ----------------------------------
  instruction_memory_mod #(N, M) instruction_memory_u (
    .clk(clk),
    .address_i(address_tb),
    .read_data_o(read_data_tb)
  );


  
 

  
  // ----------------------------------
  // Testbench tasks.
  // ----------------------------------

    task compare_32b(
        input 		[31:0]actual_Q,
        input 		[31:0]expected_Q
      );	
        reg condition;
        
        $display("// Comparison");
        condition = (expected_Q!=actual_Q);
    
        if (condition) begin
            $display("// ----------------------------------");
             $display("TEST FAILED");
     
          $display("At time %0d Q=%d ",
                    $time, actual_Q);
          $display("out should be expected_Q=%d ",                  expected_Q);
            $display("// ----------------------------------");
            $finish;
        end
        
    endtask


  

  
  // ----------------------------------
  // Simulation.
  // ----------------------------------
  initial begin
    $dumpfile("dump1.vcd"); 
    $dumpvars;
    
    $display("// ----------------------------------");
    $display("// TEST START.");
    $display("// ----------------------------------");
    


    // Test read
    address_tb = 10'd0;
    #2;
    compare_32b(read_data_tb, 32'b00000000000000000000000000000000
    );

    address_tb = 10'd4;
    #2;
    compare_32b(read_data_tb, 32'b00000000000000000000000010000001);

    address_tb = 10'd8;
    #2;
    compare_32b(read_data_tb, 32'b00001000000000000011010000000010);




   
    
    @(posedge clk);
    #2;
    
    $display("// ----------------------------------");
    $display("// TEST PASSED.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule