

module instruction_memory_tb #(parameter N = 10, parameter M = 1024)  (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;
  reg      	    	rst_n;

  // Input signals.
  reg [31:0] next_pc_tb;
  // Output signals.
  wire [N-1:0] pc_tb;


	int i;


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
  program_counter_mod #(N) program_counter_u (
    .clk(clk),
    .rst_n(rst_n),
    .next_pc_i(next_pc_tb),
    .pc_o(pc_tb)    
  );


  
 

  
  // ----------------------------------
  // Testbench tasks.
  // ----------------------------------

    task compare_Nb(
        input 		[N-1:0]actual_Q,
        input 		[N-1:0]expected_Q
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
    rst_n = 1'b1;
    #2;
    rst_n = 1'b0;
    #2;
    rst_n = 1'b1;
    #2;
    compare_Nb(pc_tb, {N{1'b0}});

    for(i=0 ; i<8 ; i++) begin
      next_pc_tb = $random;
      @(posedge clk);
      #2;
      compare_Nb(pc_tb, next_pc_tb[N-1:0]);
    end
    
    
    @(posedge clk);
    #2;
    
    $display("// ----------------------------------");
    $display("// TEST PASSED.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule