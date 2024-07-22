

module extender_tb (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;


  // Input signals.
  reg [11:0] dataA_tb;
  reg [4:0] dataB_tb;
  reg ext_ctrl_tb;
  // Output signals.
  wire [31:0] ext_data_tb;



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
  extender_mod  extender_u (
    .dataA_i(dataA_tb),
    .dataB_i(dataB_tb),
    .ext_ctrl_i(ext_ctrl_tb),
    .ext_data_o(ext_data_tb)  
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
    dataA_tb = 12'b101010101010;
    dataB_tb = 5'b11111;
    ext_ctrl_tb = 1'b0;
    #2;
    compare_32b(ext_data_tb, {dataA_tb[11], 20'd0, dataA_tb[10:0]});
    ext_ctrl_tb = 1'b1;
    #2;
    compare_32b(ext_data_tb, {dataA_tb[11], 15'd0, dataA_tb[10:0], dataB_tb});

    
    @(posedge clk);
    #2;
    
    $display("// ----------------------------------");
    $display("// TEST PASSED.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule