

module data_memory_tb  (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;
  reg      	 	   rst_n;

  // Input signals.
  reg	[0:4] address1_tb;
  reg	[0:4] address2_tb;
  reg	[0:4] address3_tb;

  reg [0:33] write_data3_tb;
  reg write_en3_tb;
  // Output signals.
  wire [0:33] read_data1_tb;
  wire [0:33] read_data2_tb;





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
  register_file_mod  register_file_u (
    .clk(clk),
    .rst_n(rst_n),
    .address1_i(address1_tb),
    .address2_i(address2_tb),
    .address3_i(address3_tb),
    .write_data3_i(write_data3_tb),
    .write_en3_i(write_en3_tb),
    .read_data1_o(read_data1_tb),
    .read_data2_o(read_data2_tb)
  );


  
 

  
  // ----------------------------------
  // Testbench tasks.
  // ----------------------------------

    task compare_34b(
        input 		[0:33]actual_Q,
        input 		[0:33]expected_Q
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
    
    // Start test.     
    rst_n = 1'b1;
    #2;
    rst_n = 1'b0;
    #2;
    rst_n = 1'b1;
    #2;

    // test reset
    for (int i = 0; i < 32; i++) begin
      address1_tb = i;
      #2;
      @(posedge clk);
      #2;
      compare_34b(read_data1_tb, 34'b0);
    end
    $display("// ----------------------------------");
    $display("// RESET PASSED.");
    $display("// ----------------------------------");
    // test write and read
    for (int i = 0; i < 32; i++) begin
      write_data3_tb = i;
      write_en3_tb = 1'b1;
      address3_tb = i;
      #2;
      @(posedge clk);
      address1_tb = i;
      address2_tb = i-1;
      #2;
      compare_34b(read_data1_tb, write_data3_tb);
      if (i > 0) begin
        compare_34b(read_data2_tb, write_data3_tb-34'b1);
      end
    end

    

   
    
    @(posedge clk);
    #2;
    
    $display("// ----------------------------------");
    $display("// TEST PASSED.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule