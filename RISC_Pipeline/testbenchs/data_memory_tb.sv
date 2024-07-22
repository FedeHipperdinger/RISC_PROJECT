

module data_memory_tb #(parameter N = 10, parameter M = 1024)  (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;
  reg      	 	   rst_n;

  // Input signals.
  reg [0:N-1] address_tb;
  reg [0:31] write_data_tb;
  reg write_en_tb;
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
  data_memory_mod #(N, M) data_memory_u (
    .clk(clk),
    .rst_n(rst_n),
    .address_i(address_tb),
    .write_data_i(write_data_tb),
    .write_en_i(write_en_tb),
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
    
    // Start test.     
    rst_n = 1'b1;
    #2;
    rst_n = 1'b0;
    #2;
    rst_n = 1'b1;
    #2;

    // Test read
    address_tb = 10'd0;
    #2;
    compare_32b(read_data_tb, 32'd4);

    address_tb = 10'd4;
    #2;
    compare_32b(read_data_tb, 32'd6);

    address_tb = 10'd8;
    #2;
    compare_32b(read_data_tb, 32'd8);

    address_tb = 10'd12;
    #2;
    compare_32b(read_data_tb, 32'd55);

    address_tb = 10'd16;
    #2;
    compare_32b(read_data_tb, 32'd133);

    address_tb = 10'd20;
    #2;
    compare_32b(read_data_tb, 32'd255);

    // Test write
    address_tb = 10'd0;
    write_data_tb = 32'd100;
    write_en_tb = 1'b1;
    #2;
    @(posedge clk);
    #2;
    compare_32b(read_data_tb, write_data_tb);


   
    
    @(posedge clk);
    #2;
    
    $display("// ----------------------------------");
    $display("// TEST PASSED.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule