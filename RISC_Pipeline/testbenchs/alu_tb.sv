

module alu_tb #()  (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;

  // Input signals.
  reg      	    	[33:0]srcA_tb;
  reg      	    	[33:0]srcB_tb;
  reg      	    	[1:0]ctrl_tb;

  // Output signals.
  wire					 alu_C_flag_tb; 
  wire					 alu_V_flag_tb; 
  wire 					 [31:0]alu_result_tb;
  
  // Testbench signals.

  
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
  alu_mod u_alu_mod (
    // ----------------------------------
    //  inputs
    // ----------------------------------
    .clk			(clk),
    .alu_srcA_i 		(srcA_tb ),
    .alu_srcB_i 		(srcB_tb ),
    .alu_ctrl_i 		(ctrl_tb ),

    // ----------------------------------
    //  outputs
    // ----------------------------------
    .alu_result_o    	(alu_result_tb	   ),
    .alu_C_flag_o    	(alu_C_flag_tb	   ),
    .alu_V_flag_o    	(alu_V_flag_tb	   )
  );
  
 

  
  // ----------------------------------
  // Testbench tasks.
  // ----------------------------------
    // - ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- ---
    // task : compare_output
    //
    // Description :
    // Used to compare the expected outputs with the actual DUTs responses.
    //
    // Input Arguments :
    // expected_res =
    // actual_res =    
  	//
    // Output Arguments :
    // None
    // - ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- --- ---- ---
  task compare_flag(
    input 		actual_Q,
    input 		expected_Q
  );	
    reg condition;
    
    $display("// Comparison");
    condition = (expected_Q!=actual_Q);

    if (condition) begin
    	$display("// ----------------------------------");
     	$display("TEST FAILED");
 
      $display("At time %0d Q=%b ",
                $time, actual_Q);
      $display("out should be expected_Q=%b ",                  expected_Q);
    	$display("// ----------------------------------");
        $finish;
    end
    
  endtask
  
  task compare_output(
    input 		[33:0]actual_Q,
    input 		[33:0]expected_Q
  );	
    reg condition;
    
    $display("// Comparison");
    condition = (expected_Q!=actual_Q);

    if (condition) begin
    	$display("// ----------------------------------");
     	$display("TEST FAILED");
 
      $display("At time %0d Q=%b ",
                $time, actual_Q);
      $display("out should be expected_Q=%b ",                  expected_Q);
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
    //add
    ctrl_tb = 2'b00;
    @(posedge clk);
    #2;
    srcA_tb = {34{1'b1}};
    srcB_tb = {34{1'b1}};

    @(posedge clk);
    #2;
    compare_output(alu_result_tb,{{31{1'b1}},1'b0});
    compare_flag(alu_C_flag_tb,1'b1);
    compare_flag(alu_V_flag_tb,1'b0);
    
    srcA_tb = {3'b000,{31{1'b1}}};
    srcB_tb = {3'b000,{31{1'b1}}};

    @(posedge clk);
    #2;
    compare_output(alu_result_tb,{{31{1'b1}},1'b0});
    compare_flag(alu_C_flag_tb,1'b0);
    compare_flag(alu_V_flag_tb,1'b1);
    
    
    srcA_tb = {3'b111,{31{1'b0}}};
    srcB_tb = {3'b111,{31{1'b0}}};

    @(posedge clk);
    #2;
    compare_output(alu_result_tb,32'b0);
    compare_flag(alu_C_flag_tb,1'b1);
    compare_flag(alu_V_flag_tb,1'b1);
    
    
    // addc
    
    ctrl_tb = 2'd2;
    @(posedge clk);
    #2;
    srcA_tb = {34{1'b1}};
    srcB_tb = {34{1'b1}};

    @(posedge clk);
    #2;
    compare_output(alu_result_tb,{{31{1'b1}},1'b0});
    compare_flag(alu_C_flag_tb,1'b0);
    compare_flag(alu_V_flag_tb,1'b0);
    
    srcA_tb = {3'b000,{31{1'b1}}};
    srcB_tb = {3'b010,{31{1'b1}}};

    @(posedge clk);
    #2;
    compare_output(alu_result_tb,{{31{1'b1}},1'b0});
    compare_flag(alu_C_flag_tb,1'b1);
    compare_flag(alu_V_flag_tb,1'b1);
    
    
    srcA_tb = {3'b111,{31{1'b0}}};
    srcB_tb = {3'b111,{31{1'b0}}};

    @(posedge clk);
    #2;
    compare_output(alu_result_tb,32'b0);
    compare_flag(alu_C_flag_tb,1'b0);
    compare_flag(alu_V_flag_tb,1'b1);
    
    
//     reg      	    	[0:31]srcA_tb;
//   reg      	    	[0:31]srcB_tb;
//   reg      	    	ctrl_tb;

//   // Output signals.
//   //usando un wire nos despreocupamos de que laa salida del modulo sea un wire o un reg
//   wire					 alu_C_flag_tb; 
//   wire					 alu_V_flag_tb; 
//   wire 					 [0:31]alu_result_tb;
   
         
    
    
    $display("// ----------------------------------");
    $display("// TEST PASSED.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule