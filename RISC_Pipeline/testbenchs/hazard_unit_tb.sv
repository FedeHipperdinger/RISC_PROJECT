

module hazard_unit_tb #(parameter N = 10)  (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;
  reg      	 	   rst_n;

  // Input signals.
  reg [31:0]  instr_tb;
  reg [1:0]   pc_sel_tb;

  // Output signals.
  wire      	   stallF_tb;
  wire      	   stallD_tb;
  wire      	   flushD_tb;
  wire      	   flushE_tb;
  wire [1:0]	   bypA_sel_tb;
  wire [1:0]	   bypB_sel_tb;

  
  // Instruction types (T)
  localparam MOVEMENT 		    = 2'b00;    // Data hazard --> Stall
  localparam LOGIC 			      = 2'b10;    // RAW data hazard --> Bypass
  localparam ARITHMETIC 	    = 2'b11;    // RAW data hazard --> Bypass
  localparam FLOW_CONTROL 	  = 2'b01;    // Control hazard --> Flush/stall
  
  //Movement Instructions
  localparam LOAD 		        = 3'b000;   // Stall
  localparam STORE 			      = 3'b010;
  localparam LOADI 	          = 3'b001;
  localparam STOREI 	        = 3'b011;
  localparam MOV 	            = 3'b100;

  //Logic Instructions
  localparam OR 		          = 3'b000;
  localparam INV 			        = 3'b001;
  localparam AND 	            = 3'b010;

  //Arithmetic Instructions
  localparam ADD 		          = 3'b000;
  localparam SUB 			        = 3'b001;
  localparam ADDC 	          = 3'b010;
  localparam SUBC 	          = 3'b011;

  //Flow control Instructions
  localparam JUMP 		        = 3'b000;   // Stall
  localparam BZ 			        = 3'b001;   // Flush
  localparam BNZ 	            = 3'b010;   // Flush
  localparam BC 	            = 3'b011;   // Flush
  localparam BV 	            = 3'b100;   // Flush
  localparam JAL 	            = 3'b101;   // Stall
  localparam JRAL 	          = 3'b110;   // Stall
  localparam RET 	            = 3'b111;   // Stall

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
    hazard_unit_mod #(N) u_hazard_unit (
        // ----------------------------------
        //  inputs
        // ----------------------------------
        .clk(clk),
        .rst_n(rst_n),
        .instr_i(instr_tb),
        .pc_sel_i(pc_sel_tb),

        // ----------------------------------
        //  outputs
        // ----------------------------------
        .stallF_o(stallF_tb),
        .stallD_o(stallD_tb),
        .flushD_o(flushD_tb),
        .flushE_o(flushE_tb),
        .bypA_sel_o(bypA_sel_tb),
        .bypB_sel_o(bypB_sel_tb)
    );


  
 

  
  // ----------------------------------
  // Testbench tasks.
  // ----------------------------------
    task compare_1b(
        input 		actual_Q,
        input 		expected_Q
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
          $display("Instruction = %b ",
                     instr_tb);
            $display("// ----------------------------------");
            $finish;
        end
        
    endtask

    task compare_2b(
        input 		[1:0]actual_Q,
        input 		[1:0]expected_Q
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
          $display("Instruction = %b ",
                     instr_tb);
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
    
    rst_n = 1'b0;
    #2;
    rst_n = 1'b1;
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);

    instr_tb = {MOVEMENT, STORE, 12'd0, 5'd0, 5'd0, 5'd0};
    pc_sel_tb = 2'b00;

    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);
    
    $display("Load data hazard Test ");
    
    instr_tb = {MOVEMENT, LOAD, 12'd0, 5'd0, 5'd0, 5'd0};

    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b1);
    compare_1b(stallD_tb, 1'b1);
    compare_1b(flushD_tb, 1'b1);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);
    
    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b1);
    compare_1b(stallD_tb, 1'b1);
    compare_1b(flushD_tb, 1'b1);
    compare_1b(flushE_tb, 1'b1);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);
    
    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);
    
    instr_tb = {MOVEMENT, STORE, 12'd0, 5'd0, 5'd0, 5'd0};
    
    $display("RAW Hazard Test ");
    
    @(posedge clk);
    
    instr_tb = {LOGIC, OR, 12'd0, 5'd1, 5'd2, 5'd3};    // Ra = 3, Rb = 2, Rc = 1
	#2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);
    
    @(posedge clk);
    
    
    instr_tb = {LOGIC, AND, 12'd0, 5'd2, 5'd3, 5'd1};   // Ra = 1, Rb = 3, Rc = 2
	
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b01); // Rb tries to use Ra=3 after 1 cycle
    compare_2b(bypB_sel_tb, 2'b00);
    
    @(posedge clk);
    
    instr_tb = {ARITHMETIC, ADD, 12'd0, 5'd2, 5'd3, 5'd4};   // Ra = 4, Rb = 3, Rc = 2
	
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b10); // Rb tries to use Ra=3 after 2 cycles
    compare_2b(bypB_sel_tb, 2'b00);
    
    @(posedge clk);
    instr_tb = {LOGIC, OR, 12'd0, 5'd4, 5'd1, 5'd4};   // Ra = 4, Rb = 1, Rc = 4
	
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b10); // Rb tries to use Ra=3 after 2 cycles
    compare_2b(bypB_sel_tb, 2'b01); // Rc tries to use Ra=4 after 1 cycle

    @(posedge clk);
    instr_tb = {LOGIC, AND, 12'd0, 5'd2, 5'd3, 5'd1};   // Ra = 1, Rb = 3, Rc = 2
	#2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);
    
    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);
    
    $display("Control Hazard Test ");

    instr_tb = {FLOW_CONTROL, JUMP, 12'd0, 5'd0, 5'd0, 5'd0};   // Ra = 0, Rb = 0, Rc = 0

    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b1);
    compare_1b(flushD_tb, 1'b1);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);

    instr_tb = {LOGIC, AND, 12'd0, 5'd2, 5'd3, 5'd1};   // Ignored instruction

    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);

    instr_tb = {LOGIC, AND, 12'd0, 5'd2, 5'd4, 5'd1};   // Ignored instruction

    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);

    // Jumped to this instruction
    instr_tb = {LOGIC, AND, 12'd0, 5'd2, 5'd3, 5'd1};  // Ra = 1, Rb = 3, Rc = 2 

    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);

    instr_tb = {FLOW_CONTROL, BZ, 12'd0, 5'd0, 5'd0, 5'd0};   // Ra = 0, Rb = 0, Rc = 0

    @(posedge clk);
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b00);
    compare_2b(bypB_sel_tb, 2'b00);

    instr_tb = {LOGIC, AND, 12'd0, 5'd2, 5'd1, 5'd3};  // Ra = 3, Rb = 1, Rc = 2   
	
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b0);
    compare_1b(flushE_tb, 1'b0);
    compare_2b(bypA_sel_tb, 2'b10); // Rb tries to use Ra=1 after 2 cycles
    compare_2b(bypB_sel_tb, 2'b00);
    
    @(posedge clk);
    

    instr_tb = {LOGIC, AND, 12'd0, 5'd2, 5'd3, 5'd1};   // Ra = 1, Rb = 3, Rc = 2
    pc_sel_tb = 2'b01;
    
    
    #2;
    compare_1b(stallF_tb, 1'b0);
    compare_1b(stallD_tb, 1'b0);
    compare_1b(flushD_tb, 1'b1);
    compare_1b(flushE_tb, 1'b1);
    compare_2b(bypA_sel_tb, 2'b01); // Rb tries to use Ra=3 after 1 cycle
    compare_2b(bypB_sel_tb, 2'b00);

    @(posedge clk);








    @(posedge clk);
    #2;
    @(posedge clk);
    #2;
    @(posedge clk);
    #2;

    $display("// ----------------------------------");
    $display("// TEST PASSED.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule