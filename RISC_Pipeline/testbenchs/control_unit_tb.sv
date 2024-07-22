

module control_unit_tb #()  (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;
  reg      	 	   rst_n;

  // Input signals.
    reg      	 	   [1:0]T_tb;  
    reg      	 	   [2:0]OPC_tb; 
    reg      	 	   [33:0]reg_tb; 
    reg      	 	   C_flag_tb;  
    reg      	 	   V_flag_tb; 
    reg      	 	   [1:0]addr_bits_tb; //two least significant bits of the data memory input address  
    reg      	 	   [1:0]pc_bits_tb; //two least significant bits of the program counter

    // Output signals.
    wire [1:0] pc_sel_tb;
    wire data_mem_w_ctrl_tb;
    wire data_mem_w_sel_tb;
    wire [2:0] alu_ctrl_tb;
    wire alu_srcA_sel_tb;
    wire [1:0] alu_srcB_sel_tb;
    wire reg_file_w_ctrl_tb;
    wire [1:0]reg_file_w_sel_tb;
    wire extend_ctrl_tb;
    wire [1:0] status_flags_tb;
    wire [2:0] exception_flags_tb;

  // Instruction types (T)
    localparam MOVEMENT 		    = 2'b00;
    localparam LOGIC 			      = 2'b10;
    localparam ARITHMETIC 	    = 2'b11;
    localparam FLOW_CONTROL 	  = 2'b01;
    
    //Movement Instructions
    localparam LOAD 		        = 3'b000;
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
    localparam JUMP 		        = 3'b000;
    localparam BZ 			        = 3'b001;
    localparam BNZ 	            = 3'b010;
    localparam BC 	            = 3'b011;
    localparam BV 	            = 3'b100;
    localparam JAL 	            = 3'b101;
    localparam JRAL 	          = 3'b110;
    localparam RET 	            = 3'b111;



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
    control_unit_mod u_control_unit (
        // ----------------------------------
        //  inputs
        // ----------------------------------
        .rst_n			(rst_n),
        .T_i			(T_tb),
        .OPC_i			(OPC_tb),
        .reg_i			(reg_tb),
        .C_flag_i		(C_flag_tb),
        .V_flag_i		(V_flag_tb),
        .addr_bits_i	(addr_bits_tb),
        .pc_bits_i		(pc_bits_tb),
        
        // ----------------------------------
        //  outputs
        // ----------------------------------
        .pc_sel_o		    (pc_sel_tb),
        .data_mem_w_ctrl_o	(data_mem_w_ctrl_tb),
        .data_mem_w_sel_o	(data_mem_w_sel_tb),
        .alu_ctrl_o		    (alu_ctrl_tb),
        .alu_srcA_sel_o	    (alu_srcA_sel_tb),
        .alu_srcB_sel_o	    (alu_srcB_sel_tb),
        .reg_file_w_ctrl_o	(reg_file_w_ctrl_tb),
        .reg_file_w_sel_o	(reg_file_w_sel_tb),
        .extend_ctrl_o	    (extend_ctrl_tb),
        .status_flags_o	    (status_flags_tb),
        .exception_flags_o	(exception_flags_tb)
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
            $display("// ----------------------------------");
            $finish;
        end
        
    endtask

    task compare_3b(
        input 		[2:0]actual_Q,
        input 		[2:0]expected_Q
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
    
    rst_n = 1'b0;
    #2;
    rst_n = 1'b1;
    #2;
    compare_3b(exception_flags_tb, 3'b000);

    @(posedge clk);
    #2;

    //Movement Instructions

    T_tb = MOVEMENT;
    OPC_tb = LOAD;
    // reg_tb = {34{1'b1}};
    // C_flag_tb = 1'b1;
    // V_flag_tb = 1'b0;
    // addr_bits_tb = 2'b00;
    // pc_bits_tb = 2'b00;
    
    @(posedge clk);
    #2;
    compare_1b(reg_file_w_ctrl_tb, 1'b1);
    compare_1b(alu_srcA_sel_tb, 1'b0);
    compare_2b(alu_srcB_sel_tb, 2'd1);
    compare_1b(extend_ctrl_tb, 1'b1);
	
    $display("// ----------------------------------");
    $display("// MOVEMENT PASSED.");
    $display("// ----------------------------------");

    // Logic Instructions

    T_tb = LOGIC;
    OPC_tb = OR;

    @(posedge clk);
    #2;
    compare_1b(reg_file_w_ctrl_tb, 1'b1);
    compare_2b(reg_file_w_sel_tb, 2'd3);
    compare_3b(alu_ctrl_tb, 3'd4);
    
    $display("// ----------------------------------");
    $display("// LOGIC PASSED.");
    $display("// ----------------------------------");
    // Arithmetic Instructions

    T_tb = ARITHMETIC;
    OPC_tb = ADD;

    @(posedge clk);
    #2;
    compare_1b(reg_file_w_ctrl_tb, 1'b1);
    compare_2b(reg_file_w_sel_tb, 2'd2);
    compare_1b(alu_srcA_sel_tb, 1'b0);
    compare_2b(alu_srcB_sel_tb, 2'd0);
    compare_2b(status_flags_tb, {V_flag_tb,C_flag_tb});
    compare_3b(alu_ctrl_tb, 3'd0);
    
    $display("// ----------------------------------");
    $display("// ARITHMETIC PASSED.");
    $display("// ----------------------------------");

    // Flow Control Instructions

    T_tb = FLOW_CONTROL;
    OPC_tb = JUMP;

    @(posedge clk);
    #2;
    compare_2b(pc_sel_tb, 2'd1);
    compare_1b(alu_srcA_sel_tb, 1'b0);
    compare_2b(alu_srcB_sel_tb, 2'd2);
    
    $display("// ----------------------------------");
    $display("// FLOW CONTROL PASSED.");
    $display("// ----------------------------------");
    
    
    @(posedge clk);
    #2;
    
    $display("// ----------------------------------");
    $display("// TEST PASSED.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule