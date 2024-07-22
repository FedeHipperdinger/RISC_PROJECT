`include "top_module.sv"

module RISC_oneCycle_tb #(parameter N = 10)  (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;
  reg      	    	rst_n;


  // Output signals.
  wire      [2:0]exception_flags_tb_o;
  wire      [31:0]alu_result_tb_o;
  wire      [33:0]reg_file_r_data1_tb_o;
  wire      [33:0]reg_file_r_data2_tb_o;
  wire      [31:0]data_mem_r_data_tb_o;
  wire      [N-1:0]pc_tb_o;
  wire      [31:0]inst_read_data_tb_o;
  wire      alu_C_flag_tb_o;
  wire      alu_V_flag_tb_o;
 
  // Testbench signals.
  reg [N-1:0]prev_pc;
  
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
  top_module u_top_module (
    // ----------------------------------
    //  inputs
    // ----------------------------------
    .clk			(clk),
    .rst_n			(rst_n),
    // ----------------------------------
    //  outputs
    // ----------------------------------
    .exception_flags_o(exception_flags_tb_o),
    .alu_result_o(alu_result_tb_o),
    .reg_file_r_data1_o(reg_file_r_data1_tb_o),
    .reg_file_r_data2_o(reg_file_r_data2_tb_o),
    .data_mem_r_data_o(data_mem_r_data_tb_o),
    .pc_o(pc_tb_o),
    .inst_read_data_o(inst_read_data_tb_o),
    .alu_C_flag_o(alu_C_flag_tb_o),
    .alu_V_flag_o(alu_V_flag_tb_o)

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
 
      $display("At time %0d Q=%b ",
                $time, actual_Q);
      $display("out should be expected_Q=%b ",                  expected_Q);
    	$display("// ----------------------------------");
        $finish;
    end
    
  endtask
  
  task compare_exceptions(
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
 
      $display("At time %0d Q=%b ",
                $time, actual_Q);
      $display("out should be expected_Q=%b ",                  expected_Q);
    	$display("// ----------------------------------");
        $finish;
    end
    
  endtask

  task compare_34b(
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
  // --------------------- Targeted testbench. --------------------
  initial begin
    $dumpfile("dump1.vcd"); 
    $dumpvars;
    
    $display("// ----------------------------------");
    $display("// TEST START.");
    $display("// ----------------------------------");
    
    
    // Start test.  
    //reset
    rst_n = 1'b1;
    #2;
    rst_n = 1'b0;
    #2;
    rst_n = 1'b1;
    
    //---------------------------------------------------------- First part ----------------------------------------------------------
    @(posedge clk);
    #2;
    // load    0,0,0       Cargar Mem[reg[0]] en el registro 0, reg[0]=Mem[0]=4
    compare_34b(reg_file_r_data1_tb_o, 34'd0);
    compare_32b(data_mem_r_data_tb_o, 32'd4);

    @(posedge clk);
    #2;
    // load    1,0,0       Cargar Mem[reg[0]] en el registro 1, reg[1]=Mem[4]=6
    compare_34b(reg_file_r_data1_tb_o, 34'd4);
    compare_32b(data_mem_r_data_tb_o, 32'd6);

    @(posedge clk);
    #2;
    // loadi   2,44        Cargar 44 en el registro 2, reg[2]=44       //44 = {26'd0 , 101100}
    compare_32b(alu_result_tb_o, 32'd44);
    @(posedge clk);
    #2;
    // testing arithmetic operations
    // add     3,2,1       Calcular reg[1] + reg[2] y guardar en reg[3], reg[3]=50
    compare_34b(reg_file_r_data1_tb_o, 34'd44);
    compare_34b(reg_file_r_data2_tb_o, 34'd6);
    compare_32b(alu_result_tb_o, 32'd50);

    @(posedge clk);
    #2;
    // store   0,3,0       Guardar reg[3] en Mem[reg[0]], Mem[4]=50
    compare_34b(reg_file_r_data1_tb_o, 34'd4);
    compare_34b(reg_file_r_data2_tb_o, 32'd50);

    @(posedge clk);
    #2;
    // load    0,0,0       Cargar Mem[reg[0]] en el registro 0, reg[0]=Mem[4]=50  // Cambiar a 0,1,0 para testear mem misalignment
    compare_34b(reg_file_r_data1_tb_o, 34'd4);
    compare_32b(data_mem_r_data_tb_o, 32'd50);

    @(posedge clk);
    #2;
    // add     1,1,1       Guardar en reg[1] la suma reg[1]+reg[1], reg[1]=12
    compare_32b(alu_result_tb_o, 32'd12);
    
    @(posedge clk);
    #2;
    // jump    1           PC = PC + reg[1] = PC + 12  // Cambiar a 3 para sumar 50 y testear instruction load misalignment
    prev_pc = pc_tb_o;
    compare_34b(reg_file_r_data1_tb_o, 34'd12);
    
    @(posedge clk);
    #2;
    compare_Nb(pc_tb_o, prev_pc + {{(N-4){1'b0}},4'd12});
    // add     3,2,1       Instruccion salteada
    // store   0,3,0       Instruccion salteada
    // inv     1,1         reg[1] = inv(reg[1])
    compare_32b(alu_result_tb_o, ~32'd12);
    @(posedge clk);
    #2;
    // add     10,1,1      reg[10]=reg[1]+reg[1] //al ser inv(12), reg[1] es muy grande y esto da overflow
    compare_34b(reg_file_r_data1_tb_o, {2'd0, ~32'd12});
    compare_32b(alu_result_tb_o, ~32'd12+~32'd12);
    compare_1b(alu_C_flag_tb_o, 1'b1);
    
    @(posedge clk);
    #2;
    // bc      11,10,8     Si hay overflow PC = PC + reg[11] + 8 = PC + 8
    prev_pc = pc_tb_o;
    @(posedge clk);
    #2;
    compare_Nb(pc_tb_o, prev_pc + {{(N-4){1'b0}},4'd8});
    // inv     1,1         Instruccion salteada
    // inv     1,1         reg[1] = inv(reg[1]) = 12
    compare_32b(alu_result_tb_o, 32'd12);


    //---------------------------------------------------------- Second part ----------------------------------------------------------
    @(posedge clk);
    #2;
    // mov     31,1        reg[31] = reg[1] = 12
    compare_34b(reg_file_r_data1_tb_o, 34'd12);

    @(posedge clk);
    #2;
    // storei  31,40       Guardar 40 en Mem[reg[31]], Mem[12]=40 
    compare_34b(reg_file_r_data1_tb_o, 34'd12);

    @(posedge clk);
    #2;
    // load    3,31,0      Cargar Mem[reg[31]] en el registro 3, reg[3]=Mem[12]=40
    compare_34b(reg_file_r_data1_tb_o, 34'd12);
    compare_32b(data_mem_r_data_tb_o, 32'd40);
    @(posedge clk);
    #2;
    // subc    11,26,1     reg[11]=reg[26]-reg[1]-reg[1].C = 2147483648 - 12   (Esto debe dar overflow)
    compare_32b(reg_file_r_data1_tb_o[31:0], 32'd2147483648);
    compare_32b(reg_file_r_data2_tb_o[31:0], 32'd12);
    compare_1b(alu_V_flag_tb_o, 1'b1);
    /////


    @(posedge clk);
    #2;
    // bv      30,11,8     Si hay overflow en Rc PC = PC + reg[30] + 8 = PC + 8
    compare_32b(reg_file_r_data1_tb_o[31:0], 32'd0);
    compare_32b(alu_result_tb_o, 32'd8);
    // or      11,0,1      Instruccion salteada

    @(posedge clk);
    #2;
    // or      11,0,1      reg[11] = reg[0] OR reg[1] = 50 OR 12 = 62
    compare_32b(alu_result_tb_o, 32'd62);

    @(posedge clk);
    #2;
    // and     12,11,0     reg[12] = reg[11] AND reg[0] = 62 AND 12 = 12
    compare_32b(alu_result_tb_o, 32'd12);

    @(posedge clk);
    #2;
    // addc     13,11,12    reg[13] = reg[11] + reg[12] + reg[12].C = 62 + 12 + 0 = 74
    compare_32b(alu_result_tb_o, 32'd74);

    @(posedge clk);
    #2;
    // sub    13,11,12    reg[14] = reg[11] - reg[12] = 62 - 12 = 50
    compare_32b(alu_result_tb_o, 32'd50);

    @(posedge clk);
    #2;
    // bz      12,13,4     If(reg[13] == 0) then PC = PC + reg[12] + 4 = PC + 16 // No va a entrar
    prev_pc = pc_tb_o;
    @(posedge clk);
    #2;
    compare_Nb(pc_tb_o, prev_pc + {{(N-5){1'b0}},5'd4});
    // bz      12,17,4     If(reg[14] == 0) then PC = PC + reg[12] + 4 = PC + 16 // Si va a entrar
    prev_pc = pc_tb_o;
    @(posedge clk);
    #2;
    compare_Nb(pc_tb_o, prev_pc + {{(N-5){1'b0}},5'd16});
    // add     3,2,1       Instruccion salteada
    // store   0,3,0       Instruccion salteada
    // add     3,2,1       Instruccion salteada
    // bnz      12,14,4     If(reg[14] == 0) then PC = PC + reg[12] + 4 = PC + 16 // Si va a entrar
    prev_pc = pc_tb_o;
    @(posedge clk);
    #2;
    compare_Nb(pc_tb_o, prev_pc + {{(N-5){1'b0}},5'd16});
    // add     3,2,1       Instruccion salteada
    // store   0,3,0       Instruccion salteada
    // add     3,2,1       Instruccion salteada
    // jal     13,8        reg[13]=PC+4 --- PC = PC + 8
    prev_pc = pc_tb_o;
    @(posedge clk);
    #2;
    compare_Nb(pc_tb_o, prev_pc + {{(N-4){1'b0}},4'd8});
    // add     3,2,1       Instruccion salteada
    // add     13,13,1     reg[13] = reg[13] + reg[1] = PC + 8
    compare_32b(alu_result_tb_o, pc_tb_o + {{(N-4){1'b0}},4'd8});

    @(posedge clk);
    #2;
    // ret     13          PC = reg[13] = PC + 4
    prev_pc = pc_tb_o;

    @(posedge clk);
    #2;
    compare_Nb(pc_tb_o, prev_pc + {{(N-4){1'b0}},4'd4});
    // jral    13,1,4      reg[13]=PC+4 --- PC = PC + reg[1] + 4 = PC + 16 // saltea derecho al stop
    prev_pc = pc_tb_o;
    @(posedge clk);
    #2;
    compare_Nb(pc_tb_o, prev_pc + {{(N-5){1'b0}},5'd16});
    // add     3,2,1       Instruccion salteada
    // store   0,3,0       Instruccion salteada
    // add     3,2,1       Instruccion salteada
    // stop
    
    $display("// ----------------------------------");
    $display("// TEST PASSED.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule