`include "top_module.sv"

module RISC_oneCycle_tb #(parameter N = 10, parameter M = 1024)  (
);
  // ----------------------------------
  // Stimulus signals.
  // ----------------------------------
  reg      	    	clk;
  reg      	    	rst_n;



  // Output signals.
  wire      [2:0]     exception_flags_tb_o;
  wire      [31:0]    alu_result_tb_o;
  wire      [33:0]    reg_file_r_data1_tb_o;
  wire      [33:0]    reg_file_r_data2_tb_o;
  wire      [31:0]    data_mem_r_data_tb_o;
  wire      [N-1:0]   pc_tb_o;
  wire      [31:0]    inst_read_data_tb_o;
  wire                alu_C_flag_tb_o;
  wire                alu_V_flag_tb_o;
 
  // Testbench signals.
  integer             watchdog;
  
  reg [N-1:0]         prev_pc;
  reg [N-1:0]         next_pc;

  reg [1:0]           T_bits;
  reg [2:0]           OPC_bits;
  reg [11:0]          imm_bits;
  reg [4:0]           Rc_bits;
  reg [4:0]           Rb_bits;
  reg [4:0]           Ra_bits;
  reg [1:0]           prev_T_bits;

  reg [31:0]          extend;
  reg [31:0]          Rb_data;
  reg                 Rb_C_flag;
  reg                 Rb_V_flag;
  reg [31:0]          Rc_data;
  reg                 Rc_C_flag;
  reg                 Rc_V_flag;

  reg [N-1:0]         data_mem_address;

  reg [0:7]           data_mem        [0:M-1];
  reg [31:0]          mem_exp_o;
  reg [0:33]          reg_mem         [0:31];
  reg [0:7]           inst_mem        [0:M-1];
  reg [4:0]           instruction_set [19:0];



  //Files
  parameter   MEM_INIT_FILE   = "data_memory.hex"; // Path to the file with the initial memory values   
  parameter   REG_INIT_FILE   = "reg_memory.bin"; // Path to the file with the initial reg file values   
  parameter   INST_INIT_FILE   = "output.bin"; // Path to the file with the initial memory values   


  
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

  //Stop signal
  localparam STOP = 3'b111; // Belongs to movement instructions


  //initialize intruction memory with random instructions
  initial begin
    instruction_set[0] = {MOVEMENT,LOAD};
    instruction_set[1] = {MOVEMENT,STORE};
    instruction_set[2] = {MOVEMENT,LOADI};
    instruction_set[3] = {MOVEMENT,STOREI};
    instruction_set[4] = {MOVEMENT,MOV};
    instruction_set[5] = {LOGIC,OR};
    instruction_set[6] = {LOGIC,INV};
    instruction_set[7] = {LOGIC,AND};
    instruction_set[8] = {ARITHMETIC,ADD};
    instruction_set[9] = {ARITHMETIC,SUB};
    instruction_set[10] = {ARITHMETIC,ADDC};
    instruction_set[11] = {ARITHMETIC,SUBC};
    instruction_set[12] = {FLOW_CONTROL,JUMP};
    instruction_set[13] = {FLOW_CONTROL,BZ};
    instruction_set[14] = {FLOW_CONTROL,BNZ};
    instruction_set[15] = {FLOW_CONTROL,BC};
    instruction_set[16] = {FLOW_CONTROL,BV};
    instruction_set[17] = {FLOW_CONTROL,JAL};
    instruction_set[18] = {FLOW_CONTROL,JRAL};
    instruction_set[19] = {FLOW_CONTROL,RET};
    
    for (int i = 0; i < M; i=i+4) begin
      inst_mem[i][0:4] = instruction_set[$urandom_range(19,0)];
      inst_mem[i][5:7] = $urandom;
      std::randomize(inst_mem[i+1]);
      std::randomize(inst_mem[i+2]);
      std::randomize(inst_mem[i+3]);
      inst_mem[i+1][7] = 1'b0; 
      inst_mem[i+2][0] = 1'b0; // 2 LSB of imm are 0 in order to have valid mem/reg access 
      inst_mem[i][6:7] = 2'b00; 
      inst_mem[i+1][0:3] = 4'b0000; // limit the size of the immediate value
      inst_mem[i+2][4:5] = 2'b00; // 2 LSB of Rc are 0 in order to have valid mem/reg access 
    end

    if (REG_INIT_FILE != "") begin
      $writememb(INST_INIT_FILE, inst_mem);
    end
  end

  //initialize register file memory and data mem with random values
  initial begin
    for (int i = 0; i < 32; i++) begin
      std::randomize(reg_mem[i]);
      reg_mem[i][32:33] = 2'b00;
    end
    for (int i = 0; i < M; i++) begin
      data_mem[i] = $urandom;
      data_mem[i][32:33] = 2'b00;
    end

    if (REG_INIT_FILE != "") begin
      $writememb(REG_INIT_FILE, reg_mem);
    end
    if (MEM_INIT_FILE != "") begin
      $writememh(MEM_INIT_FILE, data_mem);
    end 
  end

  //initialize register file memory and reg mem with preloaded values

  // initial begin
  //   if (REG_INIT_FILE != "") begin
  //     $readmemb(REG_INIT_FILE, reg_mem);
  //   end
  //   if (MEM_INIT_FILE != "") begin
  //     $readmemh(MEM_INIT_FILE, data_mem);
  //   end 
  // end


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
    // task : compare_xb
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
  
  task predictor(
    input      [2:0]exception_flags_tb_o,
    input      [31:0]alu_result_tb_o,
    input      [33:0]reg_file_r_data1_tb_o,
    input      [33:0]reg_file_r_data2_tb_o,
    input      [31:0]data_mem_r_data_tb_o,
    input      [N-1:0]pc_tb_o,
    input      [31:0]inst_read_data_tb_o,
    input      alu_C_flag_tb_o,
    input      alu_V_flag_tb_o
  );
    
    T_bits    = inst_read_data_tb_o[31:30];
    OPC_bits  = inst_read_data_tb_o[29:27];
    imm_bits  = inst_read_data_tb_o[26:15];
    Rc_bits   = inst_read_data_tb_o[14:10];
    Rb_bits   = inst_read_data_tb_o[9:5];
    Ra_bits   = inst_read_data_tb_o[4:0];

    //Finish test if exception
    if(exception_flags_tb_o != 3'b000) begin
      $display("// ----------------------------------");
      $display("// TEST PASSED WITH EXCEPTION. ");
      $display("// At time %0d Instruction: T=%b OPC=%b exceptions=%b",$time, T_bits, OPC_bits,exception_flags_tb_o);
      $display("// ----------------------------------");
      $finish;
    end

    //Get current data from registers and ALU
    {Rb_V_flag, Rb_C_flag, Rb_data} = reg_file_r_data1_tb_o;
    {Rc_V_flag, Rc_C_flag, Rc_data} = reg_file_r_data2_tb_o;
    data_mem_address = alu_result_tb_o[N-1:0];

    $display("// ----------------------------------");
    $display("At time %0d Instruction: T=%b OPC=%b Imm=%d Rc=%d Rb=%d Ra=%d",$time, T_bits, OPC_bits, imm_bits, Rc_bits, Rb_bits, Ra_bits);
    $display("Alu_o=%h Rb_data=%h Rc_data=%h",alu_result_tb_o, Rb_data, Rc_data);
    $display("// ----------------------------------");

    //Compare outputs with the testbench register memory
    compare_34b(reg_file_r_data1_tb_o, reg_mem[Rb_bits]);
    compare_34b(reg_file_r_data2_tb_o, reg_mem[Rc_bits]);

    // Chech if program counter changed to the expected value (next_pc)
    if(prev_T_bits == FLOW_CONTROL) begin
      compare_Nb(pc_tb_o, next_pc);
    end

    prev_T_bits = T_bits;

    // memory expected output
    mem_exp_o = {data_mem[data_mem_address], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd1}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd2}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd3}]};
    
    case(T_bits)
      MOVEMENT: begin
        case(OPC_bits)
          LOAD: begin
            extend           = {{15{imm_bits[11]}}, imm_bits, Rc_bits};
            reg_mem[Ra_bits] = {1'b0, 1'b0, mem_exp_o};

            compare_32b(alu_result_tb_o, Rb_data + extend);
            compare_32b(data_mem_r_data_tb_o, mem_exp_o);
          end
          STORE: begin
            extend           = {{20{imm_bits[11]}}, imm_bits};
            {data_mem[data_mem_address], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd1}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd2}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd3}]} = Rc_data;

            compare_32b(alu_result_tb_o, Rb_data + extend);
          end
          LOADI: begin
            extend           = {{15{imm_bits[11]}}, imm_bits, Rc_bits};
            reg_mem[Ra_bits] = {1'b0, 1'b0, extend};

          end
          STOREI: begin
            extend           = {{15{imm_bits[11]}}, imm_bits, Rc_bits};
            {data_mem[data_mem_address], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd1}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd2}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd3}]} = extend;

            compare_32b(alu_result_tb_o, Rb_data);
          end
          MOV: begin
            reg_mem[Ra_bits] = {Rb_V_flag, Rb_C_flag, Rb_data};
          end
          STOP: begin //Program finished
            $display("// ----------------------------------");
            $display("// TEST PASSED.");
            $display("// ----------------------------------");
            $finish;
          end
          default:
            compare_exceptions(exception_flags_tb_o, 3'b100);
        endcase
      end
      LOGIC: begin
        case(OPC_bits)
          OR: begin
            reg_mem[Ra_bits] = {1'b0, 1'b0, Rb_data | Rc_data};
          end
          INV: begin
            reg_mem[Ra_bits] = {1'b0, 1'b0, ~Rb_data};
          end
          AND: begin
            reg_mem[Ra_bits] = {1'b0, 1'b0, Rb_data & Rc_data};
          end
          default:
            compare_exceptions(exception_flags_tb_o, 3'b100);
          
        endcase
        compare_32b(alu_result_tb_o,  reg_mem[Ra_bits][2:33]);
        compare_1b(alu_C_flag_tb_o,   reg_mem[Ra_bits][1]);
        compare_1b(alu_V_flag_tb_o,   reg_mem[Ra_bits][0]);
      end
      ARITHMETIC: begin

        case(OPC_bits) 
          ADD: begin
            reg_mem[Ra_bits] = {2'b00,Rb_data}+{2'b00,Rc_data};
            reg_mem[Ra_bits][0] = (Rb_data[31]==Rc_data[31]) & (Rb_data[31] ^ reg_mem[Ra_bits][2]);
          end

          SUB: begin
            reg_mem[Ra_bits] = {2'b00,Rb_data}-{2'b00,Rc_data};
            reg_mem[Ra_bits][0] = (Rb_data[31]!=Rc_data[31]) & (Rb_data[31] ^ reg_mem[Ra_bits][2]);
          end

          ADDC: begin
            reg_mem[Ra_bits] = {2'b00,Rb_data}+{2'b00, {31'd0, Rc_C_flag} + Rc_data};
            reg_mem[Ra_bits][0] = (Rb_data[31]=={{31'd0, Rc_C_flag} + Rc_data}[31]) & (Rb_data[31] ^ reg_mem[Ra_bits][2]);
          end

          SUBC: begin
            reg_mem[Ra_bits] = {2'b00,Rb_data}-{2'b00, {31'd0, Rc_C_flag} + Rc_data};
            reg_mem[Ra_bits][0] = (Rb_data[31]!={{31'd0, Rc_C_flag} + Rc_data}[31]) & (Rb_data[31] ^ reg_mem[Ra_bits][2]);
          end  
          default:
            compare_exceptions(exception_flags_tb_o, 3'b100);
          
        endcase
        
        compare_32b(alu_result_tb_o,  reg_mem[Ra_bits][2:33]);
        compare_1b(alu_C_flag_tb_o,   reg_mem[Ra_bits][1]);
        compare_1b(alu_V_flag_tb_o,   reg_mem[Ra_bits][0]);
        
      end
      FLOW_CONTROL: begin
        case(OPC_bits)
          JUMP: begin
            next_pc = pc_tb_o + Rb_data;
          end
          BZ: begin
            if(Rc_data == 32'd0) begin
              extend           = {{20{imm_bits[11]}}, imm_bits};
              next_pc = pc_tb_o + Rb_data + extend;
            end
            else begin
              next_pc = pc_tb_o + 4;
            end
          end
          BNZ: begin
            if(Rc_data != 32'd0) begin
              extend           = {{20{imm_bits[11]}}, imm_bits};
              next_pc = pc_tb_o + Rb_data + extend;
            end
            else begin
              next_pc = pc_tb_o + 4;
            end
          end
          BC: begin
            if(Rc_C_flag == 1'b1) begin
              extend           = {{20{imm_bits[11]}}, imm_bits};
              next_pc = pc_tb_o + Rb_data + extend;
            end
            else begin
              next_pc = pc_tb_o + 4;
            end
          end
          BV: begin
            if(Rb_V_flag == 1'b1) begin
              extend           = {{20{imm_bits[11]}}, imm_bits};
              next_pc = pc_tb_o + Rb_data + extend;
            end
            else begin
              next_pc = pc_tb_o + 4;
            end
          end
          JAL: begin
            extend           = {{15{imm_bits[11]}}, imm_bits, Rc_bits};
            reg_mem[Ra_bits] = {1'b0, 1'b0, pc_tb_o + 4};
            next_pc = pc_tb_o + extend;
          end
          JRAL: begin
            extend           = {{15{imm_bits[11]}}, imm_bits, Rc_bits};
            reg_mem[Ra_bits] = {1'b0, 1'b0, pc_tb_o + 4};
            next_pc = pc_tb_o + Rb_data + extend;
          end
          RET: begin
            next_pc = Rb_data;
          end
          default:
            compare_exceptions(exception_flags_tb_o, 3'b100);
        endcase
      end
      default:
        compare_exceptions(exception_flags_tb_o, 3'b100);
    endcase
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

    // Initialize signals.
    prev_T_bits = 2'b00;
    prev_pc = {N{1'b0}};
    next_pc = {N{1'b0}};

    //reset
    rst_n = 1'b1;
    #2;
    rst_n = 1'b0;
    #2;
    rst_n = 1'b1;

    watchdog = 0;
    while (watchdog < 100) begin // Watchdog to avoid infinite loops
      watchdog = watchdog + 1;
      @ (posedge clk);
      #2;
      predictor(exception_flags_tb_o, alu_result_tb_o, reg_file_r_data1_tb_o, reg_file_r_data2_tb_o, data_mem_r_data_tb_o, pc_tb_o, inst_read_data_tb_o, alu_C_flag_tb_o, alu_V_flag_tb_o);
    end
    
    $display("// ----------------------------------");
    $display("// TEST FINISHED BY WATCHDOG.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule