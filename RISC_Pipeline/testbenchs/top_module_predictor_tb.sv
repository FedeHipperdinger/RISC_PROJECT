`include "top_module.sv"

module RISC_pipeline_tb #(parameter N = 10, parameter M = 1024)  (
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
  wire      stallF_tb_o;
  wire      stallD_tb_o;
  wire      flushD_tb_o;
  wire      flushE_tb_o;
  wire      [1:0]bypA_sel_tb_o;
  wire      [1:0]bypB_sel_tb_o;
  wire      [31:0]writeback_data_tb_o;

  // Testbench signals.
  integer watchdog;
  
  reg [N-1:0]prev_pc;
  reg [N-1:0]next_pc_1;
  reg [N-1:0]next_pc_2;
  reg [N-1:0]next_pc_3;
  reg [N-1:0]next_pc_aux;
  reg npc1_flag;
  reg npc2_flag;
  reg npc3_flag;
  reg pc_changed_flag;
  reg [4:0]pc_changed_mem;

  reg [1:0]T_bits;
  reg [2:0]OPC_bits;
  reg [11:0]imm_bits;
  reg [4:0]Rc_bits;
  reg [4:0]Rb_bits;
  reg [4:0]Ra_bits;

  reg [1:0]prev_T_bits;
  reg [2:0]prev_OPC_bits;

  reg [31:0]extend;
  reg [31:0]Rb_data;
  reg Rb_C_flag;
  reg Rb_V_flag;
  reg [31:0]Rc_data;
  reg Rc_C_flag;
  reg Rc_V_flag;

  reg [N-1:0]data_mem_address;

  reg [0:7]data_mem[0:M-1];
  reg [31:0]mem_exp_o;
  reg [0:33]reg_mem[0:31];
  reg [0:7]inst_mem[0:M-1];
  reg [4:0]instruction_set[19:0];

  reg [31:0]pipeline_status[4:0];

  reg prev_stall;
  reg prev_flushE;
  reg prev_flushD;


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
  localparam NOP              = 3'b111; // No operation

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
      if((i-1)%4 == 0)
        data_mem[i][6:7] = 2'b00;
    end

    if (REG_INIT_FILE != "") begin
      $writememb(REG_INIT_FILE, reg_mem);
    end
    if (MEM_INIT_FILE != "") begin
      $writememh(MEM_INIT_FILE, data_mem);
    end 
  end


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
    .alu_V_flag_o(alu_V_flag_tb_o),
    .stallF_o(stallF_tb_o),
    .stallD_o(stallD_tb_o),
    .flushD_o(flushD_tb_o),
    .flushE_o(flushE_tb_o),
    .bypA_sel_o(bypA_sel_tb_o),
    .bypB_sel_o(bypB_sel_tb_o),
    .writeback_data_o(writeback_data_tb_o)

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
  
  task predictor_T0(
    input      [31:0]inst_at_stage_tb,
    input      [2:0]exception_flags_tb_o,
    input      [31:0]alu_result_tb_o,
    input      [33:0]reg_file_r_data1_tb_o,
    input      [33:0]reg_file_r_data2_tb_o,
    input      [31:0]data_mem_r_data_tb_o,
    input      [N-1:0]pc_tb_o,
    input      alu_C_flag_tb_o,
    input      alu_V_flag_tb_o,
    input      stallF_tb_o,
    input      stallD_tb_o,
    input      flushD_tb_o,
    input      flushE_tb_o,
    input      [33:0]writeback_data_tb_o
  );
    
    T_bits    = inst_at_stage_tb[31:30];
    OPC_bits  = inst_at_stage_tb[29:27];
    imm_bits  = inst_at_stage_tb[26:15];
    Rc_bits   = inst_at_stage_tb[14:10];
    Rb_bits   = inst_at_stage_tb[9:5];
    Ra_bits   = inst_at_stage_tb[4:0];

    if(exception_flags_tb_o != 3'b000) begin
      $display("// ----------------------------------");
      $display("// TEST PASSED WITH EXCEPTION. ");
      $display("// At time %0d Instruction: T=%b OPC=%b exceptions=%b",$time, T_bits, OPC_bits,exception_flags_tb_o);
      $display("// ----------------------------------");
      $finish;
    end

    // {Rb_V_flag, Rb_C_flag, Rb_data} = reg_file_r_data1_tb_o;
    // {Rc_V_flag, Rc_C_flag, Rc_data} = reg_file_r_data2_tb_o;
    // data_mem_address = alu_result_tb_o[N-1:0];

    $display("// ----------------------------------");
    $display("Instruction at T0: T=%b OPC=%b Imm=%d Rc=%d Rb=%d Ra=%d", T_bits, OPC_bits, imm_bits, Rc_bits, Rb_bits, Ra_bits);
    $display("// ----------------------------------");


    case(T_bits)
      MOVEMENT: begin
        case(OPC_bits)
          LOAD: begin
            compare_1b(stallF_tb_o, 1'b1);
            compare_1b(stallD_tb_o, 1'b1);
          end
          STOP: begin
            $display("// ----------------------------------");
            $display("// TEST PASSED.");
            $display("// ----------------------------------");
            $finish;
          end
          default:
            if(OPC_bits != STORE && OPC_bits != LOADI && OPC_bits != STOREI && OPC_bits != MOV) begin
              compare_exceptions(exception_flags_tb_o, 3'b100);
            end
        endcase
      end
      LOGIC: begin
        if(OPC_bits != OR && OPC_bits != INV && OPC_bits != AND) begin
          compare_exceptions(exception_flags_tb_o, 3'b100);
        end
      end
      ARITHMETIC: begin
        if(OPC_bits != ADD && OPC_bits != SUB && OPC_bits != ADDC && OPC_bits != SUBC && OPC_bits != NOP) begin
          compare_exceptions(exception_flags_tb_o, 3'b100);
        end
      end      
      FLOW_CONTROL: begin
        if(OPC_bits == JUMP || OPC_bits == JAL || OPC_bits == JRAL || OPC_bits == RET) begin
          compare_1b(stallD_tb_o, 1'b1);
          compare_1b(flushD_tb_o, 1'b1);
        end
        else begin
          if(OPC_bits != BZ && OPC_bits != BNZ && OPC_bits != BC && OPC_bits != BV) begin
            compare_exceptions(exception_flags_tb_o, 3'b100);
          end

        end
      end
      default:
        compare_exceptions(exception_flags_tb_o, 3'b100);
    endcase
  endtask

  task predictor_T1(
    input      [31:0]inst_at_stage_tb,
    input      [2:0]exception_flags_tb_o,
    input      [31:0]alu_result_tb_o,
    input      [33:0]reg_file_r_data1_tb_o,
    input      [33:0]reg_file_r_data2_tb_o,
    input      [31:0]data_mem_r_data_tb_o,
    input      [N-1:0]pc_tb_o,
    input      alu_C_flag_tb_o,
    input      alu_V_flag_tb_o,
    input      stallF_tb_o,
    input      stallD_tb_o,
    input      flushD_tb_o,
    input      flushE_tb_o,
    input      [33:0]writeback_data_tb_o
  );
    
    T_bits    = inst_at_stage_tb[31:30];
    OPC_bits  = inst_at_stage_tb[29:27];
    imm_bits  = inst_at_stage_tb[26:15];
    Rc_bits   = inst_at_stage_tb[14:10];
    Rb_bits   = inst_at_stage_tb[9:5];
    Ra_bits   = inst_at_stage_tb[4:0];

    

    {Rb_V_flag, Rb_C_flag, Rb_data} = reg_mem[Rb_bits]; // Could be different from reg_file_r_data1_tb_o if bypass is needed
    {Rc_V_flag, Rc_C_flag, Rc_data} = reg_mem[Rc_bits];

    $display("// ----------------------------------");
    $display("Instruction at T1: T=%b OPC=%b Imm=%d Rc=%d Rb=%d Ra=%d", T_bits, OPC_bits, imm_bits, Rc_bits, Rb_bits, Ra_bits);
    $display("Alu_o=%h Rb_data=%h Rc_data=%h Rb_C=%b Rb_V=%b Rc_C=%b Rc_V=%b",alu_result_tb_o, Rb_data, Rc_data, Rb_C_flag, Rb_V_flag, Rc_C_flag, Rc_V_flag);
    $display("// ----------------------------------");


    data_mem_address = alu_result_tb_o[N-1:0];
    mem_exp_o = {data_mem[data_mem_address], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd1}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd2}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd3}]};
    
    if(prev_flushE == 1'b0) begin
      case(T_bits)
        MOVEMENT: begin
          case(OPC_bits)
            LOAD: begin
              extend           = {{15{imm_bits[11]}}, imm_bits, Rc_bits};
              reg_mem[Ra_bits] = {1'b0, 1'b0, mem_exp_o};

              compare_32b(alu_result_tb_o, Rb_data + extend);
              compare_1b(flushD_tb_o, 1'b1);
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

              compare_32b(alu_result_tb_o, Rb_data);
            end
            STOP: begin
              $display("// ----------------------------------");
              $display("// TEST PASSED.");
              $display("// ----------------------------------");
              $finish;
            end
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
            
          endcase
          if(OPC_bits != NOP) begin
            compare_32b(alu_result_tb_o,  reg_mem[Ra_bits][2:33]);
            compare_1b(alu_C_flag_tb_o,   reg_mem[Ra_bits][1]);
            compare_1b(alu_V_flag_tb_o,   reg_mem[Ra_bits][0]);
          end
          
        end
        FLOW_CONTROL: begin
          case(OPC_bits)
            JUMP: begin
              next_pc_aux = prev_pc + Rb_data;
              compare_32b(alu_result_tb_o,  Rb_data);
              pc_changed_mem[0] = 1'b1;

            end
            BZ: begin
              if(Rc_data == 32'd0) begin
                extend           = {{20{imm_bits[11]}}, imm_bits};
                next_pc_aux = prev_pc + Rb_data + extend;
                compare_32b(alu_result_tb_o,  Rb_data + extend);
                compare_1b(flushD_tb_o, 1'b1);
                compare_1b(flushE_tb_o, 1'b1);
                pc_changed_mem[0] = 1'b1;
              end
            end
            BNZ: begin
              if(Rc_data != 32'd0) begin
                extend           = {{20{imm_bits[11]}}, imm_bits};
                next_pc_aux = prev_pc + Rb_data + extend;
                compare_32b(alu_result_tb_o,  Rb_data + extend);
                compare_1b(flushD_tb_o, 1'b1);
                compare_1b(flushE_tb_o, 1'b1);
                pc_changed_mem[0] = 1'b1;
              end

            end
            BC: begin
              if(Rc_C_flag == 1'b1) begin
                extend           = {{20{imm_bits[11]}}, imm_bits};
                next_pc_aux = prev_pc + Rb_data + extend;
                compare_32b(alu_result_tb_o,  Rb_data + extend);
                compare_1b(flushD_tb_o, 1'b1);
                compare_1b(flushE_tb_o, 1'b1);
                pc_changed_mem[0] = 1'b1;
              end
            end
            BV: begin
              if(Rc_V_flag == 1'b1) begin
                extend           = {{20{imm_bits[11]}}, imm_bits};
                next_pc_aux = prev_pc + Rb_data + extend;
                compare_32b(alu_result_tb_o,  Rb_data + extend);
                compare_1b(flushD_tb_o, 1'b1);
                compare_1b(flushE_tb_o, 1'b1);
                pc_changed_mem[0] = 1'b1;
              end
            end
            JAL: begin
              extend           = {{15{imm_bits[11]}}, imm_bits, Rc_bits};
              reg_mem[Ra_bits] = {1'b0, 1'b0, prev_pc + 4};
              next_pc_aux = prev_pc + extend;
              compare_32b(alu_result_tb_o, extend);
              pc_changed_mem[0] = 1'b1;
            end
            JRAL: begin
              extend           = {{15{imm_bits[11]}}, imm_bits, Rc_bits};
              reg_mem[Ra_bits] = {1'b0, 1'b0, prev_pc + 4};
              next_pc_aux = prev_pc + Rb_data + extend;
              compare_32b(alu_result_tb_o,  Rb_data + extend);
              pc_changed_mem[0] = 1'b1;
            end
            RET: begin
              next_pc_aux = Rb_data;
              compare_32b(alu_result_tb_o,  Rb_data);
              pc_changed_mem[0] = 1'b1;
            end
          endcase
            if(npc1_flag) begin
              if(npc2_flag) begin
                next_pc_3 = next_pc_aux;
                npc3_flag = 1'b1;
              end
              else begin
                next_pc_2 = next_pc_aux;
                npc2_flag = 1'b1;
              end
            end
            else begin
              next_pc_1 = next_pc_aux;
              npc1_flag = 1'b1;
            end
          end
      endcase
    end
  endtask

  task predictor_T2(
    input      [31:0]inst_at_stage_tb,
    input      [2:0]exception_flags_tb_o,
    input      [31:0]alu_result_tb_o,
    input      [33:0]reg_file_r_data1_tb_o,
    input      [33:0]reg_file_r_data2_tb_o,
    input      [31:0]data_mem_r_data_tb_o,
    input      [N-1:0]pc_tb_o,
    input      alu_C_flag_tb_o,
    input      alu_V_flag_tb_o,
    input      stallF_tb_o,
    input      stallD_tb_o,
    input      flushD_tb_o,
    input      flushE_tb_o,
    input      [33:0]writeback_data_tb_o
  );
    
    T_bits    = inst_at_stage_tb[31:30];
    OPC_bits  = inst_at_stage_tb[29:27];
    imm_bits  = inst_at_stage_tb[26:15];
    Rc_bits   = inst_at_stage_tb[14:10];
    Rb_bits   = inst_at_stage_tb[9:5];
    Ra_bits   = inst_at_stage_tb[4:0];


    $display("// ----------------------------------");
    $display("Instruction at T2: T=%b OPC=%b Imm=%d Rc=%d Rb=%d Ra=%d", T_bits, OPC_bits, imm_bits, Rc_bits, Rb_bits, Ra_bits);
    $display("// ----------------------------------");

    {Rb_V_flag, Rb_C_flag, Rb_data} = reg_mem[Rb_bits]; // Could be different from reg_file_r_data1_tb_o if bypass is needed
    {Rc_V_flag, Rc_C_flag, Rc_data} = reg_mem[Rc_bits];
    mem_exp_o = {data_mem[data_mem_address], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd1}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd2}], data_mem[data_mem_address + {{N-2{1'b0}}, 2'd3}]};
    
    case(T_bits)
      MOVEMENT: begin
        case(OPC_bits)
          LOAD: begin
            compare_32b(data_mem_r_data_tb_o, mem_exp_o);
            compare_1b(flushD_tb_o, 1'b1);
            compare_1b(flushE_tb_o, 1'b1);
          end
        endcase
      end
    endcase
  endtask

  task predictor_T3(
    input      [31:0]inst_at_stage_tb,
    input      [2:0]exception_flags_tb_o,
    input      [31:0]alu_result_tb_o,
    input      [33:0]reg_file_r_data1_tb_o,
    input      [33:0]reg_file_r_data2_tb_o,
    input      [31:0]data_mem_r_data_tb_o,
    input      [N-1:0]pc_tb_o,
    input      alu_C_flag_tb_o,
    input      alu_V_flag_tb_o,
    input      stallF_tb_o,
    input      stallD_tb_o,
    input      flushD_tb_o,
    input      flushE_tb_o,
    input      [33:0]writeback_data_tb_o
  );
    
    T_bits    = inst_at_stage_tb[31:30];
    OPC_bits  = inst_at_stage_tb[29:27];
    imm_bits  = inst_at_stage_tb[26:15];
    Rc_bits   = inst_at_stage_tb[14:10];
    Rb_bits   = inst_at_stage_tb[9:5];
    Ra_bits   = inst_at_stage_tb[4:0];


    $display("// ----------------------------------");
    $display("Instruction at T3: T=%b OPC=%b Imm=%d Rc=%d Rb=%d Ra=%d", T_bits, OPC_bits, imm_bits, Rc_bits, Rb_bits, Ra_bits);
    $display("// ----------------------------------");

    if(prev_T_bits == FLOW_CONTROL && pc_changed_mem[2]) begin
          compare_Nb(prev_pc, next_pc_1);
          if(npc2_flag) begin
            next_pc_1 = next_pc_2;
            if(npc3_flag) begin
              next_pc_2 = next_pc_3;
            end
            else begin
              npc2_flag = 1'b0;
            end
          end
          else begin
            npc1_flag = 1'b0;
          end
    end
    prev_T_bits = T_bits;
    prev_OPC_bits = OPC_bits;
    
    case(T_bits)
      MOVEMENT: begin
        case(OPC_bits)
          LOAD: begin
            compare_32b(writeback_data_tb_o, reg_mem[Ra_bits][2:33]);
          end
          LOADI: begin
            compare_32b(writeback_data_tb_o, reg_mem[Ra_bits][2:33]);
          end
          MOV: begin
            compare_32b(writeback_data_tb_o, reg_mem[Ra_bits][2:33]);
          end
        endcase
      end
      LOGIC: begin
        compare_32b(writeback_data_tb_o, reg_mem[Ra_bits][2:33]);
      end
      ARITHMETIC: begin
        if(OPC_bits != NOP) begin
          compare_32b(writeback_data_tb_o, reg_mem[Ra_bits][2:33]);
        end
      end
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
    prev_OPC_bits = 3'b000;
    prev_pc = {N{1'b0}};
    next_pc_1 = {N{1'b0}};
    next_pc_2 = {N{1'b0}};
    next_pc_3 = {N{1'b0}};
    npc1_flag = 1'b0;
    npc2_flag = 1'b0;
    npc3_flag = 1'b0;
    pc_changed_flag = 1'b0;
    pipeline_status = {{ARITHMETIC, NOP, 27'd0}, {ARITHMETIC, NOP, 27'd0}, {ARITHMETIC, NOP, 27'd0}, {ARITHMETIC, NOP, 27'd0}, {ARITHMETIC, NOP, 27'd0}}; 
    pc_changed_mem  = {5{1'b0}};
    //reset
    rst_n = 1'b1;
    #2;
    rst_n = 1'b0;
    #2;
    rst_n = 1'b1;
    watchdog = 0;

    @ (posedge clk);
      #2;

    while (watchdog < 100) begin
      watchdog = watchdog + 1;
      @ (posedge clk);
      #2;
      pc_changed_mem = pc_changed_mem << 1;
      if(prev_stall == 1'b1 && stallD_tb_o == 1'b1) begin 
        pipeline_status = {pipeline_status[3:0], {ARITHMETIC, NOP, 27'd0}};
      end
      else begin
        pipeline_status = {pipeline_status[3:0], inst_read_data_tb_o};
      end
      prev_stall = stallD_tb_o;
      if(prev_flushD) begin
        pipeline_status = {pipeline_status[4:1], {ARITHMETIC, NOP, 27'd0}};
      end
      if(prev_flushE) begin
        pc_changed_mem[1] = 1'b0;
        pipeline_status = {pipeline_status[4:2], {ARITHMETIC, NOP, 27'd0},pipeline_status[0]};
      end

      predictor_T3(pipeline_status[3], exception_flags_tb_o, alu_result_tb_o, reg_file_r_data1_tb_o, reg_file_r_data2_tb_o, data_mem_r_data_tb_o, pc_tb_o, alu_C_flag_tb_o, alu_V_flag_tb_o, stallF_tb_o, stallD_tb_o, flushD_tb_o, flushE_tb_o, writeback_data_tb_o);
      predictor_T2(pipeline_status[2], exception_flags_tb_o, alu_result_tb_o, reg_file_r_data1_tb_o, reg_file_r_data2_tb_o, data_mem_r_data_tb_o, pc_tb_o, alu_C_flag_tb_o, alu_V_flag_tb_o, stallF_tb_o, stallD_tb_o, flushD_tb_o, flushE_tb_o, writeback_data_tb_o);
      predictor_T1(pipeline_status[1], exception_flags_tb_o, alu_result_tb_o, reg_file_r_data1_tb_o, reg_file_r_data2_tb_o, data_mem_r_data_tb_o, pc_tb_o, alu_C_flag_tb_o, alu_V_flag_tb_o, stallF_tb_o, stallD_tb_o, flushD_tb_o, flushE_tb_o, writeback_data_tb_o);
      predictor_T0(pipeline_status[0], exception_flags_tb_o, alu_result_tb_o, reg_file_r_data1_tb_o, reg_file_r_data2_tb_o, data_mem_r_data_tb_o, pc_tb_o, alu_C_flag_tb_o, alu_V_flag_tb_o, stallF_tb_o, stallD_tb_o, flushD_tb_o, flushE_tb_o, writeback_data_tb_o);
      // In the fifth cycle nothing is done (This is because T0 is taken as the decode stage, so the fetch stage is not considered)
      prev_pc = pc_tb_o;
      prev_flushE = flushE_tb_o;
    end
    
    $display("// ----------------------------------");
    $display("// TEST FINISHED BY WATCHDOG.");
    $display("// ----------------------------------");
    $finish;
  end
  
endmodule