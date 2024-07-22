`include "control_unit_mod.sv"
`include "alu_mod.sv"
`include "register_file_mod.sv"
`include "data_memory_mod.sv"
`include "instruction_memory_mod.sv"
`include "program_counter_mod.sv"
`include "extender_mod.sv"
`include "hazard_unit_mod.sv"



module top_module #(parameter N = 10)(
    input  wire clk,
    input  wire rst_n,
    output reg [2:0]exception_flags_o,
    output reg [31:0]alu_result_o,
    output reg [33:0]reg_file_r_data1_o,
    output reg [33:0]reg_file_r_data2_o,
    output reg [31:0]data_mem_r_data_o,
    output reg [N-1:0]pc_o,
    output reg [31:0]inst_read_data_o,
    output reg alu_C_flag_o,
    output reg alu_V_flag_o,
    output reg stallF_o,
    output reg stallD_o,
    output reg flushD_o,
    output reg flushE_o,
    output reg [1:0]bypA_sel_o,
    output reg [1:0]bypB_sel_o,
    output reg [31:0]writeback_data_o

);
    // Multiplexers
    reg [31:0]pc_mux_out_F;
    reg [31:0]reg_file_mux_out_D;
    reg [33:0]alu_srcA_mux_out_E;
    reg [33:0]alu_srcB_mux_out_E;
    reg [31:0]data_mem_w_mux_out_M;
    reg [31:0]writeback_mux_out_W;
    reg [33:0]alu_srcA_hazard_mux_out_E;
    reg [33:0]alu_srcB_hazard_mux_out_E;
    reg [33:0]ctrl_unit_regi_mux_out_D;


    // Pipeline 
    // Fetch -> Decode
    reg [N-1:0]pc_F;
    reg [N-1:0]pc_D;
    reg [31:0]inst_read_data_F;
    reg [31:0]inst_read_data_D;

    // Decode -> Execute
    reg [N-1:0]pc_E;
    reg [1:0]pc_sel_D;
    reg [1:0]pc_sel_E;
    reg data_mem_w_ctrl_D;
    reg data_mem_w_ctrl_E;
    reg data_mem_w_sel_D;
    reg data_mem_w_sel_E;
    reg [2:0]alu_ctrl_D;
    reg [2:0]alu_ctrl_E;
    reg alu_srcA_sel_D;
    reg alu_srcA_sel_E;
    reg [1:0]alu_srcB_sel_D;
    reg [1:0]alu_srcB_sel_E;
    reg reg_file_w_ctrl_D;
    reg reg_file_w_ctrl_E;
    reg reg_file_w_sel_D;
    reg reg_file_w_sel_E;
    reg writeback_sel_D;
    reg writeback_sel_E;
    reg [31:0]ext_data_D;
    reg [31:0]ext_data_E;
    reg [1:0]addr_bits_D;
    reg [1:0]addr_bits_E;
    reg [33:0]reg_file_r_data1_D;
    reg [33:0]reg_file_r_data1_E;
    reg [33:0]reg_file_r_data2_D;
    reg [33:0]reg_file_r_data2_E;
    reg extend_ctrl_D;
    reg [1:0]bypA_sel_D;
    reg [1:0]bypA_sel_E;
    reg [1:0]bypB_sel_D;
    reg [1:0]bypB_sel_E;
    reg [1:0]byp_CU_sel_D;
    reg [4:0]Ra_D;
    reg [4:0]Ra_E;



    // Execute -> Memory
    reg [N-1:0]pc_M;
    reg data_mem_w_ctrl_M;
    reg data_mem_w_sel_M;
    reg reg_file_w_ctrl_M;
    reg reg_file_w_sel_M;
    reg writeback_sel_M;
    reg alu_C_flag_E;
    reg alu_C_flag_M;
    reg alu_V_flag_E;
    reg alu_V_flag_M;
    reg [31:0]alu_result_E;
    reg [31:0]alu_result_M;
    reg [31:0]ext_data_M;
    reg [4:0]Ra_M;
    reg [33:0]alu_srcB_hazard_mux_out_M;

    // Memory -> Writeback
    reg [N-1:0]pc_W;
    reg reg_file_w_ctrl_W;
    reg reg_file_w_sel_W;
    reg writeback_sel_W;
    reg [31:0]data_mem_r_data_M;
    reg [31:0]data_mem_r_data_W;
    reg alu_C_flag_W;
    reg alu_V_flag_W;
    reg [31:0]alu_result_W;
    reg [4:0]Ra_W;

    // ----------------------------------


    control_unit_mod u_control_unit_mod (
        //Inputs
      	.clk(clk),
        .rst_n(rst_n),
        .T_i(inst_read_data_D[31:30]),
        .OPC_i(inst_read_data_D[29:27]),
        .reg_i(ctrl_unit_regi_mux_out_D),
        .addr_bits_i(addr_bits_D),
        .pc_bits_i(pc_F[1:0]),
        //Outputs
        .pc_sel_o(pc_sel_D),
        .data_mem_w_ctrl_o(data_mem_w_ctrl_D),
        .data_mem_w_sel_o(data_mem_w_sel_D),
        .alu_ctrl_o(alu_ctrl_D),
        .alu_srcA_sel_o(alu_srcA_sel_D),
        .alu_srcB_sel_o(alu_srcB_sel_D),
        .reg_file_w_ctrl_o(reg_file_w_ctrl_D),
        .reg_file_w_sel_o(reg_file_w_sel_D),
        .extend_ctrl_o(extend_ctrl_D),
        .writeback_sel_o(writeback_sel_D),
        .exception_flags_o(exception_flags_o)
    );

    alu_mod u_alu_mod (
        //Inputs
        .clk(clk),
        .alu_srcA_i(alu_srcA_mux_out_E),
        .alu_srcB_i(alu_srcB_mux_out_E),
        .alu_ctrl_i(alu_ctrl_E),
        //Outputs
        .alu_result_o(alu_result_E),
        .alu_C_flag_o(alu_C_flag_E),
        .alu_V_flag_o(alu_V_flag_E)
    );

    register_file_mod u_register_file_mod (
        //Inputs
        .clk(clk),
      	.rst_n(rst_n),
        .address1_i(inst_read_data_D[9:5]),
        .address2_i(inst_read_data_D[14:10]),
        .address3_i(Ra_W),
        .write_data3_i({alu_V_flag_W, alu_C_flag_W, reg_file_mux_out_D}),
        .write_en3_i(reg_file_w_ctrl_W),
        //Outputs
        .read_data1_o(reg_file_r_data1_D),
        .read_data2_o(reg_file_r_data2_D)
    );

    data_memory_mod u_data_memory_mod (
        //Inputs
        .clk(clk),
        .rst_n(rst_n),
        .address_i(alu_result_M[N-1:0]),
        .write_data_i(data_mem_w_mux_out_M),
        .write_en_i(data_mem_w_ctrl_M),
        //Outputs
        .read_data_o(data_mem_r_data_M)
    );

    instruction_memory_mod u_instruction_memory_mod (
        //Inputs
        .clk(clk),
        .rst_n(rst_n),
        .address_i(pc_F),
        //Outputs
        .read_data_o(inst_read_data_F)
    );

    program_counter_mod u_program_counter_mod (
        //Inputs
        .clk(clk),
        .rst_n(rst_n),
        .enable_i(~stallF_o),
        .next_pc_i(pc_mux_out_F),
        //Outputs
        .pc_o(pc_F)
    );

    extender_mod u_extender_mod (
        //Inputs
        .ext_ctrl_i(extend_ctrl_D),
        .dataA_i(inst_read_data_D[26:15]),
        .dataB_i(inst_read_data_D[14:10]),
        //Outputs
        .ext_data_o(ext_data_D)
    );

    hazard_unit_mod u_hazard_unit_mod (
        //Inputs
        .clk(clk),
        .rst_n(rst_n),
        .instr_i(inst_read_data_D),
        .pc_selD_i(pc_sel_D),
        .pc_selE_i(pc_sel_E),
        //Outputs
        .stallF_o(stallF_o),
        .stallD_o(stallD_o),
        .flushD_o(flushD_o),
        .flushE_o(flushE_o),
        .bypA_sel_o(bypA_sel_D),
        .bypB_sel_o(bypB_sel_D),
        .byp_CU_sel_o(byp_CU_sel_D)
    );



    // ----------------------------------


    // Outputs
    assign alu_result_o         = alu_result_E;
    assign reg_file_r_data1_o   = reg_file_r_data1_D;
    assign reg_file_r_data2_o   = reg_file_r_data2_D;
    assign data_mem_r_data_o    = data_mem_r_data_M;
    assign pc_o                 = pc_D;
    assign inst_read_data_o     = inst_read_data_D;
    assign alu_C_flag_o         = alu_C_flag_E;
    assign alu_V_flag_o         = alu_V_flag_E;
    assign bypA_sel_o           = bypA_sel_E;
    assign bypB_sel_o           = bypB_sel_E;
    assign writeback_data_o     = writeback_mux_out_W;

    assign addr_bits_E          = alu_result_E[1:0];
    assign Ra_D                 = inst_read_data_D[4:0];
    assign addr_bits_D          = addr_bits_E;



    
    
    



    always @(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            pc_D                <= 32'd0;
            inst_read_data_D    <= {5'b11111, 27'd0}; // NOP instruction
            pc_E                <= 32'd0;
            pc_sel_E            <= 2'd0;
            data_mem_w_ctrl_E   <= 1'b0;
            data_mem_w_sel_E    <= 1'b0;
            alu_ctrl_E          <= 3'd0;
            alu_srcA_sel_E      <= 1'b0;
            alu_srcB_sel_E      <= 2'd0;
            reg_file_w_ctrl_E   <= 2'd0;
            reg_file_w_sel_E    <= 2'd0;
            writeback_sel_E     <= 2'd0;
            ext_data_E          <= 32'd0;
            bypA_sel_E          <= 2'd0;
            bypB_sel_E          <= 2'd0;
            Ra_E                <= 5'd0;
            pc_M                <= 32'd0;
            data_mem_w_ctrl_M   <= 1'b0;
            data_mem_w_sel_M    <= 1'b0;
            reg_file_w_ctrl_M   <= 2'd0;
            reg_file_w_sel_M    <= 2'd0;
            writeback_sel_M     <= 2'd0;
            alu_C_flag_M        <= 1'b0;
            alu_V_flag_M        <= 1'b0;
            alu_result_M        <= 32'd0;
            ext_data_M          <= 32'd0;
            Ra_M                <= 5'd0;
            pc_W                <= 32'd0;
            reg_file_w_ctrl_W   <= 2'd0;
            reg_file_w_sel_W    <= 2'd0;
            writeback_sel_W     <= 2'd0;
            alu_C_flag_W        <= 1'b0;
            alu_V_flag_W        <= 1'b0;
            alu_result_W        <= 32'd0;
            data_mem_r_data_W   <= 32'd0;
            Ra_W                <= 5'd0;
        end
        else begin
            // Fetch -> Decode
            if(flushD_o) begin
                pc_D                <= 32'd0;
                inst_read_data_D    <= {5'b11111, 27'd0}; // NOP instruction
            end
            else if(~stallD_o) begin
                pc_D                <= pc_F;
                inst_read_data_D    <= inst_read_data_F;
            end
            
            if(flushE_o) begin
                // Decode -> Execute
                pc_E                <= 32'd0;
                pc_sel_E            <= 2'd0;
                data_mem_w_ctrl_E   <= 1'b0;
                data_mem_w_sel_E    <= 1'b0;
                alu_ctrl_E          <= 3'd0;
                alu_srcA_sel_E      <= 1'b0;
                alu_srcB_sel_E      <= 2'd0;
                reg_file_w_ctrl_E   <= 2'd0;
                reg_file_w_sel_E    <= 2'd0;
                writeback_sel_E     <= 2'd0;
                reg_file_r_data1_E  <= 34'd0;
                reg_file_r_data2_E  <= 34'd0;
                ext_data_E          <= 32'd0;
                Ra_E                <= 5'd0;

            end
            else begin
                // Decode -> Execute
                pc_E                <= pc_D;
                pc_sel_E            <= pc_sel_D;
                data_mem_w_ctrl_E   <= data_mem_w_ctrl_D;
                data_mem_w_sel_E    <= data_mem_w_sel_D;
                alu_ctrl_E          <= alu_ctrl_D;
                alu_srcA_sel_E      <= alu_srcA_sel_D;
                alu_srcB_sel_E      <= alu_srcB_sel_D;
                reg_file_w_ctrl_E   <= reg_file_w_ctrl_D;
                reg_file_w_sel_E    <= reg_file_w_sel_D;
                writeback_sel_E     <= writeback_sel_D;
                Ra_E                <= Ra_D;
                bypA_sel_E          <= bypA_sel_D;
                bypB_sel_E          <= bypB_sel_D;
                reg_file_r_data1_E  <= reg_file_r_data1_D;
                reg_file_r_data2_E  <= reg_file_r_data2_D;
                ext_data_E          <= ext_data_D;
            end
            // Execute -> Memory
            pc_M                <= pc_E;
            data_mem_w_ctrl_M   <= data_mem_w_ctrl_E;
            data_mem_w_sel_M    <= data_mem_w_sel_E;
            reg_file_w_ctrl_M   <= reg_file_w_ctrl_E;
            reg_file_w_sel_M    <= reg_file_w_sel_E;
            writeback_sel_M     <= writeback_sel_E;
            alu_C_flag_M        <= alu_C_flag_E;
            alu_V_flag_M        <= alu_V_flag_E;
            alu_result_M        <= alu_result_E;
            ext_data_M          <= ext_data_E;
            Ra_M                <= Ra_E;
            alu_srcB_hazard_mux_out_M <= alu_srcB_hazard_mux_out_E;

            // Memory -> Writeback
            pc_W                <= pc_M;
            reg_file_w_ctrl_W   <= reg_file_w_ctrl_M;
            reg_file_w_sel_W    <= reg_file_w_sel_M;
            writeback_sel_W     <= writeback_sel_M;
            alu_C_flag_W        <= alu_C_flag_M;
            alu_V_flag_W        <= alu_V_flag_M;
            alu_result_W        <= alu_result_M;
            data_mem_r_data_W   <= data_mem_r_data_M;
            Ra_W                <= Ra_M;
            
        end
    end




    // Multiplexers
    assign alu_srcA_mux_out_E = (alu_srcA_sel_E == 1'b0) ? alu_srcA_hazard_mux_out_E : 1'b0;

    assign data_mem_w_mux_out_M = (data_mem_w_sel_M == 1'b0) ? alu_srcB_hazard_mux_out_M[31:0] : ext_data_M;
	
    assign alu_srcB_mux_out_E = (alu_srcB_sel_E == 2'd0) ? alu_srcB_hazard_mux_out_E : (alu_srcB_sel_E == 2'd1) ? {2'b00, ext_data_E} : 34'd0;

    assign reg_file_mux_out_D = (reg_file_w_sel_W == 1'b0) ? pc_W + 32'd4 : writeback_mux_out_W;

    assign writeback_mux_out_W = (writeback_sel_W == 1'b1) ? alu_result_W : data_mem_r_data_W;
    

    always @(*) begin
        case(pc_sel_E)
            2'd0: pc_mux_out_F = pc_F + 32'd4;
            2'd1: pc_mux_out_F = pc_E + alu_result_E;
            2'd2: pc_mux_out_F = alu_result_E;
            2'd3: pc_mux_out_F = pc_F;
        endcase
        case(bypA_sel_E) 
            2'd0: alu_srcA_hazard_mux_out_E <= reg_file_r_data1_E;
            2'd1: alu_srcA_hazard_mux_out_E <= alu_result_M;
            2'd2: alu_srcA_hazard_mux_out_E <= writeback_mux_out_W;
        endcase
        case(bypB_sel_E) 
            2'd0: alu_srcB_hazard_mux_out_E <= reg_file_r_data2_E;
            2'd1: alu_srcB_hazard_mux_out_E <= alu_result_M;
            2'd2: alu_srcB_hazard_mux_out_E <= writeback_mux_out_W;
        endcase
        case(byp_CU_sel_D)
            2'd0: ctrl_unit_regi_mux_out_D <= reg_file_r_data2_D;
            2'd1: ctrl_unit_regi_mux_out_D <= {alu_V_flag_E, alu_C_flag_E, alu_result_E};
            2'd2: ctrl_unit_regi_mux_out_D <= {alu_V_flag_M, alu_C_flag_M, alu_result_M};
        endcase
    end



endmodule

