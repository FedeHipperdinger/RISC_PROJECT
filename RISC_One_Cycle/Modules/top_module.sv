`include "control_unit_mod.sv"
`include "alu_mod.sv"
`include "register_file_mod.sv"
`include "data_memory_mod.sv"
`include "instruction_memory_mod.sv"
`include "program_counter_mod.sv"
`include "extender_mod.sv"



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
    output reg alu_V_flag_o

);
    //control unit wires
    wire [1:0]pc_sel_o;
    wire data_mem_w_ctrl_o;
    wire data_mem_w_sel_o;
    wire [2:0]alu_ctrl_o;
    wire alu_srcA_sel_o;
    wire [1:0]alu_srcB_sel_o;
    wire reg_file_w_ctrl_o;
  	wire [1:0]reg_file_w_sel_o;
    wire extend_ctrl_o;
    wire [1:0]status_flags_o;

    //extender wires
    wire [31:0]ext_data_o;

    // Multiplexers
    reg [31:0]pc_mux_out;
    reg [31:0]reg_file_mux_out;
    reg [33:0]alu_srcA_mux_out;
    reg [33:0]alu_srcB_mux_out;
    reg [31:0]data_mem_w_mux_out;

    control_unit_mod u_control_unit_mod (
        //Inputs
      	.clk(clk),
        .rst_n(rst_n),
        .T_i(inst_read_data_o[31:30]),
        .OPC_i(inst_read_data_o[29:27]),
        .reg_i(reg_file_r_data2_o),
        .C_flag_i(alu_C_flag_o),
        .V_flag_i(alu_V_flag_o),
        .addr_bits_i(alu_result_o[1:0]),
        .pc_bits_i(pc_o[1:0]),
        //Outputs
        .pc_sel_o(pc_sel_o),
        .data_mem_w_ctrl_o(data_mem_w_ctrl_o),
        .data_mem_w_sel_o(data_mem_w_sel_o),
        .alu_ctrl_o(alu_ctrl_o),
        .alu_srcA_sel_o(alu_srcA_sel_o),
        .alu_srcB_sel_o(alu_srcB_sel_o),
        .reg_file_w_ctrl_o(reg_file_w_ctrl_o),
        .reg_file_w_sel_o(reg_file_w_sel_o),
        .extend_ctrl_o(extend_ctrl_o),
        .status_flags_o(status_flags_o),
        .exception_flags_o(exception_flags_o)
    );

    alu_mod u_alu_mod (
        //Inputs
        .clk(clk),
        .alu_srcA_i(alu_srcA_mux_out),
        .alu_srcB_i(alu_srcB_mux_out),
        .alu_ctrl_i(alu_ctrl_o),
        //Outputs
        .alu_result_o(alu_result_o),
        .alu_C_flag_o(alu_C_flag_o),
        .alu_V_flag_o(alu_V_flag_o)
    );

    register_file_mod u_register_file_mod (
        //Inputs
        .clk(clk),
      	.rst_n(rst_n),
        .address1_i(inst_read_data_o[9:5]),
        .address2_i(inst_read_data_o[14:10]),
        .address3_i(inst_read_data_o[4:0]),
        .write_data3_i({status_flags_o, reg_file_mux_out}),
        .write_en3_i(reg_file_w_ctrl_o),
        //Outputs
        .read_data1_o(reg_file_r_data1_o),
        .read_data2_o(reg_file_r_data2_o)
    );

    data_memory_mod u_data_memory_mod (
        //Inputs
        .clk(clk),
        .rst_n(rst_n),
        .address_i(alu_result_o[N-1:0]),
        .write_data_i(data_mem_w_mux_out),
        .write_en_i(data_mem_w_ctrl_o),
        //Outputs
        .read_data_o(data_mem_r_data_o)
    );

    instruction_memory_mod u_instruction_memory_mod (
        //Inputs
        .clk(clk),
        .rst_n(rst_n),
        .address_i(pc_o),
        //Outputs
        .read_data_o(inst_read_data_o)
    );

    program_counter_mod u_program_counter_mod (
        //Inputs
        .clk(clk),
        .rst_n(rst_n),
        .next_pc_i(pc_mux_out),
        //Outputs
        .pc_o(pc_o)
    );

    extender_mod u_extender_mod (
        //Inputs
        .ext_ctrl_i(extend_ctrl_o),
        .dataA_i(inst_read_data_o[26:15]),
        .dataB_i(inst_read_data_o[14:10]),
        //Outputs
        .ext_data_o(ext_data_o)
    );

    // ----------------------------------

    // Multiplexers
    assign alu_srcA_mux_out = (alu_srcA_sel_o == 1'b0) ? reg_file_r_data1_o : 1'b0;

    assign data_mem_w_mux_out = (data_mem_w_sel_o == 1'b0) ? reg_file_r_data2_o : ext_data_o;
	
    assign alu_srcB_mux_out = (alu_srcB_sel_o == 2'd0) ? reg_file_r_data2_o : (alu_srcB_sel_o == 2'd1) ? {2'b00, ext_data_o} : 34'b0;

    always @(*) begin
        case(pc_sel_o)
            2'd0: pc_mux_out = pc_o + 32'd4;
            2'd1: pc_mux_out = pc_o + alu_result_o;
            2'd2: pc_mux_out = alu_result_o;
            2'd3: pc_mux_out = pc_o;
        endcase
    end

    always @(*) begin
        case(reg_file_w_sel_o)
            2'd0: reg_file_mux_out = pc_o + 32'd4;
            2'd1: reg_file_mux_out = data_mem_r_data_o;
            2'd2: reg_file_mux_out = alu_result_o;
        endcase
    end


endmodule

