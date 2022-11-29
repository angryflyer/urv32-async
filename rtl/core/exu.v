//流水线执行模块
`include "inst_index.v"
`include "macro.v"

module exu (
    input  clk,
    input  rstn,
    //from id
    input        id2iex_valid,
    input [31:0] id2iex_pc, 
    input [31:0] id2iex_pc_plus,
    input [31:0] id2iex_inst,
    input [4:0]  id2iex_rs1,
    input [4:0]  id2iex_rs2,
    input [4:0]  id2iex_rd,
    input        id2iex_bp_taken,
    input        id2iex_bp_match,
    input [`BP_ADDR_W-1:0]  id2iex_bp_addr,
    `ifndef SYNTHESIS
        input [63:0] id2iex_inst_ascii,  
    `endif
    //from id control signal
    input        id2iex_rf_we,
    input [1:0]  id2iex_rf_rd_sel0,
    input        id2iex_rf_rd_sel1,
    input        id2iex_alu_op2_sel,
    input [3:0]  id2iex_alu_op,
    input        id2iex_mul_valid,
    input        id2iex_mem_we,
    input        id2iex_mem_re,
    input [2:0]  id2iex_mem_byte_sel,
    input        id2iex_branch,
    input        id2iex_pc_src0,
    input        id2iex_jal_jalr,
    input [31:0] id2iex_imm,
    input [11:0] id2iex_csr,
    // from id
    input id2iex_csr_we,
    input id2iex_csr_re,
    input [2:0]  id2iex_csr_sel,
    input [31:0] id2iex_csr_rd,
    // from id
    // read and write registers / reg bus	
    input [31:0] id2iex_rd1,
    input [31:0] id2iex_rd2,

    //forward from lsu to iex
    input lsu2wb_rf_we,
    input [4:0]  lsu2wb_rd,
    input [31:0] lsu2wb_dout,
    //forward from wb to iex
    input wb2rf_wren,
    input [4:0]  wb2rf_waddr,
    input [31:0] wb2rf_wdata,

    // //forward csr from lsu to iex
    // input lsu2wb_csr_we,
    // input [11:0] lsu2wb_csr_wa,
    // input [31:0] lsu2wb_csr_wd,
    // //forward csr from wb to iex
    // input wb2csr_wren,
    // input [11:0] wb2csr_waddr,
    // input [31:0] wb2csr_wdata,

    input ac2iex_flush,
    input ac2iex_stall,

    //iex to lsu control signal
    output        iex2lsu_valid,
    output [31:0] iex2lsu_pc,
    output [31:0] iex2lsu_pc_plus,
    output [31:0] iex2lsu_inst,
    output [4:0]  iex2lsu_rs1,
    output [4:0]  iex2lsu_rs2,
    output [4:0]  iex2lsu_rd,
    `ifndef SYNTHESIS
        output [63:0] iex2lsu_inst_ascii, 
    `endif
    output        iex2lsu_rf_we,
    // output [1:0]  iex2lsu_rf_rd_sel0,
    output        iex2lsu_rf_rd_sel1,
    output        iex2lsu_mem_we,
    output        iex2lsu_mem_re,
    output [2:0]  iex2lsu_mem_byte_sel,       
    // output [31:0] iex2lsu_imm,
    // output [31:0] iex2lsu_pc_plus_imm,
    // output [31:0] iex2lsu_alu_out,
    output [31:0] iex2lsu_dout,
    output [31:0] iex2lsu_rf_rd2,
    // // to lsu
    // output iex2lsu_csr_we,
    // output iex2lsu_csr_re,
    // output [31:0] iex2lsu_csr_wd,

    // to csr
    output iex2csr_wren,
    output [11:0] iex2csr_waddr,
    output [31:0] iex2csr_wdata,	    

    //iex to if, id
    output        iex_jump_valid,
    output [31:0] iex_jump_pc,

    // exu to csr
    output        iex_except_valid,
    output [3:0]  iex_except_code,
    output        iex_valid,
    output [31:0] iex_pc,
    output [31:0] iex_next_pc,
    output        iex_mret,
    // exu <-> bpu
    output  flush_valid,
    output  flush_new_pc,
    output  flush_type,        //0->miss, 1->hit
    output  [`BP_ADDR_W-1:0]  flush_addr, //record 32 branch instruction pc
    output  [31:0] flush_bp_pc,
    output  [31:0] flush_pc,

    output  iex2ac_hazard
);

	reg        reg_iex_valid;
	reg [31:0] reg_iex_pc;
	reg [31:0] reg_iex_pc_plus;
	reg [31:0] reg_iex_inst;
	`ifndef SYNTHESIS
		reg [63:0] reg_iex_inst_ascii;
	`endif
	reg [4:0] reg_iex_rs1;
	reg [4:0] reg_iex_rs2;
	reg [4:0] reg_iex_rd;

	reg reg_iex_bp_taken;
	reg reg_iex_bp_match;
	reg [`BP_ADDR_W-1:0] reg_iex_bp_addr;
	//id to ex control signal
	reg        reg_iex_rf_we;
	reg [1:0]  reg_iex_rf_rd_sel0;
	reg        reg_iex_rf_rd_sel1;
	reg        reg_iex_alu_op2_sel;
	reg [3:0]  reg_iex_alu_op;
    reg        reg_iex_mul_valid;
	reg        reg_iex_mem_we;
	reg        reg_iex_mem_re;
	reg [2:0]  reg_iex_mem_byte_sel;
	reg        reg_iex_branch;
	reg        reg_iex_pc_src0;
	reg        reg_iex_jal_jalr;
	reg [31:0] reg_iex_imm;
	reg [11:0] reg_iex_csr;	
	reg        reg_iex_csr_we;
	reg        reg_iex_csr_re;
	reg [2:0]  reg_iex_csr_sel;
	reg [31:0] reg_iex_csr_rd;

	reg [31:0] reg_iex_rf_rd1;
	reg [31:0] reg_iex_rf_rd2;

	always @(posedge clk or negedge rstn) begin
		if(!rstn) begin
			reg_iex_valid      <= `DEASSERT;
			reg_iex_pc         <= `WORD_DEASSERT;
			reg_iex_pc_plus    <= `WORD_DEASSERT;
			reg_iex_inst       <= `WORD_DEASSERT;
			`ifndef SYNTHESIS
			reg_iex_inst_ascii <= `DWORD_DEASSERT;
			`endif
			reg_iex_rd		   <= 5'h0;
			reg_iex_rf_we      <= `DEASSERT;
			reg_iex_rf_rd_sel0 <= 2'b00;
			reg_iex_rf_rd_sel1 <= `DEASSERT;
			reg_iex_alu_op2_sel<= `DEASSERT;
			reg_iex_alu_op     <= `ALU_OP_ADD;
            reg_iex_mul_valid  <= `DEASSERT;
			reg_iex_mem_we     <= `DEASSERT;
			reg_iex_mem_re     <= `DEASSERT;
			reg_iex_mem_byte_sel <= 3'h0;
			reg_iex_branch     <= `DEASSERT;
			reg_iex_pc_src0    <= `DEASSERT;
			reg_iex_jal_jalr   <= `DEASSERT;
			reg_iex_imm        <= `U_IMM;   
			reg_iex_rf_rd1     <= `WORD_DEASSERT;
			reg_iex_rf_rd2     <= `WORD_DEASSERT;
			reg_iex_rs1        <= 5'h0; 
			reg_iex_rs2        <= 5'h0;
			reg_iex_csr_we     <= `DEASSERT;
			reg_iex_csr_re     <= `DEASSERT;
			reg_iex_csr_sel    <= 3'h0;
			reg_iex_csr_rd     <= `WORD_DEASSERT;
			reg_iex_bp_taken   <= `DEASSERT;
			reg_iex_bp_match   <= `DEASSERT;
			reg_iex_bp_addr    <= `WORD_DEASSERT;
		end else if(~(ac2iex_stall | iex2ac_hazard)) begin
			reg_iex_valid      <= id2iex_valid;
			reg_iex_pc         <= id2iex_pc;
			reg_iex_pc_plus    <= id2iex_pc_plus;
			reg_iex_inst       <= id2iex_inst;
			`ifndef SYNTHESIS
			reg_iex_inst_ascii <= id2iex_inst_ascii;
			`endif
            reg_iex_rd         <= id2iex_rd;
			reg_iex_rf_we      <= id2iex_rf_we;
			reg_iex_rf_rd_sel0 <= id2iex_rf_rd_sel0;
			reg_iex_rf_rd_sel1 <= id2iex_rf_rd_sel1;
			reg_iex_alu_op2_sel<= id2iex_alu_op2_sel;
			reg_iex_alu_op     <= id2iex_alu_op;
            reg_iex_mul_valid  <= id2iex_mul_valid;
			reg_iex_mem_we     <= id2iex_mem_we;
			reg_iex_mem_re     <= id2iex_mem_re;
			reg_iex_mem_byte_sel <= id2iex_mem_byte_sel;
			reg_iex_branch     <= id2iex_branch;
			reg_iex_pc_src0    <= id2iex_pc_src0;
			reg_iex_jal_jalr   <= id2iex_jal_jalr;
			reg_iex_imm        <= id2iex_imm;		
			reg_iex_rf_rd1     <= id2iex_rd1;
			reg_iex_rf_rd2     <= id2iex_rd2;	
			reg_iex_rs1        <= id2iex_rs1; 
			reg_iex_rs2        <= id2iex_rs2; 	
			reg_iex_csr_we     <= id2iex_csr_we;
			reg_iex_csr_re     <= id2iex_csr_re;
			reg_iex_csr_sel    <= id2iex_csr_sel;
			reg_iex_csr_rd     <= id2iex_csr_rd;	
			reg_iex_bp_taken   <= id2iex_bp_taken;
			reg_iex_bp_match   <= id2iex_bp_match;
			reg_iex_bp_addr    <= id2iex_bp_addr;
		end
	end
	
	//forward control signal
	wire lsu2iex_forward_val;
	wire wb2iex_forward_val;
	wire lsu2iex_forward_rs1_val, lsu2iex_forward_rs2_val;
	wire wb2iex_forward_rs1_val, wb2iex_forward_rs2_val;
	wire lsu2iex_rd_neq_zero;
	wire wb2iex_rd_neq_zero;
	wire lsu2iex_rd_eq_rs1, lsu2iex_rd_eq_rs2;
	wire wb2iex_rd_eq_rs1, wb2iex_rd_eq_rs2;

	assign lsu2iex_rd_neq_zero = (lsu2wb_rd   != 5'h0);
	assign wb2iex_rd_neq_zero  = (wb2rf_waddr != 5'h0);

	assign lsu2iex_rd_eq_rs1   = (lsu2wb_rd   == iex2lsu_rs1);	
	assign lsu2iex_rd_eq_rs2   = (lsu2wb_rd   == iex2lsu_rs2);
	assign wb2iex_rd_eq_rs1    = (wb2rf_waddr == iex2lsu_rs1);
	assign wb2iex_rd_eq_rs2    = (wb2rf_waddr == iex2lsu_rs2);

	assign lsu2iex_forward_val = lsu2wb_rf_we && lsu2iex_rd_neq_zero;
	assign wb2iex_forward_val  = wb2rf_wren && wb2iex_rd_neq_zero;

	assign lsu2iex_forward_rs1_val = lsu2iex_forward_val && lsu2iex_rd_eq_rs1;
	assign lsu2iex_forward_rs2_val = lsu2iex_forward_val && lsu2iex_rd_eq_rs2;
	assign wb2iex_forward_rs1_val  = wb2iex_forward_val  && wb2iex_rd_eq_rs1;
	assign wb2iex_forward_rs2_val  = wb2iex_forward_val  && wb2iex_rd_eq_rs2;
	//mux forward data to iex_rd1_src
	wire [31:0] iex_rd1_src;
	wire [31:0] iex_rd2_src;
	assign iex_rd1_src = lsu2iex_forward_rs1_val ? lsu2wb_dout 
					   : (wb2iex_forward_rs1_val ? wb2rf_wdata 
					   : reg_iex_rf_rd1);
	assign iex_rd2_src = lsu2iex_forward_rs2_val ? lsu2wb_dout 
	                   : (wb2iex_forward_rs2_val ? wb2rf_wdata 
					   : reg_iex_rf_rd2);

	// reg [31:0] iex_rd1_src;
	// reg [31:0] iex_rd2_src;
	// always @(*) begin
	// 	case({lsu2iex_forward_rs1_val,wb2iex_forward_rs1_val})
	// 		2'b00 : iex_rd1_src = reg_iex_rf_rd1;
	// 		2'b01 : iex_rd1_src = wb2rf_wdata;
	// 		2'b10 : iex_rd1_src = lsu2wb_dout;
	// 		default : iex_rd1_src = lsu2wb_dout;
	// 	endcase
	// end
	
	// always @(*) begin
	// 	case({lsu2iex_forward_rs2_val,wb2iex_forward_rs2_val})
	// 		2'b00 : iex_rd2_src = reg_iex_rf_rd2;
	// 		2'b01 : iex_rd2_src = wb2rf_wdata;
	// 		2'b10 : iex_rd2_src = lsu2wb_dout;
	// 		default : iex_rd2_src = lsu2wb_dout;
	// 	endcase
	// end

	// //forward csr from lsu to iex
	// wire [11:0] iex_csr_ra;
	// assign iex_csr_ra = reg_iex_inst[31:20];

	// wire lsu2iex_csra_eq_iex_csra;
	// wire lsu2iex_forward_csr_val;
	// assign lsu2iex_csra_eq_iex_csra  = (lsu2wb_csr_wa == iex_csr_ra);
	// assign lsu2iex_forward_csr_val   = lsu2wb_csr_we && lsu2iex_csra_eq_iex_csra;

	// //forward csr from wb to iex
	// wire wb2iex_csra_eq_iex_csra;
	// wire wb2iex_forward_csr_val;
	// assign wb2iex_csra_eq_iex_csra  = (wb2csr_waddr == iex_csr_ra);
	// assign wb2iex_forward_csr_val   = wb2csr_wren && wb2iex_csra_eq_iex_csra;
	// // mux forward data to id2iex_csr_rd
	// wire [31:0] iex_csr_rd;
	// assign iex_csr_rd = lsu2iex_forward_csr_val ? lsu2wb_csr_wd 
	//                    : wb2iex_forward_csr_val ? wb2csr_wdata 
	// 				   : reg_iex_csr_rd;

	// mux forward data to id2iex_csr_rd
	wire [31:0] iex_csr_rd;
    assign iex_csr_rd = reg_iex_csr_rd;

	//alu and op sel
	wire [31:0] alu_op1, alu_op2, alu_nmul_out;
    wire [31:0] alu_mul_out;
    wire [31:0] alu_out;
	wire alu_comp_out, pc_src1;
	assign alu_op1 = reg_iex_csr_sel[1] ? iex_csr_rd : iex_rd1_src;
	assign alu_op2 = reg_iex_csr_sel[0] ? ~iex_rd1_src 
	               : reg_iex_alu_op2_sel ? reg_iex_imm 
				   : iex_rd2_src;

	alu alu_u (
		.op(reg_iex_alu_op),
		.in0(alu_op1),
		.in1(alu_op2),
		.comp(alu_comp_out),
		.out(alu_nmul_out)
	);

	wire [2:0] inst_func3;
    wire [2:0] alu_mul_op;
    wire alu_mul_valid;
    wire alu_mul_ready;
    assign alu_mul_op = inst_func3; 
    assign alu_mul_valid = reg_iex_valid && reg_iex_mul_valid && ~ac2iex_stall;

    muldiv_vio muldiv_u (
        .clk(clk),
        .rstn(rstn),
        .op_stall(ac2iex_stall),
        .op_valid(alu_mul_valid),
        .op_ready(alu_mul_ready),
        .op(alu_mul_op),
        .op1(alu_op1),
        .op2(alu_op2),
        .op_out(alu_mul_out)
    );
    assign iex2ac_hazard = alu_mul_valid && ~alu_mul_ready;
    assign alu_out = reg_iex_mul_valid ? alu_mul_out : alu_nmul_out;

	wire [31:0] iex_pc_plus_imm;
	wire bp_jump_valid;
    wire branch_valid;
	assign bp_jump_valid = reg_iex_branch && ((~alu_comp_out && reg_iex_bp_taken) || (alu_comp_out && ~reg_iex_bp_taken));
	assign branch_valid = reg_iex_branch && alu_comp_out;
	assign pc_src1      = bp_jump_valid | reg_iex_jal_jalr;

	adder adder_pc_plus_imm_u (
		.in0(reg_iex_pc),
		.in1(reg_iex_imm),
		.out(iex_pc_plus_imm)
	);

	//iex_out -> rf_wd or mem_addr
	wire [31:0] iex_no_csr_dout;
	mux4to1 mux_no_csr_dout_u(
		.sel(reg_iex_rf_rd_sel0),
		.in0(alu_out),
		.in1(iex_pc_plus_imm),
		.in2(reg_iex_pc_plus),
		.in3(reg_iex_imm),
		.out(iex_no_csr_dout)
	);

	//mux no-csr and csr
	wire [31:0] iex_dout;
	wire [31:0] iex_csr_wd;

	assign iex_dout   = reg_iex_csr_sel[2] ? iex_csr_rd : iex_no_csr_dout;
	assign iex_csr_wd = reg_iex_csr_sel[1] ? iex_no_csr_dout   : iex_rd1_src;


	wire [31:0] iex_jalr_pc;
	assign iex_jalr_pc    = {alu_out[31:1], 1'b0};
	assign iex_jump_pc    = reg_iex_pc_src0  ? iex_jalr_pc 
						  : reg_iex_bp_taken ? reg_iex_pc_plus
						  : iex_pc_plus_imm;
	assign iex_jump_valid = reg_iex_valid && pc_src1 && (!ac2iex_flush);

	// iex to bpu
	assign flush_valid    = reg_iex_branch;
	assign flush_new_pc   = ~reg_iex_bp_match;  // not match, so need add pc to bpu
	assign flush_type     = branch_valid; // 1'b0 -> njump, 1'b1 -> jump 
	assign flush_addr     = reg_iex_bp_addr;
	assign flush_bp_pc    = reg_iex_pc[`BP_ADDR_BITS-1:0];
	assign flush_pc       = iex_pc_plus_imm;	
	//iex to lsu
	assign iex2lsu_valid      = reg_iex_valid && (!ac2iex_flush);
	assign iex2lsu_pc    	  = reg_iex_pc;
	assign iex2lsu_pc_plus    = reg_iex_pc_plus;
	assign iex2lsu_inst       = reg_iex_inst;
	assign iex2lsu_rd         = reg_iex_rd;
	`ifndef SYNTHESIS
	assign iex2lsu_inst_ascii = reg_iex_inst_ascii;
	`endif

	assign iex2lsu_rf_we      = reg_iex_rf_we && (!ac2iex_flush);
	// assign iex2lsu_rf_rd_sel0 = reg_iex_rf_rd_sel0;
	assign iex2lsu_rf_rd_sel1 = reg_iex_rf_rd_sel1;
	assign iex2lsu_mem_we     = reg_iex_mem_we && (!ac2iex_flush);
	assign iex2lsu_mem_re     = reg_iex_mem_re && (!ac2iex_flush);
	assign iex2lsu_mem_byte_sel = reg_iex_mem_byte_sel;
	// assign iex2lsu_imm        = reg_iex_imm;
	// assign iex2lsu_pc_plus_imm= iex_pc_plus_imm;
	// assign iex2lsu_alu_out    = alu_out;
	assign iex2lsu_dout       = iex_dout;
	assign iex2lsu_rf_rd2     = iex_rd2_src;

	assign iex2lsu_rs1        = reg_iex_rs1;
	assign iex2lsu_rs2        = reg_iex_rs2;	

	// assign iex2lsu_csr_we     = reg_iex_csr_we && (!ac2iex_flush);
	// assign iex2lsu_csr_re     = reg_iex_csr_re && (!ac2iex_flush);
	// assign iex2lsu_csr_wd     = iex_csr_wd;

	assign iex2csr_wren           = reg_iex_csr_we && (!ac2iex_flush);
	assign iex2csr_waddr          = reg_iex_inst[31:20];
	assign iex2csr_wdata          = iex_csr_wd;

	//obtain opcode and func
	wire [4:0]   inst_op_type;
	// wire [2:0]   inst_func3;
    wire [4:0]   inst_func5;
	wire [6:0]   inst_func7;
	assign inst_op_type = reg_iex_inst[6:2];
	assign inst_func7   = reg_iex_inst[31:25];
	assign inst_func3   = reg_iex_inst[14:12];
    assign inst_func5   = reg_iex_inst[24:20];
    //TRI
    wire is_inst_func5_tri;
    wire is_inst_func3_null;
    wire is_i_tri_imi_type;
    assign is_inst_func3_null = inst_func3 == `I_INST_FUNC3;
    assign is_inst_func5_tri  = inst_func5 == `I_INST_FUNC5_TRI;
	assign is_i_tri_imi_type  = iex2lsu_valid && (inst_op_type == `I_INST_OPCODE_4  );
    assign is_mret            = is_i_tri_imi_type && is_inst_func3_null && is_inst_func5_tri && (inst_func7 == `I_INST_FUNC7_MRET);

    //fixme
    assign iex_except_valid   = 1'b0;
    assign iex_except_code    = 4'h0;
    assign iex_valid          = iex2lsu_valid;
    assign iex_pc             = reg_iex_pc;
    assign iex_next_pc        = iex_jump_valid ? iex_jump_pc 
                              : branch_valid   ? iex_pc_plus_imm
                              : reg_iex_pc_plus;
    assign iex_mret           = is_mret;

endmodule
