//流水线load store模块
`include "inst_index.v"
`include "macro.v"

module lsu
  import urv_cfg::*;
  import urv_typedef::*; 
(
    input  clk,
    input  rstn,
    //iex to isu
    //iex to lsu control signal
    input        iex2lsu_valid,
    input [31:0] iex2lsu_pc,
    input [31:0] iex2lsu_pc_plus,
    input [31:0] iex2lsu_inst,
    input [4:0]  iex2lsu_rs1,
    input [4:0]  iex2lsu_rs2,
    input [4:0]  iex2lsu_rd,
    `ifndef SYNTHESIS
        input [63:0] iex2lsu_inst_ascii,
    `endif
    input        iex2lsu_rf_we,
    // input [1:0]  iex2lsu_rf_rd_sel0,
    input        iex2lsu_rf_rd_sel1,
    input        iex2lsu_mem_we,
    input        iex2lsu_mem_re,
    input [2:0]  iex2lsu_mem_byte_sel,
    // input [31:0] iex2lsu_imm,
    // input [31:0] iex2lsu_pc_plus_imm,

    //iex to lsu address and data
    // input [31:0] iex2lsu_alu_out,
    input [31:0] iex2lsu_dout,
    input [31:0] iex2lsu_rf_rd2,

    // // from iex
    // input iex2lsu_csr_we,
    // input iex2lsu_csr_re,
    // input [31:0] iex2lsu_csr_wd,

    //forward from wb to lsu
    //lord type -> store type
    input wb2rf_wren,
    input [4:0]  wb2rf_waddr,
    input [31:0] wb2rf_wdata,

    input ac2lsu_flush,
    input ac2lsu_stall,

    // LSU memery bus
    output           dmem_req_valid,
    input            dmem_req_ready,
    output mem_req_t dmem_req,

    //lsu to wb
    output        lsu2wb_valid,
    output [31:0] lsu2wb_pc,
    output [31:0] lsu2wb_pc_plus,
    output [31:0] lsu2wb_inst,
    output [4:0]  lsu2wb_rs1,
    output [4:0]  lsu2wb_rs2,
    output [4:0]  lsu2wb_rd,
    `ifndef SYNTHESIS
        output [63:0] lsu2wb_inst_ascii, 
    `endif
    output        lsu2wb_rf_we,
    // output [1:0]  lsu2wb_rf_rd_sel0,
    output        lsu2wb_rf_rd_sel1,
    // output [31:0] lsu2wb_imm,
    // output [31:0] lsu2wb_pc_plus_imm,
    // output [31:0] lsu2wb_alu_out,
    output [31:0] lsu2wb_dout,
    output        lsu2wb_mem_valid,
    output [2:0]  lsu2wb_mem_byte_sel,
    output [1:0]  lsu2wb_mem_addr_offset,
    // // to lsu
    // output lsu2wb_csr_we,
    // output lsu2wb_csr_re,
    // output [11:0] lsu2wb_csr_wa,
    // output [31:0] lsu2wb_csr_wd,

    output lsu2ac_hazard
);

	reg        reg_lsu_valid;
	reg [31:0] reg_lsu_pc;
	reg [31:0] reg_lsu_pc_plus;
	reg [31:0] reg_lsu_inst;
	reg [4:0]  reg_lsu_rs1;
	reg [4:0]  reg_lsu_rs2;
	reg [4:0]  reg_lsu_rd;
	`ifndef SYNTHESIS
		reg [63:0] reg_lsu_inst_index;
	`endif
	//id to ex control signal
	reg        reg_lsu_rf_we;
	// reg [1:0]  reg_lsu_rf_rd_sel0;
	reg        reg_lsu_rf_rd_sel1;
	reg        reg_lsu_mem_we;
	reg        reg_lsu_mem_re;
	reg [2:0]  reg_lsu_mem_byte_sel;
	// reg [31:0] reg_lsu_imm;
	// reg [31:0] reg_lsu_pc_plus_imm;
	// reg [31:0] reg_lsu_alu_out;
	reg [31:0] reg_isu_dout;
	reg [31:0] reg_lsu_rf_rd2;

	// reg reg_lsu_csr_we;
	// reg reg_lsu_csr_re;
	// reg [31:0] reg_lsu_csr_wd;

	always @(posedge clk or negedge rstn) begin
		if(!rstn) begin
			reg_lsu_valid      <= `DEASSERT;
			reg_lsu_pc         <= `WORD_DEASSERT;
			reg_lsu_pc_plus    <= `WORD_DEASSERT;
			reg_lsu_inst       <= `WORD_DEASSERT;
			`ifndef SYNTHESIS
			reg_lsu_inst_index <= `DWORD_DEASSERT;
			`endif
			reg_lsu_rd		   <= 5'h0;
			reg_lsu_rf_we      <= `DEASSERT;
			// reg_lsu_rf_rd_sel0 <= 2'b00;
			reg_lsu_rf_rd_sel1 <= `DEASSERT;
			reg_lsu_mem_we     <= `DEASSERT;
			reg_lsu_mem_re     <= `DEASSERT;
			reg_lsu_mem_byte_sel <= 3'h0;
			// reg_lsu_imm        <= `U_IMM;  
			// reg_lsu_pc_plus_imm<= `WORD_DEASSERT;
			// reg_lsu_alu_out    <= `WORD_DEASSERT;
			reg_isu_dout       <= `WORD_DEASSERT;
			reg_lsu_rf_rd2     <= `WORD_DEASSERT;
			reg_lsu_rs1        <= 5'h0;
			reg_lsu_rs2        <= 5'h0;
			// reg_lsu_csr_we     <= `DEASSERT;
			// reg_lsu_csr_re     <= `DEASSERT;
			// reg_lsu_csr_wd     <= `WORD_DEASSERT;
		end else if(~ac2lsu_stall) begin
			reg_lsu_valid      <= iex2lsu_valid;
			reg_lsu_pc         <= iex2lsu_pc;
			reg_lsu_pc_plus    <= iex2lsu_pc_plus;
			reg_lsu_inst       <= iex2lsu_inst;
			`ifndef SYNTHESIS
			reg_lsu_inst_index <= iex2lsu_inst_ascii;
			`endif
            reg_lsu_rd         <= iex2lsu_rd;
			reg_lsu_rf_we      <= iex2lsu_rf_we;
			// reg_lsu_rf_rd_sel0 <= iex2lsu_rf_rd_sel0;
			reg_lsu_rf_rd_sel1 <= iex2lsu_rf_rd_sel1;
			reg_lsu_mem_we     <= iex2lsu_mem_we;
			reg_lsu_mem_re     <= iex2lsu_mem_re;
			reg_lsu_mem_byte_sel <= iex2lsu_mem_byte_sel;
			// reg_lsu_imm        <= iex2lsu_imm;	
			// reg_lsu_pc_plus_imm<= iex2lsu_pc_plus_imm;	
			// reg_lsu_alu_out    <= iex2lsu_alu_out;
			reg_isu_dout       <= iex2lsu_dout;
			reg_lsu_rf_rd2     <= iex2lsu_rf_rd2;
			reg_lsu_rs1        <= iex2lsu_rs1;
			reg_lsu_rs2        <= iex2lsu_rs2;
			// reg_lsu_csr_we     <= iex2lsu_csr_we;
			// reg_lsu_csr_re     <= iex2lsu_csr_re;
			// reg_lsu_csr_wd     <= iex2lsu_csr_wd;
		end
	end

	//forward control signal
	wire wb2lsu_forward_val;
	wire wb2lsu_forward_rs2_val;
	wire wb2lsu_rd_neq_zero;
	wire wb2lsu_rd_eq_rs2;
	wire [31:0] lsu_rd2_src;

	assign wb2lsu_rd_neq_zero = (wb2rf_waddr != 5'h0);
	assign wb2lsu_rd_eq_rs2   = (wb2rf_waddr == lsu2wb_rs2);
	assign wb2lsu_forward_val = wb2rf_wren && (wb2lsu_rd_neq_zero);

	assign wb2lsu_forward_rs2_val = wb2lsu_forward_val && wb2lsu_rd_eq_rs2;
	//mux wb data to lsu_rd2_src
	assign lsu_rd2_src = wb2lsu_forward_rs2_val ? wb2rf_wdata : reg_lsu_rf_rd2;


	//lsu to mem
	wire [1:0]  mem_addr_offset;
	reg  [31:0] reg_dmem_wdata;
	reg  [3:0]  reg_dmem_wstb;

	assign mem_addr_offset = reg_isu_dout[1:0];
	always @(*) begin
		case(reg_lsu_mem_byte_sel)
			`MEM_BYTE_S : begin
				reg_dmem_wdata = {4{lsu_rd2_src[7:0]}};				
				reg_dmem_wstb  = (4'b0001 << mem_addr_offset);
			end
			`MEM_HALF_S : begin
				reg_dmem_wdata = {2{lsu_rd2_src[15:0]}};				
				reg_dmem_wstb  = (4'b0011 << {mem_addr_offset[1], 1'b0});
			end
			`MEM_WORD_S : begin
				reg_dmem_wdata = lsu_rd2_src;
				reg_dmem_wstb  = 4'b1111;
			end
			default     : begin
				reg_dmem_wdata = `WORD_DEASSERT;
				reg_dmem_wstb  = 4'b0000;
			end
		endcase
	end

    wire dmem_req_handshaked;

	assign dmem_req_valid    = (reg_lsu_mem_we | reg_lsu_mem_re);
    assign dmem_req.req_type = reg_lsu_mem_we ? MEM_WRITE : MEM_READ;
	assign dmem_req.req_addr = dmem_req_valid ? reg_isu_dout : `WORD_DEASSERT;
	assign dmem_req.req_burst= 1'b1;
	assign dmem_req.req_mask = reg_dmem_wstb;
    assign dmem_req.req_data = reg_dmem_wdata;

	assign lsu2ac_hazard     = dmem_req_valid && ~dmem_req_ready;
    assign dmem_req_handshaked = dmem_req_valid && dmem_req_ready;
	//isu to wb
	// assign lsu2wb_valid       = reg_lsu_valid && dmem_req_handshaked && (!ac2lsu_flush);
    assign lsu2wb_valid       = reg_lsu_valid;
	assign lsu2wb_pc          = reg_lsu_pc;
	assign lsu2wb_pc_plus     = reg_lsu_pc_plus;
	assign lsu2wb_inst        = reg_lsu_inst;
	assign lsu2wb_rd          = reg_lsu_rd;
	`ifndef SYNTHESIS
	assign lsu2wb_inst_ascii  = reg_lsu_inst_index;
	`endif
	assign lsu2wb_rf_we       = reg_lsu_rf_we && (!ac2lsu_flush);  
	// assign lsu2wb_rf_we       = reg_lsu_rf_we; 
	// assign lsu2wb_rf_rd_sel0  = reg_lsu_rf_rd_sel0;
	assign lsu2wb_rf_rd_sel1  = reg_lsu_rf_rd_sel1;
	// assign lsu2wb_imm         = reg_lsu_imm;
	// assign lsu2wb_pc_plus_imm = reg_lsu_pc_plus_imm;
	// assign lsu2wb_alu_out     = reg_lsu_alu_out; 
	assign lsu2wb_dout        = reg_isu_dout;

	assign lsu2wb_rs1         = reg_lsu_rs1;
	assign lsu2wb_rs2         = reg_lsu_rs2;

    assign lsu2wb_mem_valid       = dmem_req_valid;
	assign lsu2wb_mem_addr_offset = mem_addr_offset;
	assign lsu2wb_mem_byte_sel    = reg_lsu_mem_byte_sel;

	// wire [11:0] lsu_csr_addr;
	// assign lsu_csr_addr       = reg_lsu_inst[31:20];
	// assign lsu2wb_csr_we      = reg_lsu_csr_we && (!ac2lsu_stall);
	// assign lsu2wb_csr_re      = reg_lsu_csr_re && (!ac2lsu_stall);
	// assign lsu2wb_csr_we      = reg_lsu_csr_we;
	// assign lsu2wb_csr_re      = reg_lsu_csr_re;    
	// assign lsu2wb_csr_wa      = lsu_csr_addr;
	// assign lsu2wb_csr_wd      = reg_lsu_csr_wd;
	// wire [31:0] lsu2rf_wd;
    // assign lsu2rf_wd = reg_ex2rf_rd_sel ? mem2iex_rdata : alu_out;
	// //imm to lsu
	// mux4to1 mux4to1_u(.sel(reg_lsu_rf_rd_sel), .in0(alu_out), .in1(iex_pc_plus_imm), .in2(reg_lsu_pc_plus), .in3(reg_lsu_imm), .out(iex2lsu_rf_wd));

	// assign iex2xreg_waddr = reg_lsu_rd;
	// assign iex2xreg_wren  = reg_lsu_rf_we;
	//lsu write or read


endmodule
