//流水线写回模块
`include "inst_index.v"
`include "macro.v"

module wbu
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input  clk,
    input  rstn,
    //lsu to wb
    input        lsu2wb_valid,
    input [31:0] lsu2wb_pc,
    input [31:0] lsu2wb_pc_plus,
    input [31:0] lsu2wb_inst,
    input [4:0]  lsu2wb_rs1,
    input [4:0]  lsu2wb_rs2,
    input [4:0]  lsu2wb_rd,
    `ifndef SYNTHESIS
        input [63:0] lsu2wb_inst_ascii, 
    `endif
    input        lsu2wb_rf_we,
    // input [1:0]  lsu2wb_rf_rd_sel0,
    input        lsu2wb_rf_rd_sel1,
    // input [31:0] lsu2wb_imm,
    // input [31:0] lsu2wb_pc_plus_imm,
    // input [31:0] lsu2wb_alu_out,
    input [31:0] lsu2wb_dout,
    input        lsu2wb_mem_valid,
    input [2:0]  lsu2wb_mem_byte_sel,
    input [1:0]  lsu2wb_mem_addr_offset,
    //from memery
    input             dmem_resp_valid,
    output            dmem_resp_ready,
    input  mem_resp_t dmem_resp, 
    // //from lsu
    // input lsu2wb_csr_we,
    // input lsu2wb_csr_re,
    // input [31:0] lsu2wb_csr_wd,	
    //from aux_ctrl
    input ac2wb_stall,
    //to acu
    output wb2ac_hazard,

    //wb to iex/forward
    output [4:0]  wb_rs1,
    output [4:0]  wb_rs2,
    //wb to rf
    output        wb2rf_wren,
    output [4:0]  wb2rf_waddr,
    output [31:0] wb2rf_wdata

    // output wb2csr_wren,
    // output [11:0] wb2csr_waddr,
    // output [31:0] wb2csr_wdata		
);
	reg        reg_wb_valid;
	reg [31:0] reg_wb_pc;
	reg [31:0] reg_wb_pc_plus;
	reg [31:0] reg_wb_inst;
	reg [4:0]  reg_wb_rs1;
	reg [4:0]  reg_wb_rs2;
	reg [4:0]  reg_wb_rd;
	`ifndef SYNTHESIS
		reg [63:0] reg_wb_inst_index;
	`endif
	//id to ex control signal
	reg        reg_wb_rf_we;
	// reg [1:0]  reg_wb_rf_rd_sel0;
	reg        reg_wb_rf_rd_sel1;
	// reg [31:0] reg_wb_imm;
	// reg [31:0] reg_wb_pc_plus_imm;
	// reg [31:0] reg_wb_alu_out;
    reg [31:0] reg_wb_dout;	
    reg        reg_wb_mem_valid;
	reg [2:0]  reg_wb_mem_byte_sel;
	reg [1:0]  reg_wb_mem_addr_offset;

	// reg reg_wb_csr_we;
	// reg reg_wb_csr_re;
	// reg [31:0] reg_wb_csr_wd;	

	always @(posedge clk or negedge rstn) begin
		if(!rstn) begin
			reg_wb_valid      <= `DEASSERT;
			reg_wb_pc         <= `WORD_DEASSERT;
			reg_wb_pc_plus    <= `WORD_DEASSERT;
			reg_wb_inst       <= `WORD_DEASSERT;
			`ifndef SYNTHESIS
			reg_wb_inst_index <= `DWORD_DEASSERT;
			`endif
			reg_wb_rd		   <= 5'h0;
			reg_wb_rf_we      <= `DEASSERT;
			// reg_wb_rf_rd_sel0 <= 2'b00;
			reg_wb_rf_rd_sel1 <= `DEASSERT;
			// reg_wb_imm        <= `U_IMM;  
			// reg_wb_pc_plus_imm<= `WORD_DEASSERT;
			// reg_wb_alu_out    <= `WORD_DEASSERT;
			reg_wb_dout       <= `WORD_DEASSERT;
			reg_wb_rs1        <= 5'h0;
			reg_wb_rs2        <= 5'h0;
            reg_wb_mem_valid  <= `DEASSERT;
			reg_wb_mem_byte_sel    <= 3'h0;
			reg_wb_mem_addr_offset <= 2'b00;
			// reg_wb_csr_we     <= `DEASSERT;
			// reg_wb_csr_re     <= `DEASSERT;
			// reg_wb_csr_wd     <= `WORD_DEASSERT;
		end else if(~ac2wb_stall) begin
			reg_wb_valid      <= lsu2wb_valid;
			reg_wb_pc         <= lsu2wb_pc;
			reg_wb_pc_plus    <= lsu2wb_pc_plus;
			reg_wb_inst       <= lsu2wb_inst;
			`ifndef SYNTHESIS
			reg_wb_inst_index <= lsu2wb_inst_ascii;
			`endif
            reg_wb_rd         <= lsu2wb_rd;
			reg_wb_rf_we      <= lsu2wb_rf_we;
			// reg_wb_rf_rd_sel0 <= lsu2wb_rf_rd_sel0;
			reg_wb_rf_rd_sel1 <= lsu2wb_rf_rd_sel1;
			// reg_wb_imm        <= lsu2wb_imm;	
			// reg_wb_pc_plus_imm<= lsu2wb_pc_plus_imm;	
			// reg_wb_alu_out    <= lsu2wb_alu_out;
			reg_wb_dout       <= lsu2wb_dout;
			reg_wb_rs1        <= lsu2wb_rs1;
			reg_wb_rs2        <= lsu2wb_rs2;
            reg_wb_mem_valid  <= lsu2wb_mem_valid;
			reg_wb_mem_byte_sel    <= lsu2wb_mem_byte_sel;
			reg_wb_mem_addr_offset <= lsu2wb_mem_addr_offset;
			// reg_wb_csr_we     <= lsu2wb_csr_we;
			// reg_wb_csr_re     <= lsu2wb_csr_re;
			// reg_wb_csr_wd     <= lsu2wb_csr_wd;
		end
	end

    // dmem_rdata handle
    wire[31:0] dmem_rdata;
	reg [31:0] reg_dmem_rdata;
	always @(*) begin
		case(reg_wb_mem_byte_sel)
			`MEM_BYTE_S : begin
				case(reg_wb_mem_addr_offset) 
					2'b00   : reg_dmem_rdata = {{24{dmem_rdata[7]} }, dmem_rdata[7:0]  };
					2'b01   : reg_dmem_rdata = {{24{dmem_rdata[15]}}, dmem_rdata[15:8] };
					2'b10   : reg_dmem_rdata = {{24{dmem_rdata[23]}}, dmem_rdata[23:16]};
					default : reg_dmem_rdata = {{24{dmem_rdata[31]}}, dmem_rdata[31:24]};
				endcase
			end
			`MEM_BYTE_U : begin
				case(reg_wb_mem_addr_offset) 
					2'b00   : reg_dmem_rdata = {{24{1'b0} }, dmem_rdata[7:0]  };
					2'b01   : reg_dmem_rdata = {{24{1'b0} }, dmem_rdata[15:8] };
					2'b10   : reg_dmem_rdata = {{24{1'b0} }, dmem_rdata[23:16]};
					default : reg_dmem_rdata = {{24{1'b0} }, dmem_rdata[31:24]};
				endcase
			end
			`MEM_HALF_S : begin
				case(reg_wb_mem_addr_offset[1]) 
					1'b0    : reg_dmem_rdata = {{16{dmem_rdata[15]} }, dmem_rdata[15:0] };
					default : reg_dmem_rdata = {{16{dmem_rdata[31]} }, dmem_rdata[31:16]};
				endcase
			end
			`MEM_HALF_U : begin
				case(reg_wb_mem_addr_offset[1]) 
					1'b0    : reg_dmem_rdata = {{16{1'b0} }, dmem_rdata[15:0] };
					default : reg_dmem_rdata = {{16{1'b0} }, dmem_rdata[31:16]};
				endcase
			end
			default : begin
				reg_dmem_rdata = dmem_rdata;
			end
		endcase
	end

    //停止信号ac2wb_stall打1拍后给到指令buffer/指令寄存器	
    reg  dbuf_stall;
    wire dbuf_stall_up;
    assign dbuf_stall_up = reg_wb_valid && reg_wb_mem_valid && dmem_resp_valid && ac2wb_stall && ~dbuf_stall;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            dbuf_stall <= `DEASSERT;
        end else if(dbuf_stall_up) begin
            dbuf_stall <= ac2wb_stall;
        end else if(~ac2wb_stall && dbuf_stall) begin
            dbuf_stall <= ac2wb_stall;
        end
    end
    //指令buffer，深度1级	
    //ac2wb_stall信号打1拍后生效
    reg  [31:0] dbuf;
    reg  ff_dmem_resp_valid;
    wire dmem_resp_valid_mux;       
    //指令buffer在ac2wb_stall有效时，dbuf_stall才有效
    //撤销ac2wb_stall后即开始恢复运行	
    wire dbuf_stall_en = ~(dbuf_stall && ac2wb_stall);
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            dbuf     <= `WORD_DEASSERT; 
            ff_dmem_resp_valid <= `DEASSERT;
        end 
        else if(dbuf_stall_en)  begin
            dbuf     <= dmem_resp.resp_data; 
            ff_dmem_resp_valid <= dmem_resp_valid;
        end
    end 
    //指令根据dbuf_stall信号进行选择dbuf还是dmem_resp.resp_data
    assign imem_resp_valid_mux = (dbuf_stall) ? ff_dmem_resp_valid : dmem_resp_valid;
    // assign dmem_resp_ready     = reg_wb_mem_valid && reg_wb_valid;
	assign dmem_resp_ready     = 1'b1;
    assign dmem_rdata          = (dbuf_stall) ? dbuf : dmem_resp.resp_data;

    // wb2ac_hazard
    assign wb2ac_hazard    = reg_wb_valid && reg_wb_mem_valid && ~imem_resp_valid_mux;
    // assign wb2ac_hazard    = 1'b0;

	//mux wb_dout or mem data
	wire [31:0] wb2rf_wd;
	assign wb2rf_wd = reg_wb_rf_rd_sel1 ? reg_dmem_rdata : reg_wb_dout;
    // wire [31:0] wb2rf_mem_alu_wd;
    // assign wb2rf_mem_alu_wd = reg_wb_rf_rd_sel1 ? reg_dmem_rdata : reg_wb_alu_out;
	// //mux data to rf
	// mux4to1 mux_rf_wd_u(.sel(reg_wb_rf_rd_sel0), .in0(wb2rf_mem_alu_wd), .in1(reg_wb_pc_plus_imm), .in2(reg_wb_pc_plus), .in3(reg_wb_imm), .out(wb2rf_wd));

	//wb to rf
	// assign wb2rf_wren  = reg_wb_rf_we && (!ac2wb_stall);
	assign wb2rf_wren  = reg_wb_rf_we;
	assign wb2rf_waddr = reg_wb_rd;
	assign wb2rf_wdata = wb2rf_wd;

	// //wb to csr
	// wire [11:0] wb_csr_addr;
	// assign wb_csr_addr  = reg_wb_inst[31:20];
	// // assign wb2csr_wren  = reg_wb_csr_we && (!ac2wb_stall);
	// assign wb2csr_wren  = reg_wb_csr_we;
	// assign wb2csr_waddr = wb_csr_addr;
	// assign wb2csr_wdata = reg_wb_csr_wd;

	assign wb_rs1 = reg_wb_rs1;
	assign wb_rs2 = reg_wb_rs2;
endmodule
