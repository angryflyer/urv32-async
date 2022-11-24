//inst_reg
`include "macro.v"
//Float-Point CSR
`define CSR_FFLAG_ADDR    12'h001
`define CSR_FRM_ADDR      12'h002
`define CSR_FCSR_ADDR     12'h003
//User Counter/Timers
//read only
`define CSR_CYCLE_ADDR    12'hc00
`define CSR_TIME_ADDR     12'hc01
`define CSR_INSTRET_ADDR  12'hc02
`define CSR_CYCLEH_ADDR   12'hc80 //RV32I only
`define CSR_TIMEH_ADDR    12'hc81 //RV32I only
`define CSR_INSTRETH_ADDR 12'hc82 //RV32I only

`define CSR_TIME_CMP_ADDR  32'h5000
`define CSR_TIME_CMPH_ADDR 32'h5004

//read only
`define CSR_MVENDORID_ADDR 12'hf11
`define CSR_MARCHID_ADDR  12'hf12
`define CSR_MIMPID_ADDR   12'hf13
`define CSR_MHARTID_ADDR  12'hf14
//machine trap setup
`define CSR_MSTATUS_ADDR  12'h300
`define CSR_MISA_ADDR     12'h301
`define CSR_MEDELEG_ADDR  12'h302
`define CSR_MIDELEG_ADDR  12'h303
`define CSR_MIE_ADDR      12'h304
`define CSR_MTVEC_ADDR    12'h305
`define CSR_MCOUNTEREN_ADDR  12'h306
//machine trap handling
`define CSR_MSCRATCH_ADDR 12'h340
`define CSR_MEPC_ADDR     12'h341
`define CSR_MCAUSE_ADDR   12'h342
`define CSR_MTVAL_ADDR    12'h343
`define CSR_MIP_ADDR      12'h344
//machine Counter/Timers
//read and write
`define CSR_MCYCLE_ADDR     12'hb00
`define CSR_MINISTRET_ADDR  12'hb02
`define CSR_MCYCLEH_ADDR    12'hb80
`define CSR_MINISTRETH_ADDR 12'hb82

module gpr (
    input clk,
    input rstn,
    input wren,
    input [4:0] waddr,
    input [31:0] wdata,
    input [4:0] raddr1,
    input [4:0] raddr2,

    output [31:0] rdata1,
    output [31:0] rdata2
);
    //通用寄存器x_reg[0:4] 32个通用寄存器
    reg [31:0] reg_x [0:31];
    //only for simulation or fpga check
    `ifndef SYNTHESIS 
//        wire [31:0] x0_zero = `WORD_DEASSERT;
        wire [31:0] x0_zero =  reg_x[0];
        wire [31:0] x1_ra = reg_x[1];
        wire [31:0] x2_sp = reg_x[2];
        wire [31:0] x3_gp = reg_x[3];
        wire [31:0] x4_tp = reg_x[4];
        wire [31:0] x5_t0 = reg_x[5];
        wire [31:0] x6_t1 = reg_x[6];
        wire [31:0] x7_t2 = reg_x[7];
        wire [31:0] x8_s0_fp = reg_x[8];
        wire [31:0] x9_s1 = reg_x[9];
        wire [31:0] x10_a0 = reg_x[10];
        wire [31:0] x11_a1 = reg_x[11];
        wire [31:0] x12_a2 = reg_x[12];
        wire [31:0] x13_a3 = reg_x[13];
        wire [31:0] x14_a4 = reg_x[14];
        wire [31:0] x15_a5 = reg_x[15];
        wire [31:0] x16_a6 = reg_x[16];
        wire [31:0] x17_a7 = reg_x[17];
        wire [31:0] x18_s2 = reg_x[18];
        wire [31:0] x19_s3 = reg_x[19];
        wire [31:0] x20_s4 = reg_x[20];
        wire [31:0] x21_s5 = reg_x[21];
        wire [31:0] x22_s6 = reg_x[22];
        wire [31:0] x23_s7 = reg_x[23];
        wire [31:0] x24_s8 = reg_x[24];
        wire [31:0] x25_s9 = reg_x[25];
        wire [31:0] x26_s10 = reg_x[26];
        wire [31:0] x27_s11 = reg_x[27];
        wire [31:0] x28_t3 = reg_x[28];
        wire [31:0] x29_t4 = reg_x[29];
        wire [31:0] x30_t5 = reg_x[30];
        wire [31:0] x31_t6 = reg_x[31];
    `endif

    wire waddr_eq_raddr1 = wren && (waddr == raddr1);
    wire waddr_eq_raddr2 = wren && (waddr == raddr2);
    wire waddr_neq_zero  = wren && (waddr != 5'h0);
    integer i; 
    
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin 
            for (i=0; i < 32; i=i+1) 
            begin
                reg_x[i] <= `WORD_DEASSERT;
            end
        end else 
        if(waddr_neq_zero) begin
            reg_x[waddr] <= wdata;
        end
        else begin
            reg_x[waddr] <= reg_x[waddr];
        end
    end

    wire [31:0] data1_out;
    wire [31:0] data2_out;
    assign data1_out = (raddr1 == 5'h0) ? `WORD_DEASSERT : (waddr_eq_raddr1 ? wdata : reg_x[raddr1]); 
    assign data2_out = (raddr2 == 5'h0) ? `WORD_DEASSERT : (waddr_eq_raddr2 ? wdata : reg_x[raddr2]); 
    assign rdata1 = data1_out;
    assign rdata2 = data2_out;
    // reg [31:0] data1_out;
    // reg [31:0] data2_out;
    // //pass wdata to rdata directly if reading when writing
    // //read raddr1
    // always @(*) begin
    //     case(raddr1)
    //         5'h0 :     data1_out = `WORD_DEASSERT;
    //       default :  data1_out = waddr_eq_raddr1 ? wdata : reg_x[raddr1];
    //         // default :  data1_out = reg_x[raddr1];
    //     endcase
    // end
    // //read raddr2
    // always @(*) begin
    //     case(raddr2)
    //         5'h0 :     data2_out = `WORD_DEASSERT;
    //        default :  data2_out = waddr_eq_raddr2 ? wdata : reg_x[raddr2];
    //         // default :  data2_out = reg_x[raddr2];
    //     endcase
    // end
endmodule

module csr
(
    input                  clk,
    input                  rstn,
    // from clint
    input                  soft_irq,
    input                  time_irq,
    input [63:0]           time_val, 
    // from plic
    input                  ext_irq,
    // from exu
    input                  iex_except_valid,
    input [3:0]            iex_except_code,
    input                  iex_valid,
    input [31:0]           iex_pc,
    input [31:0]           iex_next_pc,
    input                  iex_mret,
    // from wbu
    input                  wren,
    input [11:0]           waddr,
    input [31:0]           wdata,
    input [11:0]           raddr,
    input                  instret_stall,
    input                  ac2csr_stall,
    output [31:0]          rdata,
    // to acu
    output                 trap_jump_valid,
    output [31:0]          trap_jump_pc
);

    //parameter
    localparam CSR_MISA_PARAM      = `WORD_DEASSERT;
    localparam CSR_MVENDORID_PARAM = `WORD_DEASSERT;
    localparam CSR_MARCHID_PARAM   = `WORD_DEASSERT;
    localparam CSR_MIMPID_PARAM    = `WORD_DEASSERT;    
    localparam CSR_MHARTID_PARAM   = `WORD_DEASSERT; 
    // super/machine soft/timer/external
    localparam RES                 = 5'h0;
    localparam SSI                 = 5'h1;
    localparam MSI                 = 5'h3;
    localparam STI                 = 5'h5;
    localparam MTI                 = 5'h7;
    localparam SEI                 = 5'h9;
    localparam MEI                 = 5'h11;
    //machine trap setup
    wire [31:0] csr_misa;
    // reg [31:0] csr_medeleg;
    // reg [31:0] csr_mideleg;
    wire [31:0] csr_mie, csr_mtvec, csr_mcounteren;

    //machine trap handling
    wire [31:0] csr_mscratch;
    wire [31:0] csr_mstatus, csr_mepc, csr_mip, csr_mcause, csr_mtval;

    //machine memory protection

    //machine counter/timers
    wire [63:0] csr_cycle_count;
    wire [63:0] csr_instret_count; 

    //mtime是存储器地址空间映射的计时器
    wire [63:0] csr_time_count;
    wire [63:0] csr_time_cmp;

    //read only
    //wire [31:0] csr_misa;
    wire [31:0] csr_mvendorid, csr_marchid, csr_mimpid, csr_mhartid;

    wire mvendorid_rd_en;
    wire marchid_rd_en  ;
    wire mimpid_rd_en   ;
    wire mhartid_rd_en  ;
    wire [31:0] rdata_mvendorid, rdata_marchid, rdata_mimpid, rdata_mhartid;
    assign mvendorid_rd_en = (raddr == `CSR_MVENDORID_ADDR);
    assign marchid_rd_en   = (raddr == `CSR_MARCHID_ADDR);
    assign mimpid_rd_en    = (raddr == `CSR_MIMPID_ADDR);
    assign mhartid_rd_en   = (raddr == `CSR_MHARTID_ADDR);
    assign rdata_mvendorid = mvendorid_rd_en ? CSR_MVENDORID_PARAM : `WORD_DEASSERT;
    assign rdata_marchid   = marchid_rd_en   ? CSR_MARCHID_PARAM   : `WORD_DEASSERT;
    assign rdata_mimpid    = mimpid_rd_en    ? CSR_MIMPID_PARAM    : `WORD_DEASSERT;
    assign rdata_mhartid   = mhartid_rd_en   ? CSR_MHARTID_PARAM   : `WORD_DEASSERT;

    //cycle
    wire [31:0] rdata_cycle;
    wire [63:0] cycle_count_d;
    wire cycle_wr_en;
    wire cycle_rd_en;
    wire cycleh_rd_en;
    assign cycle_wr_en = 1'b1;
    assign cycle_rd_en  = (raddr == `CSR_CYCLE_ADDR);
    assign cycleh_rd_en = (raddr == `CSR_CYCLEH_ADDR);
    assign cycle_count_d = csr_cycle_count + 1'b1;
    stdffre #(64) ff_cycle_u (
        .clk(clk),
        .rstn(rstn),
        .en(cycle_wr_en),
        .d(cycle_count_d),
        .q(csr_cycle_count)
    );
    assign rdata_cycle = cycle_rd_en  ? csr_cycle_count[31:0] : (cycleh_rd_en ? csr_cycle_count[63:32] : `WORD_DEASSERT);

    //instret
    wire [31:0] rdata_instret;
    wire [63:0] instret_count_d;
    wire instret_wr_en;
    wire instret_rd_en;
    wire instreth_rd_en;
    assign instret_wr_en = ~instret_stall;
    assign instret_rd_en  = (raddr == `CSR_INSTRET_ADDR);
    assign instreth_rd_en = (raddr == `CSR_INSTRETH_ADDR);
    assign instret_count_d = csr_instret_count + 1'b1;
    stdffre #(64) ff_instret_u (
        .clk(clk),
        .rstn(rstn),
        .en(instret_wr_en),
        .d(instret_count_d),
        .q(csr_instret_count)
    );
    assign rdata_instret = instret_rd_en ? csr_instret_count[31:0] : (instreth_rd_en ? csr_instret_count[63:32] : `WORD_DEASSERT); 

    //time
    //fixme
    wire time_rd_en ;
    wire timeh_rd_en;
    wire [31:0] rdata_time;
    assign time_rd_en   = (raddr == `CSR_TIME_ADDR);
    assign timeh_rd_en  = (raddr == `CSR_TIMEH_ADDR);
    assign csr_time_count = time_val;
    assign rdata_time = time_rd_en ? csr_time_count[31:0] : (timeh_rd_en ? csr_time_count[63:32] : `WORD_DEASSERT);   

    //misa
    wire [31:0] rdata_misa;
    wire misa_wr_en;
    wire misa_rd_en;
    assign misa_wr_en = wren && (waddr == `CSR_MISA_ADDR);
    assign misa_rd_en = (raddr == `CSR_MISA_ADDR);
    stdffre #(32) ff_misa_u (
        .clk(clk),
        .rstn(rstn),
        .en(misa_wr_en),
        .d(wdata),
        .q(csr_misa)
    );
    assign rdata_misa = misa_rd_en ? csr_misa : `WORD_DEASSERT;

    //fixme
    //mstatus
    wire irq_en;
    wire mret_en;
    wire ext_irq_en, soft_irq_en, time_irq_en;
    wire mstatus_uie,   mstatus_sie,   mstatus_mie;
    wire mstatus_upie,  mstatus_spie,  mstatus_mpie;
    wire mstatus_uie_d, mstatus_sie_d, mstatus_mie_d;
    wire mstatus_upie_d,mstatus_spie_d,mstatus_mpie_d;
    wire mstatus_wr_en;
    wire mstatus_rd_en;
    wire mstatus_uie_sie_wr_en;
    wire mstatus_mie_wr_en;
    wire mstatus_upie_wr_en;
    wire mstatus_spie_wr_en;
    wire mstatus_mpie_wr_en;
    wire [31:0] rdata_mstatus;
    assign irq_en        = ext_irq_en | soft_irq_en | time_irq_en;
    assign mret_en       = iex_valid && iex_mret;
    assign mstatus_wr_en = wren && (waddr == `CSR_MSTATUS_ADDR);
    assign mstatus_rd_en = (raddr == `CSR_MSTATUS_ADDR);
    assign mstatus_uie_sie_wr_en = mstatus_wr_en | irq_en | ~mstatus_mie;
    assign mstatus_mie_wr_en     = mstatus_wr_en | irq_en | mret_en;
    assign mstatus_upie_wr_en    = irq_en | mret_en;
    assign mstatus_spie_wr_en    = mstatus_upie_wr_en;
    assign mstatus_mpie_wr_en    = mstatus_upie_wr_en;
    assign mstatus_mie_d = mstatus_wr_en ? wdata[3] 
                         : irq_en        ? 1'b0
                         : mret_en       ? mstatus_mpie
                         : mstatus_mie;
    assign mstatus_uie_d = mstatus_wr_en ? mstatus_mie_d && wdata[0]
                         : irq_en        ? 1'b0
                         : mstatus_uie;
    assign mstatus_sie_d = mstatus_wr_en ? mstatus_mie_d && wdata[1]
                         : irq_en        ? 1'b0
                         : mstatus_sie;
    assign mstatus_upie_d= irq_en        ? mstatus_uie
                         : mret_en       ? 1'b0
                         : mstatus_upie;     
    assign mstatus_spie_d= irq_en        ? mstatus_sie
                         : mret_en       ? 1'b0
                         : mstatus_spie;  
    assign mstatus_mpie_d= irq_en        ? mstatus_mie
                         : mret_en       ? 1'b1
                         : mstatus_mpie;    
    assign csr_mstatus   = {{24{1'b0}}, mstatus_mpie, 1'b0, mstatus_spie, mstatus_upie, mstatus_mie, 1'b0, mstatus_sie, mstatus_uie};
    assign rdata_mstatus = mstatus_rd_en ? csr_mstatus : `WORD_DEASSERT;
    stdffre #(1) ff_mstatus_uie_u (
        .clk(clk),
        .rstn(rstn),
        .en(mstatus_uie_sie_wr_en),
        .d(mstatus_uie_d),
        .q(mstatus_uie) 
    );
    stdffre #(1) ff_mstatus_sie_u (
        .clk(clk),
        .rstn(rstn),
        .en(mstatus_uie_sie_wr_en),
        .d(mstatus_sie_d),
        .q(mstatus_sie) 
    );
    stdffre #(1) ff_mstatus_mie_u (
        .clk(clk),
        .rstn(rstn),
        .en(mstatus_mie_wr_en),
        .d(mstatus_mie_d),
        .q(mstatus_mie) 
    );
    stdffre #(1) ff_mstatus_upie_u (
        .clk(clk),
        .rstn(rstn),
        .en(mstatus_upie_wr_en),
        .d(mstatus_upie_d),
        .q(mstatus_upie) 
    );
    stdffre #(1) ff_mstatus_spie_u (
        .clk(clk),
        .rstn(rstn),
        .en(mstatus_spie_wr_en),
        .d(mstatus_spie_d),
        .q(mstatus_spie) 
    );
    stdffre #(1) ff_mstatus_mpie_u (
        .clk(clk),
        .rstn(rstn),
        .en(mstatus_mpie_wr_en),
        .d(mstatus_mpie_d),
        .q(mstatus_mpie) 
    );

    //mie
    wire mie_wr_en;
    wire mie_rd_en;
    wire [31:0] rdata_mie;
    assign mie_wr_en = wren && (waddr == `CSR_MIE_ADDR);
    assign mie_rd_en = (raddr == `CSR_MIE_ADDR);
    stdffre #(32) ff_mie_u (
        .clk(clk),
        .rstn(rstn),
        .en(mie_wr_en),
        .d(wdata),
        .q(csr_mie) 
    );
    assign rdata_mie = mie_rd_en ? csr_mie : `WORD_DEASSERT;

    //mtvec
    wire mtvec_wr_en;
    wire mtvec_rd_en;
    wire [31:0] rdata_mtvec;
    assign mtvec_wr_en = wren && (waddr == `CSR_MTVEC_ADDR);
    assign mtvec_rd_en = (raddr == `CSR_MTVEC_ADDR);
    stdffre #(32) ff_mtvec_u (
        .clk(clk),
        .rstn(rstn),
        .en(mtvec_wr_en),
        .d(wdata),
        .q(csr_mtvec) 
    );
    assign rdata_mtvec = mtvec_rd_en ? csr_mtvec : `WORD_DEASSERT;

    //mcounteren
    wire mcounteren_wr_en;
    wire mcounteren_rd_en;
    wire [31:0] rdata_mcounteren;
    assign mcounteren_wr_en = wren && (waddr == `CSR_MCOUNTEREN_ADDR);
    assign mcounteren_rd_en = (raddr == `CSR_MCOUNTEREN_ADDR);
    stdffre #(32) ff_mcounteren_u (
        .clk(clk),
        .rstn(rstn),
        .en(mcounteren_wr_en),
        .d(wdata),
        .q(csr_mcounteren) 
    );
    assign rdata_mcounteren = mcounteren_rd_en ? csr_mcounteren : `WORD_DEASSERT;

    //mscratch
    wire mscratch_wr_en;
    wire mscratch_rd_en;
    wire [31:0] rdata_mscratch;
    assign mscratch_wr_en = wren && (waddr == `CSR_MSCRATCH_ADDR);
    assign mscratch_rd_en = (raddr == `CSR_MSCRATCH_ADDR);
    stdffre #(32) ff_mscratch_u (
        .clk(clk),
        .rstn(rstn),
        .en(mscratch_wr_en),
        .d(wdata),
        .q(csr_mscratch) 
    );
    assign rdata_mscratch = mscratch_rd_en ? csr_mscratch : `WORD_DEASSERT;

    //mepc
    wire mstatus_mie_valid;
    wire mie_meie, mie_msie, mie_mtie; 
    wire mepc_wr_en;
    wire mepc_rd_en;
    wire trap_en;
    wire [31:0] rdata_mepc;
    wire [31:0] mepc_flush_val;
    assign mepc_wr_en = wren && (waddr == `CSR_MEPC_ADDR);
    assign mepc_rd_en = (raddr == `CSR_MEPC_ADDR);
    assign mie_meie   = csr_mie[11];
    assign mie_msie   = csr_mie[3];
    assign mie_mtie   = csr_mie[7];
    assign mstatus_mie_valid = mstatus_mie && iex_valid && ~ac2csr_stall; 
    assign ext_irq_en = mstatus_mie_valid && mie_meie && ext_irq ;
    assign soft_irq_en= mstatus_mie_valid && mie_msie && soft_irq;
    assign time_irq_en= mstatus_mie_valid && mie_mtie && time_irq;
    assign trap_en    = irq_en | iex_except_valid;
    assign mepc_flush_val = iex_except_valid ? iex_pc : iex_next_pc;
    stdffref #(32) ff_mepc_u (
        .clk(clk),
        .rstn(rstn),
        .flush(trap_en),
        .flush_val(mepc_flush_val),
        .en(mepc_wr_en),
        .d(wdata),
        .q(csr_mepc) 
    );
    assign rdata_mepc = mepc_rd_en ? csr_mepc : `WORD_DEASSERT;

    //mcause
    reg [4:0] interrupt_code;
    wire [2:0] interrupt_code_sel;
    assign interrupt_code_sel = {time_irq_en,soft_irq_en,ext_irq_en};
    always @(*) begin
        case(interrupt_code_sel)
            3'b100                 : interrupt_code = MTI;
            3'b010, 3'b110         : interrupt_code = MSI;
            3'b001, 3'b011
          , 3'b101, 3'b111         : interrupt_code = MEI;
            default                : interrupt_code = RES;
        endcase
    end

    wire mcause_interrupt, mcause_trap_type; 
    wire [30:0] mcause_exception, mcause_trap_code;
    wire [31:0] rdata_mcause;
    wire mcause_wr_en;
    wire mcause_rd_en;
    wire mcause_trap_en;
    assign mcause_wr_en = wren && (waddr == `CSR_MCAUSE_ADDR);
    assign mcause_rd_en = (raddr == `CSR_MCAUSE_ADDR);
    assign mcause_trap_en         = trap_en;
    assign mcause_trap_type       = irq_en ? 1'b1 : 1'b0;
    assign mcause_trap_code       = irq_en ? {{(31-5){1'b0}}, interrupt_code} : {{(31-5){1'b0}}, iex_except_code};
    stdffref #(1) ff_mcause_interrupt_u (
        .clk(clk),
        .rstn(rstn),
        .flush(mcause_trap_en),
        .flush_val(mcause_trap_type),
        .en(mcause_wr_en),
        .d(wdata[31]),
        .q(mcause_interrupt) 
    );
    stdffref #(31) ff_mcause_exception_u (
        .clk(clk),
        .rstn(rstn),
        .flush(mcause_trap_en),
        .flush_val(mcause_trap_code),
        .en(mcause_wr_en),
        .d(wdata[30:0]),
        .q(mcause_exception) 
    );
    assign csr_mcause   = {mcause_interrupt, mcause_exception};
    assign rdata_mcause = mcause_rd_en ? csr_mcause : `WORD_DEASSERT;

    //mtval
    wire mtval_wr_en;
    wire mtval_rd_en;
    wire [31:0] rdata_mtval;
    // assign mtval_wr_en = wren && (waddr == `CSR_MTVAL_ADDR);
    assign mtval_wr_en = 1'b0;
    assign mtval_rd_en = (raddr == `CSR_MTVAL_ADDR);
    stdffre #(32) ff_mtval_u (
        .clk(clk),
        .rstn(rstn),
        .en(mtval_wr_en),
        .d(wdata),
        .q(csr_mtval) 
    );
    assign rdata_mtval = mtval_rd_en ? csr_mtval : `WORD_DEASSERT;

    //mip    
    wire mip_wr_en;
    wire mip_rd_en;
    wire [31:0] rdata_mip;
    wire meip, mtip, msip;
    reg  ssip, stip, seip;
    stdffre #(1) ff_mip_ssip_u (
        .clk(clk),
        .rstn(rstn),
        .en(mip_wr_en),
        .d(wdata[1]),
        .q(ssip) 
    );
    stdffre #(1) ff_mip_stip_u (
        .clk(clk),
        .rstn(rstn),
        .en(mip_wr_en),
        .d(wdata[5]),
        .q(stip) 
    );
    stdffre #(1) ff_mip_seip_u (
        .clk(clk),
        .rstn(rstn),
        .en(mip_wr_en),
        .d(wdata[9]),
        .q(seip) 
    );
    assign mip_wr_en = wren && (waddr == `CSR_MIP_ADDR);
    assign mip_rd_en = (raddr == `CSR_MIP_ADDR);
    assign meip      = ext_irq;
    assign mtip      = time_irq;
    assign msip      = soft_irq;
    assign csr_mip   = {{20{1'b0}}, meip, 1'b0, meip, 1'b0, mtip, 1'b0, stip, 1'b0, msip, 1'b0, ssip, 1'b0};
    assign rdata_mip = mip_rd_en ? csr_mip : `WORD_DEASSERT;

    wire raddr_eq_waddr;
    wire [31:0] rdata_mux;

    assign raddr_eq_waddr = wren && (raddr == waddr);

    assign rdata_mux = rdata_cycle     | rdata_instret | rdata_time
                     | rdata_mvendorid | rdata_marchid | rdata_mimpid | rdata_mhartid 
                     | rdata_mstatus   | rdata_misa    | rdata_mie    | rdata_mtvec | rdata_mcounteren 
                     | rdata_mscratch  | rdata_mepc    | rdata_mcause | rdata_mtval | rdata_mip;

    assign rdata = raddr_eq_waddr ? wdata : rdata_mux;

    assign trap_jump_valid = trap_en | mret_en;
    assign trap_jump_pc    = trap_en ? {csr_mtvec[31:2], 2'b00} : csr_mepc;

endmodule
