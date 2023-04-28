//===============================================
//Name          : apb_rst_gen
//Author        : angrybird
//Email         : 
//Date          : 2023-04-22
//Description   : apb interface
//              : generate reset signal
//              : generate reset pc
//              : support rewrite rst and rst pc
//              : support read dfx info, such as pc, gpr and csr etc,.
//-----------------------------------------------
//===============================================
module apb_rst_gen #(
    parameter N_AW      = 32,
    parameter N_DW      = 32,
    parameter N_PW      = 32,
    parameter N_CORE    = 4,
    parameter N_REG_AW  = 12,
    parameter N_DFX     = 512,
    parameter RST_BASE_ADDR = 'hFFFF_0000,
    parameter DFX_BASE_ADDR = 'hFFFF_1000
)(
// --------------------------------------------------------------------------
// Port Definitions
// --------------------------------------------------------------------------
  input  logic              pclk,     // Clock
  input  logic              presetn,  // Reset

  input  logic              psel,     // Device select
  input  logic [N_AW-1:0]   paddr,    // Address
  input  logic              penable,  // Transfer control
  input  logic              pwrite,   // Write control
  input  logic [N_DW-1:0]   pwdata,   // Write data

  // AMBA_APB4
  input  logic [2:0]        pprot,    
  input  logic [3:0]        pstrb,    // Write strobe

  output logic [N_DW-1:0]   prdata,   // Read data
  output logic              pready,   // Device ready
  output logic              pslverr,  // Device error response

  // cfg input
  input  logic [N_PW-1:0]   rst_boot_pc, // from bootmode
  // todo
  // dfx info
  input  logic [N_DFX-1 :0] dfx_info,
  // Generate Reset and Reset PC
  output logic              rst_cpu_en,
  // core rstn
  output logic [N_CORE-1:0] rst_core_en,    // core rst
  output logic              rst_uncore_en,  // core rst
  output logic              rst_core_pc_en, // tst_core_pc_en
  output logic [N_PW-1:0]   rst_core_pc     // rst_core_pc
);

logic            is_slv_addr_match;
logic            is_dfx_match;

logic            is_reg_00;
logic            is_reg_01;
logic            is_reg_dfx;

logic            ff_wr_en;
logic            ff_rd_en;

logic            ff_rst_core_pc_wr_en;
logic            ff_rst_core_pc_ctrl_wr_en;

logic            ff_rst_core_pc_rd_en;
logic            ff_rst_core_pc_ctrl_rd_en;

logic [N_PW-1:0] ff_rst_core_pc;
logic [N_DW-1:0] ff_rst_core_pc_ctrl;

logic [N_DW-1:0] reg_out_mux;

assign is_slv_addr_match         = paddr[N_AW-1:16] == (BASE_ADDR >> 16);
assign is_dfx_match              = paddr[15:12]     == DFX_BASE_ADDR[15:12];

assign ff_wr_en                  = is_slv_addr_match & psel & ~penable & pwrite;
assign ff_rd_en                  = is_slv_addr_match & psel & ~pwrite;

assign is_reg_00                 = ~is_dfx_match & (paddr[N_REG_AW-1:2] == 'h0);
assign is_reg_01                 = ~is_dfx_match & (paddr[N_REG_AW-1:2] == 'h1);

assign ff_rst_core_pc_wr_en      = ff_wr_en & is_reg_00;
assign ff_rst_core_pc_ctrl_wr_en = ff_wr_en & is_reg_01;

assign ff_rst_core_pc_rd_en      = ff_rd_en & is_reg_00;
assign ff_rst_core_pc_ctrl_rd_en = ff_rd_en & is_reg_01;

stdffrve #(N_PW) ff_rst_core_pc_u (
    .clk(pclk),
    .rstn(presetn),
    .rst_val(rst_boot_pc),
    .en(ff_rst_core_pc_wr_en),
    .d(pwdata),
    .q(ff_rst_core_pc)
);

stdffre #(N_DW) ff_rst_core_pc_ctrl_u (
    .clk(pclk),
    .rstn(presetn),
    .en(ff_rst_core_pc_ctrl_wr_en),
    .d(pwdata),
    .q(ff_rst_core_pc_ctrl)
);

assign reg_out_mux    = ff_rst_core_pc_rd_en      ? ff_rst_core_pc 
                      : ff_rst_core_pc_ctrl_rd_en ? ff_rst_core_pc_ctrl
                      : 'h0;

stdffre #(N_DW) ff_reg_readout_u (
    .clk(pclk),
    .rstn(presetn),
    .en(ff_rd_en),
    .d(reg_out_mux),
    .q(prdata)
);

assign rst_core_pc    = ff_rst_core_pc;
assign rst_core_pc_en = ff_rst_core_pc_ctrl[N_DW-1];
assign rst_core_en    = ff_rst_core_pc_ctrl[N_CORE-1:0];
assign rst_uncore_en  = ff_rst_core_pc_ctrl[N_CORE];
assign rst_cpu_en     = ff_rst_core_pc_ctrl[N_CORE+1];

assign pready         = 1'b1;
assign pslverr        = 1'b0;

endmodule