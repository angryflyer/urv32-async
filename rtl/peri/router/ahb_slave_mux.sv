//==============================================================================================
// Filename     : ahb_slave_mux.sv
// Author       : angrybird
// Email        : 
// Date         : 2023-01-25
// Description  : router 1 to 2.
//===============================================================================================

module ahb_slave_mux (
  
  input   logic         clk,
  input   logic         rstn,

  // input request
  input   logic [31:0]  s_haddr,
  input   logic         s_hsel,
  output  logic         s_hready,
  output  logic [31:0]  s_hrdata,
  output  logic         s_hresp,

  // output request to ahb1
  output  logic         s_hsel_1,
  input   logic         s_hreadyout_1,
  input   logic [31:0]  s_hrdata_1,
  input   logic         s_hresp_1,

  // output request to ahb2
  output  logic         s_hsel_2,
  input   logic         s_hreadyout_2,
  input   logic [31:0]  s_hrdata_2,
  input   logic         s_hresp_2,

  input   logic [31:0]  s_addr1,
  input   logic [31:0]  s_addr2
);

  logic rff_hsel_1;

  always_ff @(posedge clk or negedge rstn) begin
    if (rstn == 1'b0)
      rff_hsel_1 <= 1'b1;
    else if(s_hready)
      rff_hsel_1 <= s_hsel_1;
  end

  assign s_hsel_1 = ~s_hsel_2; // 32'h0 ~ 32'h4000_0000, 32'hf000_0000-32'hffff_ffff

  assign s_hsel_2 = ((s_haddr >= s_addr1) & (s_haddr < s_addr2));// 32'h4000_0000 ~ 32'hefff_ffff

  assign s_hready = rff_hsel_1 ? s_hreadyout_1  : s_hreadyout_2;
  assign s_hrdata = rff_hsel_1 ? s_hrdata_1     : s_hrdata_2;
  assign s_hresp  = rff_hsel_1 ? s_hresp_1      : s_hresp_2;

endmodule