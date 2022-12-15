//cdc mem bus
`include "inst_index.v"
`include "macro.v"

module cdc_mem_noc 
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input             src_clk,
    input             src_rstn,

    // src
    input             src_req_valid,
    output            src_req_ready,
    input  mem_req_t  src_req, 

    output            src_resp_valid,
    input             src_resp_ready,
    output mem_resp_t src_resp,  
    // dest
    input             dest_clk,
    input             dest_rstn,
    output            dest_req_valid,
    input             dest_req_ready,
    output mem_req_t  dest_req,

    input             dest_resp_valid, 
    output            dest_resp_ready,
    input  mem_resp_t dest_resp
);

    parameter MEM_REQ_T_W  = $bits(mem_req_t);
    parameter MEM_RESP_T_W = $bits(mem_resp_t);
    logic src_req_wr_en;
    logic [MEM_REQ_T_W-1:0] src_req_wr_data;
    logic src_req_full;
    logic src_req_afull;

    logic dest_req_rd_en;
    logic [MEM_REQ_T_W-1:0] dest_req_rd_data;
    logic dest_req_empty;
    logic dest_req_aempty;

    logic src_req_ready_rd_en;
    logic src_req_ready_rd_data;
    logic src_req_ready_empty;
    logic src_req_ready_aempty;

    logic dest_req_ready_wr_en;
    logic dest_req_ready_wr_data;
    logic dest_req_ready_full;
    logic dest_req_ready_afull;

    // assign src_req_wr_en   = src_req_valid && src_req_ready && ~src_req_full; 
    assign src_req_wr_en   = src_req_valid && src_req_ready && ~src_req_full;   
    // assign src_req_wr_en   = src_req_valid && ~src_req_full && ~src_req_afull;
    assign src_req_wr_data = src_req;

    assign dest_req_rd_en  = dest_req_valid && dest_req_ready;
    assign dest_req_valid  = ~dest_req_empty;
    assign dest_req        = dest_req_rd_data;

    assign dest_req_ready_wr_en   = dest_req_ready && ~dest_req_ready_full;
    assign dest_req_ready_wr_data = dest_req_ready;

    assign src_req_ready_rd_en    = ~src_req_ready_empty;
    assign src_req_ready          = src_req_ready_rd_en && src_req_ready_rd_data;        

    async_fifo #(
        .W         (MEM_REQ_T_W),
        .DP        (4),
        .WR_FAST   (1),
        .RD_FAST   (1)
    ) async_fifo_req_u (
        // Sync w.r.t WR clock
        .wr_clk    (src_clk),
        .wr_reset_n(src_rstn),
        .wr_en     (src_req_wr_en),
        .wr_data   (src_req_wr_data),
        .full      (src_req_full),                 
        .afull     (src_req_afull),                 

        // Sync w.r.t RD Clock
        .rd_clk    (dest_clk),
        .rd_reset_n(dest_rstn),
        .rd_en     (dest_req_rd_en),
        .empty     (dest_req_empty), // sync'ed to rd_clk
        .aempty    (dest_req_aempty), // sync'ed to rd_clk
        .rd_data   (dest_req_rd_data)        
    );

    async_fifo #(
        .W         (1),
        .DP        (4),
        .WR_FAST   (1),
        .RD_FAST   (1)
    ) async_fifo_req_ready_u (
        // Sync w.r.t WR clock
        .wr_clk    (dest_clk),
        .wr_reset_n(dest_rstn),
        .wr_en     (dest_req_ready_wr_en),
        .wr_data   (dest_req_ready_wr_data),
        .full      (dest_req_ready_full),                 
        .afull     (dest_req_ready_afull),                 

        // Sync w.r.t RD Clock
        .rd_clk    (src_clk),
        .rd_reset_n(src_rstn),
        .rd_en     (src_req_ready_rd_en),
        .empty     (src_req_ready_empty), // sync'ed to rd_clk
        .aempty    (src_req_ready_aempty), // sync'ed to rd_clk
        .rd_data   (src_req_ready_rd_data)        
    );

endmodule
