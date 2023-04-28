//存储器总线mem noc
//用于取指和数据存取，且可以访问同一个设备
`include "inst_index.v"
`include "macro.v"

module mem_noc 
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input  clk,
    input  rstn,

    // noc master 0
    input             mn0_req_valid,
    output            mn0_req_ready,
    input mem_req_t   mn0_req, 

    output            mn0_resp_valid,
    input             mn0_resp_ready,
    output mem_resp_t mn0_resp,
    // noc master 1
    input             mn1_req_valid,
    output            mn1_req_ready,
    input mem_req_t   mn1_req, 

    output            mn1_resp_valid, 
    input             mn1_resp_ready,
    output mem_resp_t mn1_resp, 

    //noc slave 0 
    input  [MEM_ADDR_W-1:0] sn0_base_addr,
    output            sn0_req_valid,
    input             sn0_req_ready,
    output mem_req_t  sn0_req, 

    input             sn0_resp_valid,
    output            sn0_resp_ready,
    input  mem_resp_t sn0_resp,

    //noc slave 1 
    input  [MEM_ADDR_W-1:0] sn1_base_addr,
    output            sn1_req_valid,
    input             sn1_req_ready,
    output mem_req_t  sn1_req, 

    input             sn1_resp_valid,
    output            sn1_resp_ready,
    input  mem_resp_t sn1_resp
);

    typedef enum logic {
        SNOC0 = 1'b0,
        SNOC1 = 1'b1
    } noc_tid_t;

    logic            mn0_sn0_req_valid;
    logic            mn0_sn0_req_ready;
    mem_req_t        mn0_sn0_req;

    logic            mn0_sn0_resp_valid;
    logic            mn0_sn0_resp_ready;
    mem_resp_t       mn0_sn0_resp;

    logic            mn0_sn1_req_valid;
    logic            mn0_sn1_req_ready;
    mem_req_t        mn0_sn1_req;

    logic            mn0_sn1_resp_valid;
    logic            mn0_sn1_resp_ready;
    mem_resp_t       mn0_sn1_resp;

    logic            mn1_sn0_req_valid;
    logic            mn1_sn0_req_ready;
    mem_req_t        mn1_sn0_req; 

    logic            mn1_sn0_resp_valid;
    logic            mn1_sn0_resp_ready;
    mem_resp_t       mn1_sn0_resp;

    logic            mn1_sn1_req_valid;
    logic            mn1_sn1_req_ready;
    mem_req_t        mn1_sn1_req;

    logic            mn1_sn1_resp_valid; 
    logic            mn1_sn1_resp_ready;
    mem_resp_t       mn1_sn1_resp;

    logic            mn0_sn0_tid, mn0_sn1_tid;
    logic            mn1_sn0_tid, mn1_sn1_tid;

    localparam DEC_TAG_VAL = `RAM_BASE_ADDR;
    mem_noc_router_1to2 #(
        .DEC_NUM(2),
        .DEC_TAG_H(31),
        .DEC_TAG_L(16),
        .DEC_TAG_VAL(DEC_TAG_VAL[31:16])
    ) mn0_router_1to2_u (
        .clk(clk),
        .rstn(rstn),
        //noc master 0 
        .mn_req_valid(mn0_req_valid),
        .mn_req_ready(mn0_req_ready),
        .mn_req(mn0_req),
        .mn_resp_valid(mn0_resp_valid),
        .mn_resp_ready(mn0_resp_ready),
        .mn_resp(mn0_resp),
        //noc slave 0 
        .sn0_req_valid(mn0_sn0_req_valid),
        .sn0_req_ready(mn0_sn0_req_ready),
        .sn0_req(mn0_sn0_req),
        .sn0_resp_valid(mn0_sn0_resp_valid),
        .sn0_resp_ready(mn0_sn0_resp_ready),
        .sn0_resp(mn0_sn0_resp),
        .sn0_tid(mn0_sn0_tid),
        //noc slave 1 
        .sn1_req_valid(mn0_sn1_req_valid),
        .sn1_req_ready(mn0_sn1_req_ready),
        .sn1_req(mn0_sn1_req),
        .sn1_resp_valid(mn0_sn1_resp_valid),
        .sn1_resp_ready(mn0_sn1_resp_ready),
        .sn1_resp(mn0_sn1_resp),
        .sn1_tid(mn0_sn1_tid)        
    );

    mem_noc_router_1to2 #(
        .DEC_NUM(2),
        .DEC_TAG_H(31),
        .DEC_TAG_L(16),
        .DEC_TAG_VAL(DEC_TAG_VAL[31:16])
    ) mn1_router_1to2_u (
        .clk(clk),
        .rstn(rstn),
        //noc master 0 
        .mn_req_valid(mn1_req_valid),
        .mn_req_ready(mn1_req_ready),
        .mn_req(mn1_req),
        .mn_resp_valid(mn1_resp_valid),
        .mn_resp_ready(mn1_resp_ready),
        .mn_resp(mn1_resp),
        //noc slave 0 
        .sn0_req_valid(mn1_sn0_req_valid),
        .sn0_req_ready(mn1_sn0_req_ready),
        .sn0_req(mn1_sn0_req),
        .sn0_resp_valid(mn1_sn0_resp_valid),
        .sn0_resp_ready(mn1_sn0_resp_ready),
        .sn0_resp(mn1_sn0_resp),
        .sn0_tid(mn1_sn0_tid),
        //noc slave 1 
        .sn1_req_valid(mn1_sn1_req_valid),
        .sn1_req_ready(mn1_sn1_req_ready),
        .sn1_req(mn1_sn1_req),
        .sn1_resp_valid(mn1_sn1_resp_valid),
        .sn1_resp_ready(mn1_sn1_resp_ready),
        .sn1_resp(mn1_sn1_resp),
        .sn1_tid(mn1_sn1_tid)      
    );

    mem_noc_arb_2to1 sn0_arb_2to1_u (
        .clk(clk),
        .rstn(rstn),
        //noc 0 
        .mn0_req_valid(mn0_sn0_req_valid),
        .mn0_req_ready(mn0_sn0_req_ready),
        .mn0_req(mn0_sn0_req),
        .mn0_resp_valid(mn0_sn0_resp_valid),
        .mn0_resp_ready(mn0_sn0_resp_ready),
        .mn0_resp(mn0_sn0_resp),
        .mn0_tid(mn0_sn0_tid),
        //noc 1 
        .mn1_req_valid(mn1_sn0_req_valid),
        .mn1_req_ready(mn1_sn0_req_ready),
        .mn1_req(mn1_sn0_req),
        .mn1_resp_valid(mn1_sn0_resp_valid),
        .mn1_resp_ready(mn1_sn0_resp_ready),
        .mn1_resp(mn1_sn0_resp),
        .mn1_tid(mn1_sn0_tid),
        //noc arb
        .sn_req_valid(sn0_req_valid),
        .sn_req_ready(sn0_req_ready),
        .sn_req(sn0_req),
        .sn_resp_valid(sn0_resp_valid),
        .sn_resp_ready(sn0_resp_ready),
        .sn_resp(sn0_resp),
        .sn_tid(SNOC0)       
    );

    mem_noc_arb_2to1 sn1_arb_2to1_u (
        .clk(clk),
        .rstn(rstn),
        //noc 0 
        .mn0_req_valid(mn0_sn1_req_valid),
        .mn0_req_ready(mn0_sn1_req_ready),
        .mn0_req(mn0_sn1_req),
        .mn0_resp_valid(mn0_sn1_resp_valid),
        .mn0_resp_ready(mn0_sn1_resp_ready),
        .mn0_resp(mn0_sn1_resp),
        .mn0_tid(mn0_sn1_tid),
        //noc 1 
        .mn1_req_valid(mn1_sn1_req_valid),
        .mn1_req_ready(mn1_sn1_req_ready),
        .mn1_req(mn1_sn1_req),
        .mn1_resp_valid(mn1_sn1_resp_valid),
        .mn1_resp_ready(mn1_sn1_resp_ready),
        .mn1_resp(mn1_sn1_resp),
        .mn1_tid(mn1_sn1_tid),
        //noc arb
        .sn_req_valid(sn1_req_valid),
        .sn_req_ready(sn1_req_ready),
        .sn_req(sn1_req),
        .sn_resp_valid(sn1_resp_valid),
        .sn_resp_ready(sn1_resp_ready),
        .sn_resp(sn1_resp),
        .sn_tid(SNOC1)         
    );

endmodule
