//===============================================
//Name          : urv_uncore
//Author        : angrybird
//Email         : 
//Date          : 2023-03-05
//Description   : urv_uncore, including dm, clint, plic, sys_mem_if(arb imem_if and dmem_if to sys_mem_if)
//===============================================

module urv_uncore 
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input  logic                    clk,
    input  logic                    rstn,
    // ndm rstn
    input  logic                    dm_rstn,
    output logic                    ndm_rst,
    // interrupt
    input  logic [PLIC_IRQ_N-1:0]   ext_irq_src,
    output logic                    ext_irq,
    output logic                    soft_irq,
    output logic                    time_irq,
    output logic [CSR_TIME_W-1:0]   time_val,
    // jtag signal todo
    input  logic                    jtag_tck,
    input  logic                    jtag_tms,
    input  logic                    jtag_tdi,
    output logic                    jtag_tdo,
    output logic                    jtag_tdo_en,
    input  logic                    jtag_trstn,
    // debug port
    input  logic                    sdp_ck,
    input  logic                    sdp_di,
    output logic                    sdp_do,
    output logic                    sdp_doen,
    input  logic                    sdp_rstn,    
    // data bus -> dm/clint/plic of uncore
    input  logic                    dmem_ocp_req_valid,
    output logic                    dmem_ocp_req_ready,
    input  mem_req_t                dmem_ocp_req,
    output logic                    dmem_ocp_resp_valid,
    input  logic                    dmem_ocp_resp_ready,
    output mem_resp_t               dmem_ocp_resp,
    // inst bus -> arb of uncore
    input  logic                    imem_sys_req_valid,
    output logic                    imem_sys_req_ready,
    input  mem_req_t                imem_sys_req,
    output logic                    imem_sys_resp_valid,
    input  logic                    imem_sys_resp_ready,
    output mem_resp_t               imem_sys_resp,
    // data bus -> arb of uncore
    input  logic                    dmem_sys_req_valid,
    output logic                    dmem_sys_req_ready,
    input  mem_req_t                dmem_sys_req,
    output logic                    dmem_sys_resp_valid,
    input  logic                    dmem_sys_resp_ready,
    output mem_resp_t               dmem_sys_resp,
    // debug port output -> core/pipeline
    output logic                    dp_req_valid,
    input  logic                    dp_req_ready,
    output mem_req_t                dp_req,
    input  logic                    dp_resp_valid,
    output logic                    dp_resp_ready,
    input  mem_resp_t               dp_resp,
    // arb output -> system
    output logic                    sys_req_valid,
    input  logic                    sys_req_ready,
    output mem_req_t                sys_req,
    input  logic                    sys_resp_valid,
    output logic                    sys_resp_ready,
    input  mem_resp_t               sys_resp
);

logic               dmem_ocp_req_valid_int;
logic               dmem_ocp_req_ready_int;
mem_req_t           dmem_ocp_req_int;

logic               dmem_ocp_resp_valid_int;
logic               dmem_ocp_resp_ready_int;
mem_resp_t          dmem_ocp_resp_int;

logic [2:0]         sn_req_valid;
logic [2:0]         sn_req_ready;
mem_req_t [2:0]     sn_req;

logic [2:0]         sn_resp_valid;
logic [2:0]         sn_resp_ready;
mem_resp_t [2:0]    sn_resp;

logic               clint_req_valid;
logic               clint_req_ready;
mem_req_t           clint_req;

logic               clint_resp_valid;
logic               clint_resp_ready;
mem_resp_t          clint_resp;

logic               plic_req_valid;
logic               plic_req_ready;
mem_req_t           plic_req;

logic               plic_resp_valid;
logic               plic_resp_ready;
mem_resp_t          plic_resp;

logic               dm_req_valid;
logic               dm_req_ready;
mem_req_t           dm_req;

logic               dm_resp_valid;
logic               dm_resp_ready;
mem_resp_t          dm_resp;

logic               sys_req_valid_int;
logic               sys_req_ready_int;
mem_req_t           sys_req_int;

logic               sys_resp_valid_int;
logic               sys_resp_ready_int;
mem_resp_t          sys_resp_int;

logic               cdc_dp_req_valid;
logic               cdc_dp_req_ready;
mem_req_t           cdc_dp_req;
logic               cdc_dp_resp_valid;
logic               cdc_dp_resp_ready;
mem_resp_t          cdc_dp_resp;

mem_noc_sync  #(
    .MOD            (1),
    .W              ($bits(mem_req_t))
) dmem_ocp_req_sync_u (
    .clk            (clk),
    .rstn           (rstn),
    .src_valid      (dmem_ocp_req_valid),
    .src_ready      (dmem_ocp_req_ready),
    .src            (dmem_ocp_req),
    .dst_valid      (dmem_ocp_req_valid_int),
    .dst_ready      (dmem_ocp_req_ready_int),
    .dst            (dmem_ocp_req_int)
);

mem_noc_sync  #(
    .MOD            (1),
    .W              ($bits(mem_resp_t))
)  dmem_ocp_resp_sync_u (
    .clk            (clk),
    .rstn           (rstn),
    .src_valid      (dmem_ocp_resp_valid_int),
    .src_ready      (dmem_ocp_resp_ready_int),
    .src            (dmem_ocp_resp_int),
    .dst_valid      (dmem_ocp_resp_valid),
    .dst_ready      (dmem_ocp_resp_ready),
    .dst            (dmem_ocp_resp)
);

mem_noc_router  #(
    .ENABLE_DEC_TAG(0)
    ,.ENABLE_DEC_IDX(1)
    ,.DEC_NUM(3)
    // ,.DEC_TAG_H(15)
    // ,.DEC_TAG_L(15)
    ,.DEC_IDX_H(14)
    ,.DEC_IDX_L(12)
    // ,.DEC_TAG_VAL(1'b0)
) mem_noc_router_1to3_u (
    .clk            (clk),
    .rstn           (rstn),

    //noc master 0 
    .mn_req_valid   (dmem_ocp_req_valid_int),
    .mn_req_ready   (dmem_ocp_req_ready_int),
    .mn_req         (dmem_ocp_req_int),

    .mn_resp_valid  (dmem_ocp_resp_valid_int),
    .mn_resp_ready  (dmem_ocp_resp_ready_int),
    .mn_resp        (dmem_ocp_resp_int),  

    //noc slave 0 to clint
    .sn_req_valid   (sn_req_valid),
    .sn_req_ready   (sn_req_ready),
    .sn_req         (sn_req),

    .sn_resp_valid  (sn_resp_valid),
    .sn_resp_ready  (sn_resp_ready),
    .sn_resp        (sn_resp)   
);

clint clint_u (
    .clk            (clk),
    .rstn           (rstn),
    // mem interface
    .mem_req_valid  (sn_req_valid[0]),
    .mem_req_ready  (sn_req_ready[0]),
    .mem_req        (sn_req[0]), 

    .mem_resp_valid (sn_resp_valid[0]),
    .mem_resp_ready (sn_resp_ready[0]),
    .mem_resp       (sn_resp[0]), 

    .soft_irq       (soft_irq),  
    .time_irq       (time_irq),
    .time_val       (time_val)        
);

plic plic_u (
    .clk            (clk),
    .rstn           (rstn),
    // mem interface
    .mem_req_valid  (sn_req_valid[1]),
    .mem_req_ready  (sn_req_ready[1]),
    .mem_req        (sn_req[1]), 

    .mem_resp_valid (sn_resp_valid[1]),
    .mem_resp_ready (sn_resp_ready[1]),
    .mem_resp       (sn_resp[1]), 

    .ext_irq_src    (ext_irq_src),
    .ext_irq        (ext_irq)        
);

// todo 
// debug module
// dm_top dm_top_u (
//     .clk(clk),
//     .rstn(rstn),
//     .dm_rstn(dm_rstn),
//     .ndm_rst(ndm_rst),

//     // jtag interface
//     .jtag_tck(jtag_tck),
//     .jtag_tms(jtag_tms),
//     .jtag_tdi(jtag_tdi),
//     .jtag_tdo(jtag_tdo),
//     .jtag_tdo_en(jtag_tdo_en),
//     .jtag_trstn(jtag_trstn),

//     // from core, config reg etc..
//     .mem_req_valid(sn_req_valid[2]),
//     .mem_req_ready(sn_req_ready[2]),
//     .mem_req(sn_req[2]),

//     .mem_resp_valid(sn_resp_valid[2]),
//     .mem_resp_ready(sn_resp_ready[2]),
//     .mem_resp(sn_resp[2]),

//     // to core, control pipe line etc..
//     .debug_int(debug_int),
//     .debug_req_valid(debug_req_valid),
//     .debug_req_ready(debug_req_ready),
//     .debug_req(debug_req),

//     .debug_resp_valid(debug_resp_valid),
//     .debug_resp_ready(debug_resp_ready),
//     .debug_resp(debug_resp)
// );
// todo
assign sn_req_ready[2]  = 'h1;
assign sn_resp_valid[2] = 'h1;
assign sn_resp[2]       = 'h0;
assign ndm_rst          = 'h0;
assign jtag_tdo         = 'h0;
assign jtag_tdo_en      = 'h0;

sdp2mem sdp2mem_u (
    .sdp_irq        (),

    // dp signal
    .sdp_ck         (sdp_ck),
    .sdp_di         (sdp_di),
    .sdp_do         (sdp_do),
    .sdp_doen       (sdp_doen),
    .sdp_rstn       (sdp_rstn),
    // mem interface
    .mem_req_valid  (cdc_dp_req_valid),
    .mem_req_ready  (cdc_dp_req_ready),
    .mem_req        (cdc_dp_req),
    .mem_resp_valid (cdc_dp_resp_valid),
    .mem_resp_ready (cdc_dp_resp_ready),
    .mem_resp       (cdc_dp_resp)  
);

// cdc from debug port clk domain to core clk domain 
cdc_2phase #(
    .T(mem_req_t)
) cdc_2phase_dp_req_u (
    .src_rst_ni     (sdp_rstn),
    .src_clk_i      (sdp_ck),
    .src_data_i     (cdc_dp_req),
    .src_valid_i    (cdc_dp_req_valid),
    .src_ready_o    (cdc_dp_req_ready),

    .dst_rst_ni     (rstn),
    .dst_clk_i      (clk),
    .dst_data_o     (dp_req),
    .dst_valid_o    (dp_req_valid),
    .dst_ready_i    (dp_req_ready)
);

cdc_2phase #(
    .T(mem_resp_t)
) cdc_2phase_dp_resp_u (
    .src_rst_ni     (rstn),
    .src_clk_i      (clk),
    .src_data_i     (dp_resp),
    .src_valid_i    (dp_resp_valid),
    .src_ready_o    (dp_resp_ready),

    .dst_rst_ni     (sdp_rstn),
    .dst_clk_i      (sdp_ck),
    .dst_data_o     (cdc_dp_resp),
    .dst_valid_o    (cdc_dp_resp_valid),
    .dst_ready_i    (cdc_dp_resp_ready)
);

// // cdc with async fifo
// cdc_mem_noc cdc_dp_mem_noc_u (
//     // src
//     .src_clk        (sdp_ck),
//     .src_rstn       (sdp_rstn),
//     .src_req_valid  (cdc_dp_req_valid),
//     .src_req_ready  (cdc_dp_req_ready),
//     .src_req        (cdc_dp_req),
//     .src_resp_valid (cdc_dp_resp_valid),
//     .src_resp_ready (cdc_dp_resp_ready),
//     .src_resp       (cdc_dp_resp),
//     // dest
//     .dest_clk       (clk),
//     .dest_rstn      (rstn),
//     .dest_req_valid (dp_req_valid),
//     .dest_req_ready (dp_req_ready),
//     .dest_req       (dp_req),
//     .dest_resp_valid(dp_resp_valid),
//     .dest_resp_ready(dp_resp_ready),
//     .dest_resp      (dp_resp)
// );

mem_noc_arb_2to1 sys_arb_u (
    .clk            (clk),
    .rstn           (rstn),
    //noc 0 
    .mn0_req_valid  (imem_sys_req_valid),
    .mn0_req_ready  (imem_sys_req_ready),
    .mn0_req        (imem_sys_req),
    .mn0_resp_valid (imem_sys_resp_valid),
    .mn0_resp_ready (imem_sys_resp_ready),
    .mn0_resp       (imem_sys_resp),
    .mn0_tid        (1'b0),
    //noc 1 
    .mn1_req_valid  (dmem_sys_req_valid),
    .mn1_req_ready  (dmem_sys_req_ready),
    .mn1_req        (dmem_sys_req),
    .mn1_resp_valid (dmem_sys_resp_valid),
    .mn1_resp_ready (dmem_sys_resp_ready),
    .mn1_resp       (dmem_sys_resp),
    .mn1_tid        (1'b0),
    //noc arb
    .sn_req_valid   (sys_req_valid_int),
    .sn_req_ready   (sys_req_ready_int),
    .sn_req         (sys_req_int),
    .sn_resp_valid  (sys_resp_valid_int),
    .sn_resp_ready  (sys_resp_ready_int),
    .sn_resp        (sys_resp_int),
    .sn_tid         (1'b0) 
);

// buffered sys_req/sys_resp
(* keep_hierarchy="yes" *)
mem_noc_sync #(
    .MOD            (1),
    .W              ($bits(mem_req_t))
) sys_req_sync_u (
    .clk            (clk),
    .rstn           (rstn),
    .src_valid      (sys_req_valid_int),
    .src_ready      (sys_req_ready_int),
    .src            (sys_req_int),
    .dst_valid      (sys_req_valid),
    .dst_ready      (sys_req_ready),
    .dst            (sys_req)
);
(* keep_hierarchy="yes" *)
mem_noc_sync #(
    .MOD            (1),
    .W              ($bits(mem_resp_t))
) sys_resp_sync_u (
    .clk            (clk),
    .rstn           (rstn),
    .src_valid      (sys_resp_valid),
    .src_ready      (sys_resp_ready),
    .src            (sys_resp),
    .dst_valid      (sys_resp_valid_int),
    .dst_ready      (sys_resp_ready_int),
    .dst            (sys_resp_int)
);

endmodule