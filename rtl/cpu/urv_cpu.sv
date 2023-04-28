
module urv_cpu 
    import urv_cfg::*;
    import urv_typedef::*;
(
     input  logic                        clk
    ,input  logic                        core_rstn    // core resetn, including pipeline, imem_router and icache_ctrl
    ,input  logic                        cpu_rstn     // cpu subsystem resetn
    ,input  logic                        dm_rstn      // reset debug module
    ,output logic                        ndm_rst      // reset non-debug module
    ,input  logic [MEM_ADDR_W-1:0]       rst_pc
    ,input  logic                        cfg_icache_en // icache active high
    // ext irq src
    ,input  logic [PLIC_IRQ_N-1:0]       ext_irq_src
    // jtag
    ,input  logic                        jtag_tck
    ,input  logic                        jtag_tms
    ,input  logic                        jtag_tdi
    ,output logic                        jtag_tdo
    ,output logic                        jtag_tdo_en
    ,input  logic                        jtag_trstn
    // debug port
    ,input  logic                        sdp_ck
    ,input  logic                        sdp_di
    ,output logic                        sdp_do
    ,output logic                        sdp_doen
    ,input  logic                        sdp_rstn
    // system bus
    ,output logic                        sys_req_valid
    ,input  logic                        sys_req_ready
    ,output mem_req_t                    sys_req

    ,input  logic                        sys_resp_valid
    ,output logic                        sys_resp_ready
    ,input  mem_resp_t                   sys_resp
);

// to clint/plic/dm of uncore
// on cpu periperial
logic                       dmem_ocp_req_valid;
logic                       dmem_ocp_req_ready;
mem_req_t                   dmem_ocp_req; 

logic                       dmem_ocp_resp_valid;
logic                       dmem_ocp_resp_ready;
mem_resp_t                  dmem_ocp_resp;

// to core/pipeline
logic                       dp_req_valid;
logic                       dp_req_ready;
mem_req_t                   dp_req;

logic                       dp_resp_valid;
logic                       dp_resp_ready;
mem_resp_t                  dp_resp;

// to sys arb of uncore
logic                       dmem_sys_req_valid;
logic                       dmem_sys_req_ready;
mem_req_t                   dmem_sys_req; 

logic                       dmem_sys_resp_valid;
logic                       dmem_sys_resp_ready;
mem_resp_t                  dmem_sys_resp;

// todo
// uon-cacheable to sys arb of uncore
logic                       imem_sys_req_valid;
logic                       imem_sys_req_ready;
mem_req_t                   imem_sys_req; 

logic                       imem_sys_resp_valid;
logic                       imem_sys_resp_ready;
mem_resp_t                  imem_sys_resp;

// to sys arb of uncore
logic                       icache_sys_req_valid;
logic                       icache_sys_req_ready;
mem_req_t                   icache_sys_req; 

logic                       icache_sys_resp_valid;
logic                       icache_sys_resp_ready;
mem_resp_t                  icache_sys_resp;

// logic                       sys_req_valid;
// logic                       sys_req_ready;
// mem_req_t                   sys_req;

// logic                       sys_resp_valid;
// logic                       sys_resp_ready;
// mem_resp_t                  sys_resp;

logic                       ext_irq;
logic                       soft_irq;
logic                       time_irq;
logic [CSR_TIME_W-1:0]      time_val;

urv_core urv_core_u (
    .clk                    (clk),
    .core_rstn              (core_rstn),
    .cpu_rstn               (cpu_rstn),
    .rst_pc                 (rst_pc),
    .cfg_icache_en          (cfg_icache_en),

    .ext_irq                (ext_irq),
    // .ext_irq                (1'b0),
    .soft_irq               (soft_irq),  
    .time_irq               (time_irq),
    .time_val               (time_val), 
    // to clint/plic/dm of uncore
    .dmem_ocp_req_valid     (dmem_ocp_req_valid),
    .dmem_ocp_req_ready     (dmem_ocp_req_ready),
    .dmem_ocp_req           (dmem_ocp_req),     
    .dmem_ocp_resp_valid    (dmem_ocp_resp_valid),
    .dmem_ocp_resp_ready    (dmem_ocp_resp_ready),
    .dmem_ocp_resp          (dmem_ocp_resp), 
    // from uncore to core/pipeline
    .dp_req_valid           (dp_req_valid),
    .dp_req_ready           (dp_req_ready),
    .dp_req                 (dp_req),
    .dp_resp_valid          (dp_resp_valid),
    .dp_resp_ready          (dp_resp_ready),
    .dp_resp                (dp_resp),
    // to sys arb of uncore
    .dmem_sys_req_valid     (dmem_sys_req_valid),
    .dmem_sys_req_ready     (dmem_sys_req_ready),
    .dmem_sys_req           (dmem_sys_req), 
    .dmem_sys_resp_valid    (dmem_sys_resp_valid),
    .dmem_sys_resp_ready    (dmem_sys_resp_ready),
    .dmem_sys_resp          (dmem_sys_resp),
    // todo
    // non-cacheable to sys arb of uncore
    .imem_sys_req_valid     (imem_sys_req_valid),
    .imem_sys_req_ready     (imem_sys_req_ready),
    .imem_sys_req           (imem_sys_req), 
    .imem_sys_resp_valid    (imem_sys_resp_valid),
    .imem_sys_resp_ready    (imem_sys_resp_ready),
    .imem_sys_resp          (imem_sys_resp),
    // non-cacheable to sys arb of uncore
    .icache_sys_req_valid   (icache_sys_req_valid),
    .icache_sys_req_ready   (icache_sys_req_ready),
    .icache_sys_req         (icache_sys_req), 
    .icache_sys_resp_valid  (icache_sys_resp_valid),
    .icache_sys_resp_ready  (icache_sys_resp_ready),
    .icache_sys_resp        (icache_sys_resp)
);

urv_uncore urv_uncore_u (
    .clk                    (clk),
    .rstn                   (cpu_rstn),
    // dm/ndm rstn
    .dm_rstn                (dm_rstn),
    .ndm_rst                (ndm_rst), 
    // interrupt
    .ext_irq_src            (ext_irq_src),
    .ext_irq                (ext_irq),
    .soft_irq               (soft_irq),
    .time_irq               (time_irq),
    .time_val               (time_val),
    // todo
    // jtag signal
    .jtag_tck               (jtag_tck),
    .jtag_tms               (jtag_tms),
    .jtag_tdi               (jtag_tdi),
    .jtag_tdo               (jtag_tdo),
    .jtag_tdo_en            (jtag_tdo_en),
    .jtag_trstn             (jtag_trstn),
    // dp signal
    .sdp_ck                 (sdp_ck),
    .sdp_di                 (sdp_di),
    .sdp_do                 (sdp_do),
    .sdp_doen               (sdp_doen),
    .sdp_rstn               (sdp_rstn),
    // data bus -> dm/clint/plic of uncore
    .dmem_ocp_req_valid     (dmem_ocp_req_valid),
    .dmem_ocp_req_ready     (dmem_ocp_req_ready),
    .dmem_ocp_req           (dmem_ocp_req),
    .dmem_ocp_resp_valid    (dmem_ocp_resp_valid),
    .dmem_ocp_resp_ready    (dmem_ocp_resp_ready),
    .dmem_ocp_resp          (dmem_ocp_resp),

    // inst bus -> arb of uncore
    .imem_sys_req_valid     (icache_sys_req_valid),
    .imem_sys_req_ready     (icache_sys_req_ready),
    .imem_sys_req           (icache_sys_req),
    .imem_sys_resp_valid    (icache_sys_resp_valid),
    .imem_sys_resp_ready    (icache_sys_resp_ready),
    .imem_sys_resp          (icache_sys_resp),

    // // for debug
    // .imem_sys_req_valid     (1'b0),
    // .imem_sys_req_ready     (icache_sys_ready),
    // .imem_sys_req           (icache_sys_req),
    // .imem_sys_resp_valid    (icache_sys_resp_valid),
    // .imem_sys_resp_ready    (1'b1),
    // .imem_sys_resp          (icache_sys_resp),

    // data bus -> arb of uncore
    .dmem_sys_req_valid     (dmem_sys_req_valid),
    .dmem_sys_req_ready     (dmem_sys_req_ready),
    .dmem_sys_req           (dmem_sys_req),
    .dmem_sys_resp_valid    (dmem_sys_resp_valid),
    .dmem_sys_resp_ready    (dmem_sys_resp_ready),
    .dmem_sys_resp          (dmem_sys_resp),

    // from debug port to core/pipeline
    .dp_req_valid           (dp_req_valid),
    .dp_req_ready           (dp_req_ready),
    .dp_req                 (dp_req),
    .dp_resp_valid          (dp_resp_valid),
    .dp_resp_ready          (dp_resp_ready),
    .dp_resp                (dp_resp),

    // arb output -> system
    .sys_req_valid          (sys_req_valid),
    .sys_req_ready          (sys_req_ready),
    .sys_req                (sys_req),
    .sys_resp_valid         (sys_resp_valid),
    .sys_resp_ready         (sys_resp_ready),
    .sys_resp               (sys_resp)
);

endmodule