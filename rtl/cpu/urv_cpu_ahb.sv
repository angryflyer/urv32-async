
module urv_cpu_ahb 
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
    //ahb master to ahb slave
    ,output logic [1:0]                  htrans
    ,output logic [2:0]                  hburst
    ,output logic [MEM_DATA_W-1:0]       haddr
    ,output logic [2:0]                  hsize
    ,output logic [MEM_DATA_W-1:0]       hwdata
    ,output logic                        hwrite
    ,output logic                        hmastlock
    ,output logic [6:0]                  hprot
    // from slave
    ,input  logic [MEM_DATA_W-1:0]       hrdata
    ,input  logic                        hresp
    ,input  logic                        hready
);

logic                        sys_req_valid;
logic                        sys_req_ready;
mem_req_t                    sys_req;

logic                        sys_resp_valid;
logic                        sys_resp_ready;
mem_resp_t                   sys_resp;

urv_cpu urv_cpu_u (
    .clk                    (clk),
    .core_rstn              (core_rstn),
    .cpu_rstn               (cpu_rstn),
    // todo 
    // dm/ndm rstn
    .dm_rstn                (dm_rstn),
    .ndm_rst                (ndm_rst),
    .rst_pc                 (rst_pc),
    .cfg_icache_en          (cfg_icache_en),
    .ext_irq_src            (ext_irq_src),
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
    // sys bus
    .sys_req_valid          (sys_req_valid),
    .sys_req_ready          (sys_req_ready),
    .sys_req                (sys_req),

    .sys_resp_valid         (sys_resp_valid),
    .sys_resp_ready         (sys_resp_ready),
    .sys_resp               (sys_resp)
);

mem2ahb mem2ahb_u (
    .clk                    (clk),
    .rstn                   (cpu_rstn),
    //from mem master
    .mem_req_valid          (sys_req_valid),
    .mem_req_ready          (sys_req_ready),
    .mem_req                (sys_req),

    .mem_resp_valid         (sys_resp_valid),
    .mem_resp_ready         (sys_resp_ready),
    .mem_resp               (sys_resp),
    //ahb master to ahb slave
    .htrans                 (htrans),
    .hburst                 (hburst),
    .haddr                  (haddr),
    .hsize                  (hsize),
    .hwdata                 (hwdata),
    .hwrite                 (hwrite),
    .hmastlock              (hmastlock),
    .hprot                  (hprot),
    // from slave
    .hrdata                 (hrdata),
    .hresp                  (hresp),
    .hready                 (hready)
);

endmodule