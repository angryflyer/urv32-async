// todo 
// need support burst
module urv_cpu_wb 
    import urv_cfg::*;
    import urv_typedef::*; 
(
     input  logic                        clk
    ,input  logic                        rstn
    ,input  logic                        dm_rstn       // reset debug module
    ,output logic                        ndm_rst      // reset non-debug module
    ,input  logic [MEM_ADDR_W-1:0]       rst_pc
    ,input  logic                        cfg_icache_en // wishbone bridge do not supports burst for now, cfg_icache_en should be 1'b0 here
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
    // wishbone
    //wishbone
    ,output logic                        wb_stb_o	
    ,output logic [MEM_ADDR_W-1:0]       wb_addr_o	 
    ,output logic                        wb_we_o         
    ,output logic [MEM_DATA_W-1:0]       wb_data_o	
    ,output logic [MEM_MASK_W-1:0]       wb_sel_o
    ,output logic                        wb_cyc_o 
    ,input  logic                        wb_ack_i
    ,input  logic                        wb_err_i //not used here	
    ,input  logic [MEM_DATA_W-1:0]       wb_data_i
);

logic                        sys_req_valid;
logic                        sys_req_ready;
mem_req_t                    sys_req;

logic                        sys_resp_valid;
logic                        sys_resp_ready;
mem_resp_t                   sys_resp;

urv_cpu urv_cpu_u (
    .clk                    (clk),
    .rstn                   (rstn),
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
    .rstn                   (rstn),
    //from mem master
    .mem_req_valid          (sys_req_valid),
    .mem_req_ready          (sys_req_ready),
    .mem_req                (sys_req),

    .mem_resp_valid         (sys_resp_valid),
    .mem_resp_ready         (sys_resp_ready),
    .mem_resp               (sys_resp),
    //wishbone
    .wb_stb_o               (wb_stb_o),	
    .wb_addr_o              (wb_addr_o),	 
    .wb_we_o                (wb_we_o),         
    .wb_data_o              (wb_data_o),	
    .wb_sel_o               (wb_sel_o),
    .wb_cyc_o               (wb_cyc_o), 
    .wb_ack_i               (wb_ack_i),
    .wb_err_i               (wb_err_i), //not used here	
    .wb_data_i              (wb_data_i),
);

endmodule