//流水线译码测试
`include "inst_index.v"
`include "macro.v"

module urv_core
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input  logic                    clk,
    input  logic                    core_rstn,     // core resetn, including pipeline, imem_router and icache_ctrl
    input  logic                    cpu_rstn,      // cpu subsystem resetn
    input  logic [MEM_ADDR_W-1:0]   rst_pc,
    input  logic                    cfg_icache_en, // icache active high

    input  logic                    ext_irq,
    input  logic                    soft_irq,
    input  logic                    time_irq,
    input  logic [CSR_TIME_W-1:0]   time_val,
    // from debug port output -> core/pipeline
    input  logic                    dp_req_valid,
    output logic                    dp_req_ready,
    input  mem_req_t                dp_req,
    output logic                    dp_resp_valid,
    input  logic                    dp_resp_ready,
    output mem_resp_t               dp_resp,
    // to clint/plic/dm of uncore
    // ocp(on cpu periperial)
    output logic                    dmem_ocp_req_valid,
    input  logic                    dmem_ocp_req_ready,
    output mem_req_t                dmem_ocp_req,

    input  logic                    dmem_ocp_resp_valid,
    output logic                    dmem_ocp_resp_ready,
    input  mem_resp_t               dmem_ocp_resp,
    // to sys arb of uncore
    output logic                    dmem_sys_req_valid,
    input  logic                    dmem_sys_req_ready,
    output mem_req_t                dmem_sys_req, 

    input  logic                    dmem_sys_resp_valid,
    output logic                    dmem_sys_resp_ready,
    input  mem_resp_t               dmem_sys_resp,

    // todo
    // non-cacheable to sys arb of uncore
    output logic                    imem_sys_req_valid,
    input  logic                    imem_sys_req_ready,
    output mem_req_t                imem_sys_req, 

    input  logic                    imem_sys_resp_valid,
    output logic                    imem_sys_resp_ready,
    input  mem_resp_t               imem_sys_resp,

    // to sys arb of uncore
    output logic                    icache_sys_req_valid,
    input  logic                    icache_sys_req_ready,
    output mem_req_t                icache_sys_req, 

    input  logic                    icache_sys_resp_valid,
    output logic                    icache_sys_resp_ready,
    input  mem_resp_t               icache_sys_resp
);

parameter DTCM_SIZE  = DTCM_WIDTH_IN_BYTE >> 2;
parameter ITCM_SIZE  = ITCM_WIDTH_IN_BYTE >> 2;

logic               imem_req_valid;
logic               imem_req_ready;
mem_req_t           imem_req;

logic               imem_resp_valid;
logic               imem_resp_ready;
mem_resp_t          imem_resp;

logic               dmem_req_valid;
logic               dmem_req_ready;
mem_req_t           dmem_req;

logic               dmem_resp_valid;
logic               dmem_resp_ready;
mem_resp_t          dmem_resp;

logic               dmem_debug_req_valid;
logic               dmem_debug_req_ready;
mem_req_t           dmem_debug_req;

logic               dmem_debug_resp_valid;
logic               dmem_debug_resp_ready;
mem_resp_t          dmem_debug_resp;

logic               dmem_itcm_req_valid;
logic               dmem_itcm_req_ready;
mem_req_t           dmem_itcm_req;

logic               dmem_itcm_resp_valid;
logic               dmem_itcm_resp_ready;
mem_resp_t          dmem_itcm_resp;

logic               dmem_dtcm_req_valid;
logic               dmem_dtcm_req_ready;
mem_req_t           dmem_dtcm_req;

logic               dmem_dtcm_resp_valid;
logic               dmem_dtcm_resp_ready;
mem_resp_t          dmem_dtcm_resp;

logic               imem_itcm_req_valid;
logic               imem_itcm_req_ready;
mem_req_t           imem_itcm_req;

logic               imem_itcm_resp_valid;
logic               imem_itcm_resp_ready;
mem_resp_t          imem_itcm_resp;

logic               imem_icache_req_valid;
logic               imem_icache_req_ready;
mem_req_t           imem_icache_req;

logic               imem_icache_resp_valid;
logic               imem_icache_resp_ready;
mem_resp_t          imem_icache_resp;

logic               itcm_req_valid;
logic               itcm_req_ready;
mem_req_t           itcm_req;

logic               itcm_resp_valid;
logic               itcm_resp_ready;
mem_resp_t          itcm_resp;

logic               sram_req_valid;  // strobe/request
logic               sram_req_ready;
mem_req_t           sram_req;  // strobe/request

logic               sram_resp_valid;  // strobe/request
logic               sram_resp_ready;
mem_resp_t          sram_resp;  // strobe/request

logic [CACHE_WAY_NUM-1:0]                           tag_ram_csn;
logic [CACHE_WAY_NUM-1:0]                           tag_ram_wen;
logic [CACHE_WAY_NUM-1:0][CACHE_TAG_MASK_W-1:0]     tag_ram_web;
logic [CACHE_WAY_NUM-1:0][CACHE_INDEX_W-1+2:2]      tag_ram_addr;
logic [CACHE_WAY_NUM-1:0][CACHE_TAG_W-1:0]          tag_ram_din;
logic [CACHE_WAY_NUM-1:0][CACHE_TAG_W-1:0]          tag_ram_dout;

// to data_ram    
logic [CACHE_WAY_NUM-1:0]                           data_ram_csn;
logic [CACHE_WAY_NUM-1:0]                           data_ram_wen;
logic [CACHE_WAY_NUM-1:0][CACHE_DATA_MASK_W-1:0]    data_ram_web;
logic [CACHE_WAY_NUM-1:0][CACHE_INDEX_W-1+2:2]      data_ram_addr;
logic [CACHE_WAY_NUM-1:0][CACHE_DATA_W-1:0]         data_ram_din;
logic [CACHE_WAY_NUM-1:0][CACHE_DATA_W-1:0]         data_ram_dout;

urv_pipe urv_pipe_u (
    .clk                (clk),
    .rstn               (core_rstn),
    .rst_pc             (rst_pc),

    .ext_irq            (ext_irq),
    .soft_irq           (soft_irq),  
    .time_irq           (time_irq),
    .time_val           (time_val), 
    // instruction fetch bus
    .imem_req_valid     (imem_req_valid),
    .imem_req_ready     (imem_req_ready),
    .imem_req           (imem_req),     
    .imem_resp_valid    (imem_resp_valid),
    .imem_resp_ready    (imem_resp_ready),
    .imem_resp          (imem_resp), 
    // memery bus
    .dmem_req_valid     (dmem_req_valid),
    .dmem_req_ready     (dmem_req_ready),
    .dmem_req           (dmem_req), 
    .dmem_resp_valid    (dmem_resp_valid),
    .dmem_resp_ready    (dmem_resp_ready),
    .dmem_resp          (dmem_resp)
);

mem_noc_arb_2to1 dmem_noc_arb_u (
    .clk                (clk),
    .rstn               (cpu_rstn),
    //noc 0 
    .mn0_req_valid      (dmem_req_valid),
    .mn0_req_ready      (dmem_req_ready),
    .mn0_req            (dmem_req),
    .mn0_resp_valid     (dmem_resp_valid),
    .mn0_resp_ready     (dmem_resp_ready),
    .mn0_resp           (dmem_resp),
    .mn0_tid            (1'b0),
    //noc 1 
    .mn1_req_valid      (dp_req_valid),
    .mn1_req_ready      (dp_req_ready),
    .mn1_req            (dp_req),
    .mn1_resp_valid     (dp_resp_valid),
    .mn1_resp_ready     (dp_resp_ready),
    .mn1_resp           (dp_resp),
    .mn1_tid            (1'b0),
    //noc arb
    .sn_req_valid       (dmem_debug_req_valid),
    .sn_req_ready       (dmem_debug_req_ready),
    .sn_req             (dmem_debug_req),
    .sn_resp_valid      (dmem_debug_resp_valid),
    .sn_resp_ready      (dmem_debug_resp_ready),
    .sn_resp            (dmem_debug_resp),
    .sn_tid             (1'b0)    
);

dmem_noc_router_1to4 dmem_noc_router_1to4_u (
    .clk(clk),
    .rstn(cpu_rstn), 

    //noc master 0 
    .mn_req_valid       (dmem_debug_req_valid),
    .mn_req_ready       (dmem_debug_req_ready),
    .mn_req             (dmem_debug_req),

    .mn_resp_valid      (dmem_debug_resp_valid),
    .mn_resp_ready      (dmem_debug_resp_ready),
    .mn_resp            (dmem_debug_resp),  

    //noc slave 0 
    .sn0_req_valid      (dmem_ocp_req_valid),
    .sn0_req_ready      (dmem_ocp_req_ready),
    .sn0_req            (dmem_ocp_req),

    .sn0_resp_valid     (dmem_ocp_resp_valid),
    .sn0_resp_ready     (dmem_ocp_resp_ready),
    .sn0_resp           (dmem_ocp_resp),

    //noc slave 1 
    .sn1_req_valid      (dmem_itcm_req_valid),
    .sn1_req_ready      (dmem_itcm_req_ready),
    .sn1_req            (dmem_itcm_req),

    .sn1_resp_valid     (dmem_itcm_resp_valid),
    .sn1_resp_ready     (dmem_itcm_resp_ready),
    .sn1_resp           (dmem_itcm_resp),

    //noc slave 2 
    .sn2_req_valid      (dmem_dtcm_req_valid),
    .sn2_req_ready      (dmem_dtcm_req_ready),
    .sn2_req            (dmem_dtcm_req),

    .sn2_resp_valid     (dmem_dtcm_resp_valid),
    .sn2_resp_ready     (dmem_dtcm_resp_ready),
    .sn2_resp           (dmem_dtcm_resp),  

    //noc slave 3 
    .sn3_req_valid      (dmem_sys_req_valid),
    .sn3_req_ready      (dmem_sys_req_ready),
    .sn3_req            (dmem_sys_req),

    .sn3_resp_valid     (dmem_sys_resp_valid),
    .sn3_resp_ready     (dmem_sys_resp_ready),
    .sn3_resp           (dmem_sys_resp)     
);

imem_noc_router_1to2 #(
    .DEC_NUM(2),
    .DEC_TAG_H(31),
    .DEC_TAG_L(28),
    .DEC_TAG_VAL(MEM_BASE_ADDR_ITCM[31:28])
) imem_noc_router_1to2_u (
    .clk                (clk),
    .rstn               (core_rstn),
    //noc master 0 
    .mn_req_valid       (imem_req_valid),
    .mn_req_ready       (imem_req_ready),
    .mn_req             (imem_req),
    .mn_resp_valid      (imem_resp_valid),
    .mn_resp_ready      (imem_resp_ready),
    .mn_resp            (imem_resp),
    //noc slave 0 
    .sn0_req_valid      (imem_itcm_req_valid),
    .sn0_req_ready      (imem_itcm_req_ready),
    .sn0_req            (imem_itcm_req),
    .sn0_resp_valid     (imem_itcm_resp_valid),
    .sn0_resp_ready     (imem_itcm_resp_ready),
    .sn0_resp           (imem_itcm_resp),
    //noc slave 1 
    .sn1_req_valid      (imem_icache_req_valid),
    .sn1_req_ready      (imem_icache_req_ready),
    .sn1_req            (imem_icache_req),
    .sn1_resp_valid     (imem_icache_resp_valid),
    .sn1_resp_ready     (imem_icache_resp_ready),
    .sn1_resp           (imem_icache_resp) 
);

mem_noc_arb_2to1 itcm_sram_arb_u (
    .clk                (clk),
    .rstn               (cpu_rstn),
    //noc 0 
    .mn0_req_valid      (imem_itcm_req_valid),
    .mn0_req_ready      (imem_itcm_req_ready),
    .mn0_req            (imem_itcm_req),
    .mn0_resp_valid     (imem_itcm_resp_valid),
    .mn0_resp_ready     (imem_itcm_resp_ready),
    .mn0_resp           (imem_itcm_resp),
    .mn0_tid            (1'b0),
    //noc 1 
    .mn1_req_valid      (dmem_itcm_req_valid),
    .mn1_req_ready      (dmem_itcm_req_ready),
    .mn1_req            (dmem_itcm_req),
    .mn1_resp_valid     (dmem_itcm_resp_valid),
    .mn1_resp_ready     (dmem_itcm_resp_ready),
    .mn1_resp           (dmem_itcm_resp),
    .mn1_tid            (1'b0),
    //noc arb
    .sn_req_valid       (itcm_req_valid),
    .sn_req_ready       (itcm_req_ready),
    .sn_req             (itcm_req),
    .sn_resp_valid      (itcm_resp_valid),
    .sn_resp_ready      (itcm_resp_ready),
    .sn_resp            (itcm_resp),
    .sn_tid             (1'b0)         
);

// assign icache_sys_req_valid = imem_icache_req_valid;
// assign icache_sys_req       = imem_icache_req;
// assign imem_icache_req_ready= icache_sys_req_ready;

// assign imem_icache_resp_valid = icache_sys_resp_valid;
// assign imem_icache_resp       = icache_sys_resp;
// assign icache_sys_resp_ready  = imem_icache_resp_ready;


icache_ctrl icache_ctrl_u 
(
    .clk                        (clk),
    .rstn                       (core_rstn),
    // cfg
    .cfg_icache_en              (cfg_icache_en),     // icache active high 
    // from core
    .core_mem_req_valid         (imem_icache_req_valid),
    .core_mem_req_ready         (imem_icache_req_ready),
    .core_mem_req               (imem_icache_req),

    .core_mem_resp_valid        (imem_icache_resp_valid),
    .core_mem_resp_ready        (imem_icache_resp_ready),
    .core_mem_resp              (imem_icache_resp),

    // to memory
    .imem_req_valid             (icache_sys_req_valid),
    .imem_req_ready             (icache_sys_req_ready),
    .imem_req                   (icache_sys_req), 

    .imem_resp_valid            (icache_sys_resp_valid),
    .imem_resp_ready            (icache_sys_resp_ready),
    .imem_resp                  (icache_sys_resp),

    // .core_mem_req_valid         (itcm_req_valid),
    // .core_mem_req_ready         (itcm_req_ready),
    // .core_mem_req               (itcm_req),

    // .core_mem_resp_valid        (itcm_resp_valid),
    // .core_mem_resp_ready        (itcm_resp_ready),
    // .core_mem_resp              (itcm_resp),

    // // to memory
    // .imem_req_valid             (sram_req_valid),
    // .imem_req_ready             (sram_req_ready),
    // .imem_req                   (sram_req), 

    // .imem_resp_valid            (sram_resp_valid),
    // .imem_resp_ready            (sram_resp_ready),
    // .imem_resp                  (sram_resp),

    // to tag_ram
    .tag_ram_csn                (tag_ram_csn),
    .tag_ram_wen                (tag_ram_wen),
    .tag_ram_web                (tag_ram_web),
    .tag_ram_addr               (tag_ram_addr),
    .tag_ram_din                (tag_ram_din),
    .tag_ram_dout               (tag_ram_dout),

    // to data_ram    
    .data_ram_csn               (data_ram_csn),
    .data_ram_wen               (data_ram_wen),
    .data_ram_web               (data_ram_web),
    .data_ram_addr              (data_ram_addr),
    .data_ram_din               (data_ram_din),
    .data_ram_dout              (data_ram_dout)
);

generate
    for(genvar i = 0; i < CACHE_WAY_NUM; i = i + 1) begin : TAG_RAM_GEN
        fast_sram_sp #(
            .N_DW(MEM_ADDR_W),
            .N_DP(1)
        ) tag_ram_u (
            .clk(clk),
            .csn(tag_ram_csn[i]), // chip select/enable, active low
            .wen(tag_ram_wen[i]), // write enable, active low
            .web(tag_ram_web[i]), // byte select, active low
            .addr(tag_ram_addr[i]),
            .din(tag_ram_din[i]),
            .dout(tag_ram_dout[i])        
        );
    end
endgenerate

generate
    for(genvar i = 0; i < CACHE_WAY_NUM; i = i + 1) begin : DATA_RAM_GEN
        fast_sram_sp #(
            .N_DW(CACHE_DATA_W),
            .N_DP({CACHE_INDEX_W{1'b1}}+1)
        ) tag_ram_u (
            .clk(clk),
            .csn(data_ram_csn[i]), // chip select/enable, active low
            .wen(data_ram_wen[i]), // write enable, active low
            .web(data_ram_web[i]), // byte select, active low
            .addr(data_ram_addr[i]),
            .din(data_ram_din[i]),
            .dout(data_ram_dout[i])        
        );
    end
endgenerate

urv_sram #(
    .WORDS(DTCM_SIZE)
) dtcm_u (
    .clk                (clk),
    .rstn               (cpu_rstn),
    .mem_req_valid      (dmem_dtcm_req_valid),
    .mem_req_ready      (dmem_dtcm_req_ready),
    .mem_req            (dmem_dtcm_req),

    .mem_resp_valid     (dmem_dtcm_resp_valid),
    .mem_resp_ready     (dmem_dtcm_resp_ready),
    .mem_resp           (dmem_dtcm_resp)
);

urv_sram #(
    .WORDS(ITCM_SIZE)
) itcm_u (
    .clk                 (clk),
    .rstn                (cpu_rstn),
    .mem_req_valid       (itcm_req_valid),
    .mem_req_ready       (itcm_req_ready),
    .mem_req             (itcm_req),

    .mem_resp_valid      (itcm_resp_valid),
    .mem_resp_ready      (itcm_resp_ready),
    .mem_resp            (itcm_resp)
);

// urv_sram_burst #(
//     .WORDS(ITCM_SIZE)
// ) itcm_u (
//     .clk                (clk),
//     .rstn               (cpu_rstn),
//     .mem_req_valid      (sram_req_valid),
//     .mem_req_ready      (sram_req_ready),
//     .mem_req            (sram_req),

//     .mem_resp_valid     (sram_resp_valid),
//     .mem_resp_ready     (sram_resp_ready),
//     .mem_resp           (sram_resp)
// );

endmodule