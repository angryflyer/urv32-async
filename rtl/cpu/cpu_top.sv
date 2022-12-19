//流水线译码测试
`include "inst_index.v"
`include "macro.v"
// `include "cfg.sv"
// `include "typedef.sv"

module cpu_top 
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input clk,
    input rstn,
    // for sim observe
    output iex2ac_jump_valid,
    output [31:0] iex2ac_jump_pc,
    output [31:0] iex2lsu_pc
);

parameter integer RAM_SIZE = `RAM_WIDTH_IN_BYTE >> 2;
parameter integer ROM_SIZE = `ROM_WIDTH_IN_BYTE >> 2;

logic imem_req_valid;
logic imem_req_ready;
mem_req_t imem_req;

logic imem_resp_valid;
logic imem_resp_ready;
mem_resp_t imem_resp;

logic dmem_req_valid;
logic dmem_req_ready;
mem_req_t dmem_req;

logic dmem_resp_valid;
logic dmem_resp_ready;
mem_resp_t dmem_resp;

logic sn0_req_valid;
logic sn0_req_ready;
mem_req_t sn0_req;

logic sn0_resp_valid;
logic sn0_resp_ready;
mem_resp_t sn0_resp;

logic sn1_req_valid;
logic sn1_req_ready;
mem_req_t sn1_req;

logic sn1_resp_valid;
logic sn1_resp_ready;
mem_resp_t sn1_resp;

logic sn2_req_valid;
logic sn2_req_ready;
mem_req_t sn2_req;

logic sn2_resp_valid;
logic sn2_resp_ready;
mem_resp_t sn2_resp;

logic sn3_req_valid;
logic sn3_req_ready;
mem_req_t sn3_req;

logic sn3_resp_valid;
logic sn3_resp_ready;
mem_resp_t sn3_resp;

logic rom_req_valid;
logic rom_req_ready;
mem_req_t rom_req;

logic rom_resp_valid;
logic rom_resp_ready;
mem_resp_t rom_resp;

logic ram_req_valid;
logic ram_req_ready;
mem_req_t ram_req;

logic ram_resp_valid;
logic ram_resp_ready;
mem_resp_t ram_resp;

logic ti_mem_req_valid;
logic ti_mem_req_ready;
mem_req_t ti_mem_req;

logic ti_mem_resp_valid;
logic ti_mem_resp_ready;
mem_resp_t ti_mem_resp;

logic ext_irq;
logic soft_irq;
logic time_irq;
logic [CSR_TIME_W-1:0] time_val;

core_top core_top_u (
    .clk(clk),
    .rstn(rstn),
    .rst_pc(`RST_PC),

    // .ext_irq(ext_irq),
    .ext_irq(1'b0),
    .soft_irq(soft_irq),  
    .time_irq(time_irq),
    .time_val(time_val), 
    // instruction fetch bus
    .imem_req_valid(imem_req_valid),
    .imem_req_ready(imem_req_ready),
    .imem_req(imem_req),     
    .imem_resp_valid(imem_resp_valid),
    .imem_resp_ready(imem_resp_ready),
    .imem_resp(imem_resp), 
    // memery bus
    .dmem_req_valid(dmem_req_valid),
    .dmem_req_ready(dmem_req_ready),
    .dmem_req(dmem_req), 
    .dmem_resp_valid(dmem_resp_valid),
    .dmem_resp_ready(dmem_resp_ready),
    .dmem_resp(dmem_resp), 
    // for sim observe
    .iex2ac_jump_valid(iex2ac_jump_valid),
    .iex2ac_jump_pc(iex2ac_jump_pc),   
    .iex2lsu_pc(iex2lsu_pc)	
    );

mem_noc_router_1to4 mem_noc_router_1to4_u (
    .clk(clk),
    .rstn(rstn), 

    //noc master 0 
    .mn_req_valid(dmem_req_valid),
    .mn_req_ready(dmem_req_ready),
    .mn_req(dmem_req),

    .mn_resp_valid(dmem_resp_valid),
    .mn_resp_ready(dmem_resp_ready),
    .mn_resp(dmem_resp),  

    //noc slave 0 
    .sn0_req_valid(sn0_req_valid),
    .sn0_req_ready(sn0_req_ready),
    .sn0_req(sn0_req),

    .sn0_resp_valid(sn0_resp_valid),
    .sn0_resp_ready(sn0_resp_ready),
    .sn0_resp(sn0_resp),

    //noc slave 1 
    .sn1_req_valid(sn1_req_valid),
    .sn1_req_ready(sn1_req_ready),
    .sn1_req(sn1_req),

    .sn1_resp_valid(sn1_resp_valid),
    .sn1_resp_ready(sn1_resp_ready),
    .sn1_resp(sn1_resp),  

    //noc slave 2 
    .sn2_req_valid(sn2_req_valid),
    .sn2_req_ready(sn2_req_ready),
    .sn2_req(sn2_req),

    .sn2_resp_valid(sn2_resp_valid),
    .sn2_resp_ready(sn2_resp_ready),
    .sn2_resp(sn2_resp),  

    //noc slave 3 
    .sn3_req_valid(sn3_req_valid),
    .sn3_req_ready(sn3_req_ready),
    .sn3_req(sn3_req),

    .sn3_resp_valid(sn3_resp_valid),
    .sn3_resp_ready(sn3_resp_ready),
    .sn3_resp(sn3_resp)         
);

clint clint_u (
    .clk(clk),
    .rstn(rstn),
    // mem interface
    .mem_req_valid(sn0_req_valid),
    .mem_req_ready(sn0_req_ready),
    .mem_req(sn0_req), 

    .mem_resp_valid(sn0_resp_valid),
    .mem_resp_ready(sn0_resp_ready),
    .mem_resp(sn0_resp), 

    .soft_irq(soft_irq),  
    .time_irq(time_irq),
    .time_val(time_val)        
);

plic plic_u (
    .clk(clk),
    .rstn(rstn),
    // mem interface
    .mem_req_valid(sn1_req_valid),
    .mem_req_ready(sn1_req_ready),
    .mem_req(sn1_req), 

    .mem_resp_valid(sn1_resp_valid),
    .mem_resp_ready(sn1_resp_ready),
    .mem_resp(sn1_resp), 

    .ext_irq_src({time_irq,time_irq,1'b0}),
    .ext_irq(ext_irq)        
);

mem_noc mem_noc_u (
    .clk(clk),
    .rstn(rstn),

    //noc master 0 
    .mn0_req_valid(imem_req_valid),
    .mn0_req_ready(imem_req_ready),
    .mn0_req(imem_req),

    .mn0_resp_valid(imem_resp_valid),
    .mn0_resp_ready(imem_resp_ready),
    .mn0_resp(imem_resp), 

    //noc master 1
    .mn1_req_valid(sn3_req_valid),
    .mn1_req_ready(sn3_req_ready),
    .mn1_req(sn3_req),

    .mn1_resp_valid(sn3_resp_valid),
    .mn1_resp_ready(sn3_resp_ready),
    .mn1_resp(sn3_resp), 

    //noc slave 0 
    .sn0_base_addr(`ROM_BASE_ADDR),
    .sn0_req_valid(rom_req_valid),
    .sn0_req_ready(rom_req_ready),
    .sn0_req(rom_req),

    .sn0_resp_valid(rom_resp_valid),
    .sn0_resp_ready(rom_resp_ready),
    .sn0_resp(rom_resp), 

    //noc slave 1 
    .sn1_base_addr(`RAM_BASE_ADDR),
    .sn1_req_valid(ram_req_valid),
    .sn1_req_ready(ram_req_ready),
    .sn1_req(ram_req),

    .sn1_resp_valid(ram_resp_valid),
    .sn1_resp_ready(ram_resp_ready),
    .sn1_resp(ram_resp)

    // .sn1_req_valid(ti_mem_req_valid),
    // .sn1_req_ready(ti_mem_req_ready),
    // .sn1_req(ti_mem_req),

    // .sn1_resp_valid(ti_mem_resp_valid),
    // .sn1_resp_ready(ti_mem_resp_ready),
    // .sn1_resp(ti_mem_resp)
);

// logic test_clk;
// logic test_din;
// logic test_dout;

// logic ti_dat_i;
// logic ti_clk_o;
// logic ti_dat_o;

// assign test_clk = ti_clk_o;
// assign test_din = ti_dat_o;
// assign ti_dat_i = test_dout;

// testio_slv testio_slv_u
// (
//     .mem_req_valid         (ti_mem_req_valid),
//     .mem_req_ready         (ti_mem_req_ready),
//     .mem_req               (ti_mem_req),

//     .mem_resp_valid        (ti_mem_resp_valid),
//     .mem_resp_ready        (ti_mem_resp_ready),
//     .mem_resp              (ti_mem_resp),
//     //testio          
//     .ti_clk_i                 (clk),          //Peripheral clock signal
//     .ti_rstn_i                (rstn),          //Peripheral async reset signal
//     .ti_mod_i                 (2'b00  ),           //Peripheral mode select

//     .ti_dat_i                 (ti_dat_i),           //Peripheral data in
//     .ti_clk_o                 (ti_clk_o),           //testio clock signal 
//     .ti_clk_oen               (),
//     .ti_dat_o                 (ti_dat_o), 
//     .ti_dat_oen               (),
//     .ti_int_o                 ()
// );

// testio_ma testio_ma_u
// (
//     .rstn_i                   (rstn),      //Peripheral async reset signal

//     .test_intr                (test_intr),

//     // IO Ports
//     .test_clk                 (test_clk),
//     .test_din                 (test_din),
//     .test_dout                (test_dout),
//     .test_doen                (),

//     //mem interface
//     .mem_req_valid         (ram_req_valid),
//     .mem_req_ready         (ram_req_ready),
//     .mem_req               (ram_req),

//     .mem_resp_valid        (ram_resp_valid),
//     .mem_resp_ready        (ram_resp_ready),
//     .mem_resp              (ram_resp)      
// );

// ram

//fixme
// `ifdef INTEL_FPGA
// 	sysram ram_u(
// 		.clock(clk),
// 		.wren(ram_we),
// 		.byteena(snoc1_mem_wstb),
// 		.address(ram_addr[16:2]),
// 		.data(snoc1_mem_wdata),
// 		.q(ram_rdata)		
// 	);
// `else
    ram #(
        .WORDS(RAM_SIZE)
    ) ram_u (
        .clk(clk),
        .rstn(rstn),
        .mem_req_valid(ram_req_valid),
        .mem_req_ready(ram_req_ready),
        .mem_req(ram_req),

        .mem_resp_valid(ram_resp_valid),
        .mem_resp_ready(ram_resp_ready),
        .mem_resp(ram_resp)	
    );
// `endif

// boot_ram #(
// 	.WORDS(RAM_SIZE)    
// ) ram_u (
//     .mem_req_valid                ( ram_req_valid  ) ,
//     .mem_req                      ( ram_req        ) ,
//     .mem_req_ready                ( ram_req_ready  ) ,
//     .mem_resp_valid               ( ram_resp_valid ) ,
//     .mem_resp                     ( ram_resp       ) ,
//     .mem_resp_ready               ( ram_resp_ready ) ,
//     .clk                          ( clk            ) ,
//     .rstn                         ( rstn           ) 
// );

// logic            cdc_mem_req_valid;  // strobe/request
// logic            cdc_mem_req_ready;
// mem_req_t        cdc_mem_req;  // strobe/request

// logic            cdc_mem_resp_valid;  // strobe/request
// logic            cdc_mem_resp_ready;
// mem_resp_t       cdc_mem_resp;  // strobe/request

// cdc_mem_noc cdc_mem_noc_u (
//     .src_clk(clk),
//     .src_rstn(rstn),
//     .src_req_valid(ram_req_valid),
//     .src_req_ready(ram_req_ready),
//     .src_req(ram_req),
//     .src_resp_valid(ram_resp_valid),
//     .src_resp_ready(ram_resp_ready),
//     .src_resp(ram_resp),

//     .dest_clk(clk),
//     .dest_rstn(rstn),
//     .dest_req_valid(cdc_mem_req_valid),
//     .dest_req_ready(cdc_mem_req_ready),
//     .dest_req(cdc_mem_req),
//     .dest_resp_valid(cdc_mem_resp_valid),
//     .dest_resp_ready(cdc_mem_resp_ready),
//     .dest_resp(cdc_mem_resp)
// );

// ram #(
// 	.WORDS(RAM_SIZE)
// ) ram_u (
// 	.clk(clk),
//     .rstn(rstn),
//     .mem_req_valid(cdc_mem_req_valid),
//     .mem_req_ready(cdc_mem_req_ready),
//     .mem_req(cdc_mem_req),

//     .mem_resp_valid(cdc_mem_resp_valid),
//     .mem_resp_ready(cdc_mem_resp_ready),
//     .mem_resp(cdc_mem_resp)
// );

//rom
rom #(
    .WORDS(ROM_SIZE)
) rom_u (
    .clk(clk),
    .rstn(rstn),
    .mem_req_valid(rom_req_valid),
    .mem_req_ready(rom_req_ready),
    .mem_req(rom_req),

    .mem_resp_valid(rom_resp_valid),
    .mem_resp_ready(rom_resp_ready),
    .mem_resp(rom_resp)
);

// logic            cdc_mem_req_valid;  // strobe/request
// logic            cdc_mem_req_ready;
// mem_req_t        cdc_mem_req;  // strobe/request

// logic            cdc_mem_resp_valid;  // strobe/request
// logic            cdc_mem_resp_ready;
// mem_resp_t       cdc_mem_resp;  // strobe/request

// cdc_mem_noc cdc_mem_noc_u (
//     .src_clk(clk),
//     .src_rstn(rstn),
//     .src_req_valid(rom_req_valid),
//     .src_req_ready(rom_req_ready),
//     .src_req(rom_req),
//     .src_resp_valid(rom_resp_valid),
//     .src_resp_ready(rom_resp_ready),
//     .src_resp(rom_resp),

//     .dest_clk(clk),
//     .dest_rstn(rstn),
//     .dest_req_valid(cdc_mem_req_valid),
//     .dest_req_ready(cdc_mem_req_ready),
//     .dest_req(cdc_mem_req),
//     .dest_resp_valid(cdc_mem_resp_valid),
//     .dest_resp_ready(cdc_mem_resp_ready),
//     .dest_resp(cdc_mem_resp)
// );

// rom #(
// 	.WORDS(ROM_SIZE)
// ) rom_u (
// 	.clk(clk),
//     .rstn(rstn),
//     .mem_req_valid(cdc_mem_req_valid),
//     .mem_req_ready(cdc_mem_req_ready),
//     .mem_req(cdc_mem_req),

//     .mem_resp_valid(cdc_mem_resp_valid),
//     .mem_resp_ready(cdc_mem_resp_ready),
//     .mem_resp(cdc_mem_resp)
// );

endmodule