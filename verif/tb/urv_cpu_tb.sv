
`include "inst_index.v"
`include "macro.v"
`timescale 1ns / 1ns

//SIM_FREQ unit:MHz
`ifndef SIM_FREQ
`define SIM_FREQ 32'd32
`endif

//SIM_PERIOD unit:ns
`ifndef SIM_PERIOD
`define SIM_PERIOD 15.625
`endif

`ifndef SIM_TIME
`define SIM_TIME 32'd10
`endif


module urv_cpu_tb;
    import urv_cfg::*;
    import urv_typedef::*; 

    logic clk;
    logic rstn;

    logic [31:0] log;
    initial begin
        if(`DB == 1)
            log = $fopen("./vcs_log_urv_cpu_tb_debug_ok.txt","w");
        else if(`DB == 2)
            log = $fopen("./vcs_log_urv_cpu_tb_debug_error.txt","w");
        else if(`DB == 0)
            log = $fopen("./vcs_log_urv_cpu_tb.txt","w");
    end

    localparam SIMULATION_PERIOD = `SIM_PERIOD;
    initial begin
        clk = 0;
        $display("\nSIMULATION_PERIOD %.3fns with frequency %2dMHz", SIMULATION_PERIOD, `SIM_FREQ);
        // repeat(1000000) #15.625 clk = ~clk;
        forever #(SIMULATION_PERIOD) clk = ~clk;
        // forever #20 clk = ~clk;
    end

    initial begin
        rstn = 0;
        #50 rstn = 1;
    end

    //Use Backdoor Load image
    initial begin
        string     image_path;
        int        image_fd;
        bit [31:0] data;
        logic[31:0]bdata;
        int        num_byte;
        bit [31:0] wdata;
        bit [31:0] addr;
        bit [31:0] image_addr;
        bit [31:0] adr;
        //Loading Image
        if ($value$plusargs("backdoor_load_image=%s", image_path)) begin
            // $display("Start Loading image from backdoor");
            // $display("backdoor load image = %s", image_path);
            image_fd = $fopen(image_path, "rb");
        if (image_fd == 0)
            $display("TOP", $sformatf("Unable to open %s.bin for read", image_path));
            image_addr = RST_PC;
            addr = image_addr;
            num_byte = $fread(bdata, image_fd);
        while (num_byte > 0) begin                            
            // align byte endianess
            if (num_byte == 4) begin
                data[31:24] = bdata[7:0];
                data[23:16] = bdata[15:8];
                data[15:8]  = bdata[23:16];
                data[7:0]   = bdata[31:24];
            end
            else if (num_byte == 3) begin
                data[31:24] = 0;
                data[23:16] = bdata[15:8];
                data[15:8]  = bdata[23:16];
                data[7:0]   = bdata[31:24];
            end
            else if (num_byte == 2) begin
                data[31:24] = 0;
                data[23:16] = 0;
                data[15:8]  = bdata[23:16];
                data[7:0]   = bdata[31:24];
            end
            else if (num_byte == 1) begin
                data[31:24] = 0;
                data[23:16] = 0;
                data[15:8]  = 0;
                data[7:0]   = bdata[31:24];
            end
            wdata = data[31:0];

            adr = image_addr;
            // $display("backdoor adr=%h wdata=%h bdata=%h data=%h num_byte=%h", adr, wdata, bdata, data, num_byte);		  
            //backdoor_load_to_mem
            `ifdef ROM
                urv_cpu_tb.urv_cpu_u.urv_core_u.itcm_u.fast_sram_sp_u.mem[adr[17:2]] = wdata; 
                urv_cpu_tb.sram_u.fast_sram_sp_u.mem[adr[17:2]] = wdata; 
            `endif
            `ifdef RAM
                urv_cpu_tb.sram_u.fast_sram_sp_u.mem[adr[17:2]] = wdata; 	  
            `endif
            image_addr += 4;
            num_byte = $fread(bdata, image_fd);               //此系统函数地址自增
            end
            $fclose(image_fd);
            $display("Done Loading Image from backdoor. addr range: %h %h\n",
                    addr, image_addr);
        end
    end

    localparam SIMULATION_FREQ      = `SIM_FREQ * 32'd1000000;
    localparam CYCLE_1MS            = SIMULATION_FREQ / 32'd1000;
    localparam SIMULATION_CYCLE_END = `SIM_TIME * CYCLE_1MS;
    localparam CYCLE_100MS          = 32'd100 * CYCLE_1MS;
    localparam SIMULATION_TIME      = `SIM_TIME;

    logic [31:0] cycle_num;
    logic [31:0] cycle_100ms_count;
    logic [31:0] cycle_100ms_cmp;
    logic cycle_time_100ms_flag, cycle_time_out;
    always_ff @(posedge clk or negedge rstn) begin
        if(!rstn) 
        begin
            cycle_num             <= 32'h0;
            cycle_100ms_count     <= 32'h0;
            cycle_100ms_cmp       <= CYCLE_100MS;
            cycle_time_out        <= 1'b0;
            cycle_time_100ms_flag <= 1'b0;
        end 
        else if(cycle_num == cycle_100ms_cmp) 
        begin
            cycle_100ms_count     <= cycle_100ms_count + 1'b1;
            cycle_100ms_cmp       <= cycle_100ms_cmp + CYCLE_100MS;
            cycle_time_100ms_flag <= 1'b1;
        end 
        else if(cycle_num == SIMULATION_CYCLE_END)
        begin
            cycle_time_out    <= 1'b1;
        end 
        else 
        begin
            cycle_num <= cycle_num + 1'b1;
            cycle_time_100ms_flag <= 1'b0;
        end
    end

    if(`DB == 0) begin : NORMAL_GEN
        logic print_valid;
        logic [31:0] print_data; 
        // assign print_valid = urv_cpu_tb.urv_cpu_u.mem_noc_router_1to4_u.sn3_req_valid 
        //                   && urv_cpu_tb.urv_cpu_u.mem_noc_router_1to4_u.sn3_req_ready 
        //                   && (urv_cpu_tb.urv_cpu_u.mem_noc_router_1to4_u.sn3_req.req_addr == 32'h10010000);
        // assign print_data  = print_valid ? urv_cpu_tb.urv_cpu_u.mem_noc_router_1to4_u.sn3_req.req_data : 32'h0;
        // assign print_valid = urv_cpu_tb.urv_cpu_u.mem_noc_router_1to4_u.sn_req_valid[3] 
        //                   && urv_cpu_tb.urv_cpu_u.mem_noc_router_1to4_u.sn_req_ready[3] 
        //                   && (urv_cpu_tb.urv_cpu_u.mem_noc_router_1to4_u.sn_req[3].req_addr == 32'h10010000);
        // assign print_data  = print_valid ? urv_cpu_tb.urv_cpu_u.mem_noc_router_1to4_u.sn_req[3].req_data : 32'h0;
        assign print_valid = urv_cpu_tb.uart_u.mem_req_valid 
                        && urv_cpu_tb.uart_u.mem_req_ready
                        && (urv_cpu_tb.uart_u.mem_req.req_addr == 32'h10010000);
        assign print_data  = print_valid ? urv_cpu_tb.uart_u.mem_req.req_data : 32'h0;
        always_ff @(posedge clk) begin
            if(print_valid) begin
                    $write("%c",print_data);
                    //$write("snoc1_mem_wdata_d=0x%x\n",print_data);
                    $fwrite(log,"%c", print_data); 
                    // $display("0x%x",print_data);
            end
        end
    end else begin : DEBUG_GEN
        logic print_valid;
        logic [31:0] print_pc, print_inst;
        logic [63:0] print_inst_ascii;
        logic [31:0] print_count; 
        logic [31:0] print_alu_out;
        logic        print_lsu_req_valid;
        logic        print_lsu_resp_valid;
        logic        print_lsu_req_type;
        logic [31:0] print_lsu_req_addr;
        logic [31:0] print_lsu_req_wdata;
        logic [31:0] print_lsu_resp_rdata;
        assign print_valid = urv_cpu_tb.urv_cpu_u.urv_core_u.urv_pipe_u.exu_u.iex2lsu_valid;
        assign print_pc    = urv_cpu_tb.urv_cpu_u.urv_core_u.urv_pipe_u.exu_u.iex2lsu_pc; 
        assign print_inst  = urv_cpu_tb.urv_cpu_u.urv_core_u.urv_pipe_u.exu_u.iex2lsu_inst;
        assign print_inst_ascii = urv_cpu_tb.urv_cpu_u.urv_core_u.urv_pipe_u.exu_u.iex2lsu_inst_ascii;
        assign print_alu_out    = urv_cpu_tb.urv_cpu_u.urv_core_u.urv_pipe_u.exu_u.alu_u.out;
        // assign print_lsu_req_valid  = urv_cpu_tb.sram_u.mem_req_valid;
        // assign print_lsu_req_type   = urv_cpu_tb.sram_u.mem_req.req_type;
        // assign print_lsu_req_addr   = urv_cpu_tb.sram_u.mem_req.req_addr;
        // assign print_lsu_req_wdata  = urv_cpu_tb.sram_u.mem_req.req_data;
        // assign print_lsu_resp_valid = urv_cpu_tb.sram_u.mem_resp_valid;
        // assign print_lsu_resp_rdata = print_lsu_resp_valid ? urv_cpu_tb.sram_u.mem_resp.resp_data : 'h0;
        always @(posedge clk or negedge rstn) begin
            if(!rstn) begin
                print_count <= 64'h1;
            end else
            if(print_valid) begin
                    // $write("cycle_count=%d, iex_pc=0x%x, iex_inst=0x%x, iex_inst_iex=%s, iex_alu_out=0x%x, lsu_req_valid=0x%x, lsu_req_type=0x%x, print_lsu_req_addr=0x%x,  print_lsu_req_wdata=0x%x, print_lsu_resp_valid=0x%x, print_lsu_resp_rdata=0x%x\n", print_count, print_pc, print_inst, print_inst_ascii, print_alu_out, print_lsu_req_valid, print_lsu_req_type, print_lsu_req_addr, print_lsu_req_wdata, print_lsu_resp_valid, print_lsu_resp_rdata);                 //$write("snoc1_mem_wdata_d=0x%x\n",print_data);
                    // $fwrite(log,"cycle_count=%d, iex_pc=0x%x, iex_inst=0x%x, iex_inst_iex=%s, iex_alu_out=0x%x, lsu_req_valid=0x%x, lsu_req_type=0x%x, print_lsu_req_addr=0x%x,  print_lsu_req_wdata=0x%x, print_lsu_resp_valid=0x%x, print_lsu_resp_rdata=0x%x\n", print_count, print_pc, print_inst, print_inst_ascii, print_alu_out, print_lsu_req_valid, print_lsu_req_type, print_lsu_req_addr, print_lsu_req_wdata, print_lsu_resp_valid, print_lsu_resp_rdata);                 //$write("snoc1_mem_wdata_d=0x%x\n",print_data); 
                    $write("cycle_count=%d, iex_pc=0x%x, iex_inst=0x%x, iex_inst_iex=%s, iex_alu_out=0x%x\n", print_count, print_pc, print_inst, print_inst_ascii, print_alu_out);                 //$write("snoc1_mem_wdata_d=0x%x\n",print_data);
                    $fwrite(log,"cycle_count=%d, iex_pc=0x%x, iex_inst=0x%x, iex_inst_iex=%s, iex_alu_out=0x%x\n", print_count, print_pc, print_inst, print_inst_ascii, print_alu_out);                 //$write("snoc1_mem_wdata_d=0x%x\n",print_data);                    
                    // $display("0x%x",print_data);
                    print_count <= print_count + 1'b1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if(cycle_time_out) 
        begin
            $display("time over max limitation, auto finish at %dms", SIMULATION_TIME);
            $fclose(log); 
            $finish(1);
        end 
        else if(cycle_time_100ms_flag) 
        begin
            $display("time-consuming %dms", cycle_100ms_count * 32'd100);
        end
    end

    initial
    begin
        `ifdef FSDB_USOC
            if(`DB == 0) begin
                $fsdbDumpfile("urv_cpu_tb.fsdb");  //生成fsdb文件，记录仿真信息
                $fsdbDumpvars(0, urv_cpu_tb);  //指定层次数，记录信号 
                $fsdbDumpvars("+struct");
                $fsdbDumpvars("+mda");
                $fsdbDumpvars("+all");
                $fsdbDumpon;
            end else if(`DB == 1) begin
                $fsdbDumpfile("urv_cpu_tb_ok.fsdb");  //生成fsdb文件，记录仿真信息
                $fsdbDumpvars(0, urv_cpu_tb);  //指定层次数，记录信号 
                $fsdbDumpvars("+struct");
                $fsdbDumpvars("+mda");
                $fsdbDumpvars("+all");
                $fsdbDumpon;
            end else if(`DB == 2) begin
                $fsdbDumpfile("urv_cpu_tb_error.fsdb");  //生成fsdb文件，记录仿真信息
                $fsdbDumpvars(0, urv_cpu_tb);  //指定层次数，记录信号 
                $fsdbDumpvars("+struct");
                $fsdbDumpvars("+mda");
                $fsdbDumpvars("+all");
                $fsdbDumpon;
            end
        `endif   
        // $dumpfile("urv_cpu_tb.vcd");  //生成vcd文件，记录仿真信息
        // $dumpvars(0, urv_cpu_tb);  //指定层次数，记录信号
    end

logic                        jtag_tck;
logic                        jtag_tms;
logic                        jtag_tdi;
logic                        jtag_tdo;
logic                        jtag_tdo_en;
logic                        jtag_trstn;

logic                        sdp_ck;
logic                        sdp_di;
logic                        sdp_do;
logic                        sdp_doen;
logic                        sdp_rstn;

logic                        dm_rstn;
logic                        ndm_rst;

logic                        cfg_icache_en;
logic     [PLIC_IRQ_N-1:0]   ext_irq_src;

logic                        sys_req_valid;
logic                        sys_req_ready;
mem_req_t                    sys_req;

logic                        sys_resp_valid;
logic                        sys_resp_ready;
mem_resp_t                   sys_resp;

logic                        sys_sram_req_valid;
logic                        sys_sram_req_ready;
mem_req_t                    sys_sram_req;

logic                        sys_sram_resp_valid;
logic                        sys_sram_resp_ready;
mem_resp_t                   sys_sram_resp;

logic                        sys_uart_req_valid;
logic                        sys_uart_req_ready;
mem_req_t                    sys_uart_req;

logic                        sys_uart_resp_valid;
logic                        sys_uart_resp_ready;
mem_resp_t                   sys_uart_resp;

assign cfg_icache_en         =  1'b1;
assign ext_irq_src           =  {PLIC_IRQ_N{1'b0}};

urv_cpu urv_cpu_u (
    .clk                    (clk),
    .core_rstn              (rstn),
    .cpu_rstn               (rstn),
    // todo 
    // dm/ndm rstn
    .dm_rstn                (dm_rstn),
    .ndm_rst                (ndm_rst),
    .rst_pc                 (RST_PC),
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

// parameter  DEC_NUM          = 2;

// logic      [DEC_NUM-1:0]    sn_sys_req_valid;
// logic      [DEC_NUM-1:0]    sn_sys_req_ready;
// mem_req_t  [DEC_NUM-1:0]    sn_sys_req;

// logic      [DEC_NUM-1:0]    sn_sys_resp_valid;
// logic      [DEC_NUM-1:0]    sn_sys_resp_ready;
// mem_resp_t [DEC_NUM-1:0]    sn_sys_resp;

// mem_noc_router  #(
//      .ENABLE_DEC_TAG(1)
//     ,.ENABLE_DEC_IDX(0)
//     ,.DEC_NUM(DEC_NUM)
//     ,.DEC_TAG_H(31)
//     ,.DEC_TAG_L(28)
//     // ,.DEC_IDX_H(14)
//     // ,.DEC_IDX_L(12)
//     ,.DEC_TAG_VAL(MEM_BASE_ADDR_DDR[31:28])
// ) sys_noc_router_1to2_u (
//     .clk            (clk),
//     .rstn           (rstn),

//     //noc master 0 
//     .mn_req_valid   (sys_req_valid),
//     .mn_req_ready   (sys_req_ready),
//     .mn_req         (sys_req),

//     .mn_resp_valid  (sys_resp_valid),
//     .mn_resp_ready  (sys_resp_ready),
//     .mn_resp        (sys_resp),  

//     //noc slave 0 to clint
//     .sn_req_valid   (sn_sys_req_valid),
//     .sn_req_ready   (sn_sys_req_ready),
//     .sn_req         (sn_sys_req),

//     .sn_resp_valid  (sn_sys_resp_valid),
//     .sn_resp_ready  (sn_sys_resp_ready),
//     .sn_resp        (sn_sys_resp)   
// );

// assign sys_sram_req_valid   = sn_sys_req_valid[0];
// assign sys_sram_req         = sn_sys_req[0];
// assign sn_sys_req_ready[0]  = sys_sram_req_ready;

// assign sn_sys_resp_valid[0] = sys_sram_resp_valid;
// assign sn_sys_resp[0]       = sys_sram_resp;
// assign sys_sram_resp_ready  = sn_sys_resp_ready[0];

// assign sys_uart_req_valid   = sn_sys_req_valid[1];
// assign sys_uart_req         = sn_sys_req[1];
// assign sn_sys_req_ready[1]  = sys_uart_req_ready;

// assign sn_sys_resp_valid[1] = sys_uart_resp_valid;
// assign sn_sys_resp[1]       = sys_uart_resp;
// assign sys_uart_resp_ready  = sn_sys_resp_ready[1];

mem_noc_router_1to2 #(
    .DEC_NUM(2),
    .DEC_TAG_H(31),
    .DEC_TAG_L(28),
    .DEC_TAG_VAL(MEM_BASE_ADDR_DDR[31:28])
) sys_noc_router_1to2_u (
    .clk                    (clk),
    .rstn                   (rstn),
    //noc master 0 
    .mn_req_valid           (sys_req_valid),
    .mn_req_ready           (sys_req_ready),
    .mn_req                 (sys_req),
    .mn_resp_valid          (sys_resp_valid),
    .mn_resp_ready          (sys_resp_ready),
    .mn_resp                (sys_resp),
    //noc slave 0 
    .sn0_req_valid          (sys_sram_req_valid),
    .sn0_req_ready          (sys_sram_req_ready),
    .sn0_req                (sys_sram_req),
    .sn0_resp_valid         (sys_sram_resp_valid),
    .sn0_resp_ready         (sys_sram_resp_ready),
    .sn0_resp               (sys_sram_resp),
    //noc slave 1 
    .sn1_req_valid          (sys_uart_req_valid),
    .sn1_req_ready          (sys_uart_req_ready),
    .sn1_req                (sys_uart_req),
    .sn1_resp_valid         (sys_uart_resp_valid),
    .sn1_resp_ready         (sys_uart_resp_ready),
    .sn1_resp               (sys_uart_resp) 
);

urv_sram_burst #(
    .WORDS(128 * 1024)
) sram_u (
    .clk                    (clk),
    .rstn                   (rstn),
    .mem_req_valid          (sys_sram_req_valid),
    .mem_req_ready          (sys_sram_req_ready),
    .mem_req                (sys_sram_req),

    .mem_resp_valid         (sys_sram_resp_valid),
    .mem_resp_ready         (sys_sram_resp_ready),
    .mem_resp               (sys_sram_resp)
);

urv_sram_burst #(
    .WORDS(4)
) uart_u (
    .clk                    (clk),
    .rstn                   (rstn),
    .mem_req_valid          (sys_uart_req_valid),
    .mem_req_ready          (sys_uart_req_ready),
    .mem_req                (sys_uart_req),

    .mem_resp_valid         (sys_uart_resp_valid),
    .mem_resp_ready         (sys_uart_resp_ready),
    .mem_resp               (sys_uart_resp)
);

endmodule