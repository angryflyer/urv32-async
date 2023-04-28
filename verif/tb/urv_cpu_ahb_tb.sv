
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


module urv_cpu_ahb_tb;
    import urv_cfg::*;
    import urv_typedef::*; 

    logic clk;
    logic rstn;
    logic sdp_load_done;

    logic [31:0] log;
    initial begin      
        log = $fopen("./vcs_log_urv_cpu_ahb_tb.txt","w");
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
        rstn          = 0;
        sdp_load_done = 0;
        // #50 rstn      = 1;
    end

    //Use Backdoor Load image
    initial begin
        string     image_path;
        int        image_fd;
        bit [31:0] data;
        logic[31:0]bdata;
        logic[31:0]rdata;
        int        num_byte;
        bit [31:0] wdata;
        bit [31:0] addr;
        bit [31:0] image_addr;
        bit [31:0] adr;
        #50 rstn = 1;
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
            // urv_cpu_ahb_tb.urv_cpu_u.urv_core_u.itcm_u.fast_sram_sp_u.mem[adr[17:2]] = wdata; 
            // urv_cpu_ahb_tb.ahb_sram_u.mem[adr[17:2]] = wdata;
            sdp_write(adr, wdata);
            image_addr += 4;
            num_byte = $fread(bdata, image_fd);               //此系统函数地址自增
            end
            $fclose(image_fd);
            $display("Done Loading Image from backdoor. addr range: %h %h\n",
                    addr, image_addr);
            // sdp_read_check('h40000000, 'h93);
            // sdp_read_check('h40000004, 'h113);
            // sdp_read('h40000008, rdata);
            // sdp_read('h4000000c, rdata);
            // sdp_read('h40000010, rdata);
            // sdp_read('h40000014, rdata);            
            sdp_load_done = 1;
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

    logic print_valid;
    logic [31:0] print_data; 
    // assign print_valid = urv_cpu_ahb_tb.urv_cpu_u.mem_noc_router_1to4_u.sn3_req_valid 
    //                   && urv_cpu_ahb_tb.urv_cpu_u.mem_noc_router_1to4_u.sn3_req_ready 
    //                   && (urv_cpu_ahb_tb.urv_cpu_u.mem_noc_router_1to4_u.sn3_req.req_addr == 32'h10010000);
    // assign print_data  = print_valid ? urv_cpu_ahb_tb.urv_cpu_u.mem_noc_router_1to4_u.sn3_req.req_data : 32'h0;
    // assign print_valid = urv_cpu_ahb_tb.urv_cpu_u.mem_noc_router_1to4_u.sn_req_valid[3] 
    //                   && urv_cpu_ahb_tb.urv_cpu_u.mem_noc_router_1to4_u.sn_req_ready[3] 
    //                   && (urv_cpu_ahb_tb.urv_cpu_u.mem_noc_router_1to4_u.sn_req[3].req_addr == 32'h10010000);
    // assign print_data  = print_valid ? urv_cpu_ahb_tb.urv_cpu_u.mem_noc_router_1to4_u.sn_req[3].req_data : 32'h0;
    assign print_valid = urv_cpu_ahb_tb.urv_cpu_ahb_u.mem2ahb_u.mem_req_valid 
                      && urv_cpu_ahb_tb.urv_cpu_ahb_u.mem2ahb_u.mem_req_ready
                      && (urv_cpu_ahb_tb.urv_cpu_ahb_u.mem2ahb_u.mem_req.req_addr == 32'h10010000);
    assign print_data  = print_valid ? urv_cpu_ahb_tb.urv_cpu_ahb_u.mem2ahb_u.mem_req.req_data : 32'h0;
    always_ff @(posedge clk) begin
        if(print_valid) begin
                $write("%c",print_data);
                //$write("snoc1_mem_wdata_d=0x%x\n",print_data);
                $fwrite(log,"%c", print_data); 
                // $display("0x%x",print_data);
        end
    end

    // logic print_valid;
    // logic [31:0] print_pc, print_inst;
    // logic [63:0] print_inst_ascii;
    // logic  [31:0] print_count; 
    // logic [31:0] print_alu_out;
    // assign print_valid = urv_cpu_ahb_tb.urv_cpu_u.core_top_u.iex_u.iex2lsu_valid;
    // assign print_pc    = urv_cpu_ahb_tb.urv_cpu_u.core_top_u.iex_u.iex2lsu_pc; 
    // assign print_inst  = urv_cpu_ahb_tb.urv_cpu_u.core_top_u.iex_u.iex2lsu_inst;
    // assign print_inst_ascii = urv_cpu_ahb_tb.urv_cpu_u.core_top_u.iex_u.iex2lsu_inst_ascii;
    // assign print_alu_out    = urv_cpu_ahb_tb.urv_cpu_u.core_top_u.iex_u.alu_u.out;
    // always @(posedge clk or negedge rstn) begin
    //     if(!rstn) begin
    //         print_count <= 64'h1;
    //     end else
    //     if(print_valid) begin
    //             $write("cycle_count=%d, iex_pc=0x%x, iex_inst=0x%x, iex_inst_iex=%s, iex_alu_out=0x%x\n",print_count, print_pc, print_inst, print_inst_ascii, print_alu_out);
    //             //$write("snoc1_mem_wdata_d=0x%x\n",print_data);
    //             $fwrite(log,"cycle_count=%d, iex_pc=0x%x, iex_inst=0x%x, iex_inst_iex=%s, iex_alu_out=0x%x\n",print_count, print_pc, print_inst, print_inst_ascii, print_alu_out); 
    //             // $display("0x%x",print_data);
    //             print_count <= print_count + 1'b1;
    //     end
    // end

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
        $fsdbDumpfile("urv_cpu_ahb_tb.fsdb");  //生成fsdb文件，记录仿真信息
        $fsdbDumpvars(0, urv_cpu_ahb_tb);  //指定层次数，记录信号 
        $fsdbDumpvars("+struct");
        $fsdbDumpvars("+mda");
        $fsdbDumpvars("+all");
        $fsdbDumpon;
        `endif   
        // $dumpfile("urv_cpu_ahb_tb.vcd");  //生成vcd文件，记录仿真信息
        // $dumpvars(0, urv_cpu_ahb_tb);  //指定层次数，记录信号
    end

//ahb master to ahb slave
logic [1:0]                  htrans;
logic [2:0]                  hburst;
logic [32-1:0]               haddr;
logic [2:0]                  hsize;
logic [32-1:0]               hwdata;
logic                        hwrite;
logic                        hmastlock;
logic [6:0]                  hprot;
logic [32-1:0]               hrdata;
logic                        hresp;
logic                        hready;

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

logic                        core_rstn;
logic                        cpu_rstn;
logic                        dm_rstn;
logic                        ndm_rst;

logic                        cfg_icache_en;
logic [PLIC_IRQ_N-1:0]       ext_irq_src;

assign cfg_icache_en         =  1'b1;
assign ext_irq_src           =  {PLIC_IRQ_N{1'b0}};

assign sdp_rstn              = rstn;
assign cpu_rstn              = rstn;
assign core_rstn             = rstn & sdp_load_done;

tri                          SDP_CK;
tri                          SDP_DIO;
logic [15:0]                 div_coe;

initial begin
    div_coe = 'h0;           // ck = clk / (div_coe + 1)
end

pullup(SDP_CK);
pullup(SDP_DIO);

assign SDP_DIO               = sdp_doen ? 1'bz    : sdp_do;
assign sdp_di                = sdp_doen ? SDP_DIO : 1'bz;
assign sdp_ck                = SDP_CK;
sdp_agent tb_sdp_master (
    .clk                    (clk),
    .div_coe                (div_coe),
    .sdp_ck                 (SDP_CK),
    .sdp_dio                (SDP_DIO)
);

urv_cpu_ahb urv_cpu_ahb_u (
    .clk                    (clk),
    .core_rstn              (core_rstn),
    .cpu_rstn               (cpu_rstn),
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

// ahb_sram #(
//     .MEMSIZE(128*1024)
// ) ahb_sram_u (
//     .clk(clk),
//     .rstn(cpu_rstn),
//     .hsel(1'b1),
//     .clk_strobe(1'b1),
//     .base_addr(RST_PC),
//     .htrans(htrans),
//     .hburst(hburst),
//     .haddr(haddr),
//     .hsize(hsize),
//     .hwdata(hwdata),
//     .hwrite(hwrite),
//     .hready(hready),
//     .hrdata(hrdata),
//     .hresp(hresp),
//     .hreadyout(hready)
// );

ahb_sram #(
    .MEMSIZE(512*1024)
) ahb_sram_u (
    .hclk(clk),
    .hreset_n(cpu_rstn),
    .hsel(1'b1),
    .base_addr(RST_PC),
    .htrans(htrans),
    .haddr(haddr),
    .hsize(hsize),
    .hwdata(hwdata),
    .hwrite(hwrite),
    .hready_in(hready),
    .hrdata(hrdata),
    .hresp(hresp),
    .hready_out(hready)
);

`include "sdp_task.sv"

endmodule