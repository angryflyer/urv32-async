//===============================================
//Name          : mem2ahb
//Author        : angrybird
//Email         : 
//Date          : 2023-03-05
//Description   : mem2ahb
//              : ahb -> support single only
//              : mem -> support burst read and single read
//              : mem -> support single write
//===============================================
module mem2ahb
    import urv_cfg::*;
    import urv_typedef::*; 
#(
    parameter N_AW = 32,
    parameter N_DW = 32    
) (
    input  logic                    clk,
    input  logic                    rstn,
    //from mem master
    input  logic                    mem_req_valid,
    output logic                    mem_req_ready,
    input  mem_req_t                mem_req,

    output logic                    mem_resp_valid,
    input  logic                    mem_resp_ready,
    output mem_resp_t               mem_resp,
    //ahb master to ahb slave
    output logic [1:0]              htrans,
    output logic [2:0]              hburst,
    output logic [N_AW-1:0]         haddr,
    output logic [2:0]              hsize,
    output logic [N_DW-1:0]         hwdata,
    output logic                    hwrite,
    output logic                    hmastlock,
    output logic [6:0]              hprot,
    // from slave
    input  logic [N_DW-1:0]         hrdata,
    input  logic                    hresp,
    input  logic                    hready
);
    localparam MEM_REQ_T_W  = $bits(mem_req_t);
    localparam MEM_RESP_T_W = $bits(mem_resp_t);

    typedef enum logic [1:0] {
        HTRANS_IDLE     = 2'b00,
        HTRANS_BUSY     = 2'b01,
        HTRANS_NONSEQ   = 2'b10,
        HTRANS_SEQ      = 2'b11
    } htrans_type_t;

    typedef enum logic [2:0] {
        SIGNLE          = 3'b000,
        INCR            = 3'b001,
        WRAP4           = 3'b010,
        INCR4           = 3'b011,
        WRAP8           = 3'b100,
        INCR8           = 3'b101,
        WRAP16          = 3'b110,
        INCR16          = 3'b111
    } hburst_type_t;

    typedef enum logic [2:0] {
        BYTE            = 3'b000,// 1/4word 8
        HALF            = 3'b001,// 1/2word 16
        WORD            = 3'b010,// 1-word  32
        DWORD           = 3'b011,// 2-word  64
        FWORD           = 3'b100,// 4-word  128
        EWORD           = 3'b101,// 8-word  256
        SWORD           = 3'b110,// 16-word 512
        TWORD           = 3'b111 // 32-word 1024
    } hsize_type_t;

    typedef enum logic [2:0] {
        AHB_ADDR        = 3'b000,
        AHB_WDATA       = 3'b001, // data single //nonseq or first seq
        AHB_RDATA       = 3'b010,
        AHB_RESP_S      = 3'b011,
        AHB_PRE_B       = 3'b100,
        AHB_ADDR_B      = 3'b101,  // data burst  //seq
        AHB_RESP_B      = 3'b110
    } ahb_state_t;

    logic [1:0] haddr_off_d;

    logic [2:0] curt_ahb_state;
    logic [2:0] next_ahb_state;

    logic       mem_req_handshaked;
    logic       mem_resp_handshaked;
    logic       mem_req_wr;
    logic       mem_req_rd;

    logic       ahb_req_valid;

    logic       is_ahb_idle, is_ahb_busy, is_ahb_nonreq;
    logic       is_ahb_addr, is_ahb_wdata, is_ahb_rdata, is_ahb_resp_s;
    logic       is_ahb_pre_b;
    logic       is_ahb_addr_b, is_ahb_resp_b;
    logic       is_ahb_resp;

    logic       is_ahb_rd;
    logic       is_ahb_wr;

    logic       ff_mem_req_flush;
    logic       ff_req_en;
    mem_req_t   mem_req_flush_val;
    mem_req_t   mem_req_d;
    mem_req_t   ff_mem_req;
    logic       ff_mem_req_wr;
    // signle
    logic       burst_s;
    // signle
    logic       rff_burst_s;
    // burst transfer done if burst count equal 1'b1
    logic       burst_done; 

    logic [2:0] hsize_d, ff_hsize;

    logic [MEM_BURST_W-1:0] ff_ahb_resp_burst;

    assign mem_req_handshaked = mem_req_valid  & mem_req_ready;
    assign mem_resp_handshaked= mem_resp_valid & mem_resp_ready;

    assign mem_req_wr         = (mem_req.req_type == MEM_WRITE);
    assign mem_req_rd         = (mem_req.req_type == MEM_READ);

//    assign ahb_req_valid      = mem_req_valid  & hready;
    assign is_ahb_rd          = mem_req_valid & hready & mem_req_rd;
    assign is_ahb_wr          = mem_req_valid & hready & mem_req_wr;

    assign ff_mem_req_flush   = is_ahb_addr
                              | is_ahb_resp_s
                              | (is_ahb_resp_b & burst_done);
    assign ff_req_en          = (is_ahb_addr_b & hready) | (is_ahb_resp_s & mem_resp_ready) | (is_ahb_resp_b & hready & mem_resp_ready);

    assign ff_mem_req_wr      = (ff_mem_req.req_type  == MEM_WRITE);

    assign burst_s            = (mem_req.req_burst    == 1'b1);
    assign rff_burst_s        = (ff_mem_req.req_burst == 1'b1);

    assign mem_req_flush_val.req_type = mem_req.req_type;
    assign mem_req_flush_val.req_mask = mem_req.req_mask;
    assign mem_req_flush_val.req_data = mem_req.req_data;
    assign mem_req_flush_val.req_burst= mem_req.req_burst;
    assign mem_req_flush_val.req_addr = mem_req.req_addr;

    assign mem_req_d.req_type = ff_mem_req.req_type;
    assign mem_req_d.req_mask = ff_mem_req.req_mask;
    assign mem_req_d.req_data = ff_mem_req.req_data;
    assign mem_req_d.req_burst= ff_mem_req.req_burst - 1'b1;
    assign mem_req_d.req_addr = ff_mem_req.req_addr  + 3'h4;
    always_comb begin
        case(mem_req.req_mask)
            4'h1, 4'h2, 4'h4, 4'h8 : begin 
                hsize_d = BYTE;
            end
            4'b1100, 4'b0011       : begin
                hsize_d = HALF;
            end
            default                : begin
                hsize_d = WORD;
            end
        endcase
    end

    stdffr #($bits(ahb_state_t)) ff_ahb_state_u (
        .clk(clk),
        .rstn(rstn),
        .d(next_ahb_state),
        .q(curt_ahb_state) 
    );

    stdffref #(MEM_REQ_T_W) ff_mem_req_u (
        .clk(clk),
        .rstn(rstn),
        .flush(ff_mem_req_flush),
        .flush_val(mem_req_flush_val),
        .en(ff_req_en),
        .d(mem_req_d),
        .q(ff_mem_req)
    );

    stdffrve #($bits(hsize_type_t)) ff_hsize_u (
        .clk(clk),
        .rstn(rstn),
        .rst_val(WORD),
        .en(ff_mem_req_flush),
        .d(hsize_d),
        .q(ff_hsize) 
    );

    stdffre #(1) ff_resp_burst_u (
        .clk(clk),
        .rstn(rstn),
        .en(ff_req_en),
        .d(rff_burst_s),
        .q(burst_done)
    );

    logic            ff_resp_data_en;
    logic [N_DW-1:0] ff_hrdata;
    assign ff_resp_data_en = (is_ahb_rdata | is_ahb_addr_b) & hready;
    stdffre #(N_DW) ff_resp_data_u (
        .clk(clk),
        .rstn(rstn),
        .en(ff_resp_data_en),
        .d(hrdata),
        .q(ff_hrdata)
    );
    
    logic ff_is_ahb_addr_b_en;
    logic ff_is_ahb_addr_b_set;
    logic ff_is_ahb_addr_b_clr;
    logic is_ahb_addr_b_d;
    logic ff_is_ahb_addr_b;
    assign ff_is_ahb_addr_b_set = is_ahb_addr_b & hready;
    assign ff_is_ahb_addr_b_clr = is_ahb_resp_b & hready & mem_resp_ready & ff_is_ahb_addr_b;
    assign ff_is_ahb_addr_b_en  = ff_is_ahb_addr_b_set | ff_is_ahb_addr_b_clr;
    assign is_ahb_addr_b_d      = ff_is_ahb_addr_b_set | ~ff_is_ahb_addr_b_clr;
    stdffre #(1) ff_is_ahb_addr_b_u (
        .clk(clk),
        .rstn(rstn),
        .en(ff_is_ahb_addr_b_en),
        .d(is_ahb_addr_b_d),
        .q(ff_is_ahb_addr_b)
    );

    always_comb  begin
        next_ahb_state = curt_ahb_state;
        case(curt_ahb_state)
            AHB_ADDR   : begin
                next_ahb_state = is_ahb_rd ? (burst_s  ? AHB_RDATA : AHB_PRE_B)
                               : (is_ahb_wr & burst_s) ? AHB_WDATA
                               : AHB_ADDR;
            end
            AHB_WDATA : begin
                next_ahb_state = hready ? AHB_RESP_S : AHB_WDATA;
            end
            AHB_RDATA : begin
                next_ahb_state = hready ? AHB_RESP_S : AHB_RDATA;
            end
            AHB_RESP_S : begin
                next_ahb_state = mem_resp_ready ? (is_ahb_rd ? (burst_s  ? AHB_RDATA : AHB_PRE_B)
                               : (is_ahb_wr & burst_s) ? AHB_WDATA : AHB_ADDR)
                               : AHB_RESP_S;
            end
            AHB_PRE_B  : begin
                next_ahb_state = hready ? AHB_ADDR_B : AHB_PRE_B;
            end
            AHB_ADDR_B : begin
                next_ahb_state = hready ? AHB_RESP_B : AHB_ADDR_B;
            end
            AHB_RESP_B : begin
                next_ahb_state = (mem_resp_ready & hready & burst_done) ? AHB_ADDR
                               : AHB_RESP_B;
            end
        endcase
    end
    
    assign is_ahb_nonreq = (is_ahb_addr & mem_req_valid) 
                         | (is_ahb_resp_s & mem_resp_ready & mem_req_valid)
                         | (is_ahb_pre_b)
                         | (is_ahb_resp_b & hready & mem_resp_ready);
    
//    assign is_ahb_idle   = (is_ahb_addr  | (hready & is_ahb_resp_s & mem_resp_ready) | (hready & is_ahb_resp_b & mem_resp_ready & burst_done)) & ~mem_req_valid;

    assign is_ahb_busy   = (is_ahb_resp_s & ~mem_resp_ready) | (is_ahb_resp_b & hready & ~mem_resp_ready);

    assign is_ahb_addr   = (curt_ahb_state == AHB_ADDR);
    assign is_ahb_wdata  = (curt_ahb_state == AHB_WDATA);
    assign is_ahb_rdata  = (curt_ahb_state == AHB_RDATA);
    assign is_ahb_resp_s = (curt_ahb_state == AHB_RESP_S);
    assign is_ahb_pre_b  = (curt_ahb_state == AHB_PRE_B);
    assign is_ahb_addr_b = (curt_ahb_state == AHB_ADDR_B);
    assign is_ahb_resp_b = (curt_ahb_state == AHB_RESP_B);
    assign is_ahb_resp   = is_ahb_resp_s | is_ahb_resp_b;

    assign htrans        = is_ahb_busy ? HTRANS_BUSY
                         : is_ahb_nonreq ? HTRANS_NONSEQ
                         : HTRANS_IDLE;

    assign haddr         = (is_ahb_addr | is_ahb_resp_s) ? mem_req_flush_val.req_addr : ff_mem_req.req_addr;
    assign hburst        = SIGNLE;
    assign hsize         = (is_ahb_addr | is_ahb_resp_s) ? hsize_d : ff_hsize;
    assign hwdata        = ff_mem_req.req_data;
    assign hmastlock     = 1'b0;
    assign hprot         = 7'h3;
    
    // fixme
//    assign hwrite        = is_ahb_resp ? ff_mem_req_wr : (~is_ahb_wdata & mem_req_wr);
    assign hwrite        = (is_ahb_addr | is_ahb_resp_s) & is_ahb_wr;
    assign mem_resp_valid     = is_ahb_resp_s | (is_ahb_resp_b & hready);
    assign mem_req_ready      = hready;

    assign mem_resp.resp_type = ff_mem_req.req_type;
    assign mem_resp.resp_data = (is_ahb_resp_s | ff_is_ahb_addr_b) ? ff_hrdata : hrdata;
    assign mem_resp.resp_last = is_ahb_resp_s | (is_ahb_resp_b & burst_done);

endmodule