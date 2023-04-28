//===============================================
//Name          : icache_ctrl
//Author        : angrybird
//Email         : 
//Date          : 2023-03-04
//Description   : icache_ctrl to mem_if / cache sram
//===============================================

// TBD
// addr[31:11]  : tag    
// addr[10:04]  : index  for 128 sets
// addr[03:00]  : offset for 128bit/16bytes
// #####################################
// way         : 2,4,8,16 ways
// data_size   : size = ways * index * bytes = 4 * 128 * 16 = 4 * 2048 bytes = 4 * 2KB
// #####################################
// cache_line
// tag_ram ：valid + dirty + tag_data
// data_ram: data 
// #####################################
module icache_ctrl 
    import urv_cfg::*;
    import urv_typedef::*; 
(
    input  logic                                            clk,
    input  logic                                            rstn,
    // cfg
    input  logic                                            cfg_icache_en,     // icache active high 
    // from core
    input  logic                                            core_mem_req_valid,
    output logic                                            core_mem_req_ready,
    input  mem_req_t                                        core_mem_req, 

    output logic                                            core_mem_resp_valid,
    input  logic                                            core_mem_resp_ready,
    output mem_resp_t                                       core_mem_resp,

    // to memory
    output logic                                            imem_req_valid,
    input  logic                                            imem_req_ready,
    output mem_req_t                                        imem_req, 

    input  logic                                            imem_resp_valid,
    output logic                                            imem_resp_ready,
    input  mem_resp_t                                       imem_resp,

    // to tag_ram
    output logic [CACHE_WAY_NUM-1:0]                        tag_ram_csn,
    output logic [CACHE_WAY_NUM-1:0]                        tag_ram_wen,
    output logic [CACHE_WAY_NUM-1:0][CACHE_TAG_MASK_W-1:0]  tag_ram_web,
    output logic [CACHE_WAY_NUM-1:0][0:0]                   tag_ram_addr,
    output logic [CACHE_WAY_NUM-1:0][CACHE_TAG_W-1:0]       tag_ram_din,
    input  logic [CACHE_WAY_NUM-1:0][CACHE_TAG_W-1:0]       tag_ram_dout,

    // to data_ram    
    output logic [CACHE_WAY_NUM-1:0]                        data_ram_csn,
    output logic [CACHE_WAY_NUM-1:0]                        data_ram_wen,
    output logic [CACHE_WAY_NUM-1:0][CACHE_DATA_MASK_W-1:0] data_ram_web,
    output logic [CACHE_WAY_NUM-1:0][CACHE_INDEX_W-1:0]     data_ram_addr,
    output logic [CACHE_WAY_NUM-1:0][CACHE_DATA_W-1:0]      data_ram_din,
    input  logic [CACHE_WAY_NUM-1:0][CACHE_DATA_W-1:0]      data_ram_dout
);

// cache state enum
typedef enum logic [1:0] {
    IDLE            = 2'b00,
    TAG_CMP         = 2'b01,
    REFILL          = 2'b10,
    FILL_DONE       = 2'b11
} cache_state_t;

// refill state enum
typedef enum logic [1:0] {
    FILL_IDLE       = 2'b00,
    FILL_FETCH_REQ  = 2'b01,
    FILL_FETCH_RESP = 2'b10
} fill_state_t;

// state declare
logic [1:0]         curt_cache_state;
logic [1:0]         next_cache_state;

logic [1:0]         curt_fill_state;
logic [1:0]         next_fill_state;

// state change condition
logic               cache_fill_done;
logic               fill_fetch_req_done;
logic               fill_fetch_resp_done;
logic               cache_hit;
logic               cache_req_en;
logic               prefill_req_en;
logic               refill_req_en;
logic               fill_req_en;

// state confirm
logic               is_idle;
logic               is_tag_cmp;
logic               is_refill;

logic               is_fill_idle;
logic               is_fill_fetch_req;
logic               is_fill_fetch_resp;

// mem_if to cache
logic               cache_mem_req_valid;
logic               cache_mem_req_ready;
mem_req_t           cache_mem_req;

logic               cache_mem_resp_valid;
logic               cache_mem_resp_ready;
mem_resp_t          cache_mem_resp;

// refill to tag_ram
logic [CACHE_WAY_NUM-1:0]                           refill_tag_ram_csn;
logic [CACHE_WAY_NUM-1:0]                           refill_tag_ram_wen;
logic [CACHE_WAY_NUM-1:0][CACHE_TAG_MASK_W-1:0]     refill_tag_ram_web;
logic [CACHE_WAY_NUM-1:0][0:0]                      refill_tag_ram_addr;
logic [CACHE_WAY_NUM-1:0][CACHE_TAG_W-1:0]          refill_tag_ram_din;
logic [CACHE_WAY_NUM-1:0][CACHE_TAG_W-1:0]          refill_tag_ram_dout;

// refill to data_ram    
logic [CACHE_WAY_NUM-1:0]                           refill_data_ram_csn;
logic [CACHE_WAY_NUM-1:0]                           refill_data_ram_wen;
logic [CACHE_WAY_NUM-1:0][CACHE_DATA_MASK_W-1:0]    refill_data_ram_web;
logic [CACHE_WAY_NUM-1:0][CACHE_INDEX_W-1:0]        refill_data_ram_addr;
logic [CACHE_WAY_NUM-1:0][CACHE_DATA_W-1:0]         refill_data_ram_din;
logic [CACHE_WAY_NUM-1:0][CACHE_DATA_W-1:0]         refill_data_ram_dout;

// prefecth to tag_ram
logic [CACHE_WAY_NUM-1:0]                           ifetch_tag_ram_csn;
logic [CACHE_WAY_NUM-1:0]                           ifetch_tag_ram_wen;
logic [CACHE_WAY_NUM-1:0][CACHE_TAG_MASK_W-1:0]     ifetch_tag_ram_web;
logic [CACHE_WAY_NUM-1:0][0:0]                      ifetch_tag_ram_addr;
logic [CACHE_WAY_NUM-1:0][CACHE_TAG_W-1:0]          ifetch_tag_ram_din;
logic [CACHE_WAY_NUM-1:0][CACHE_TAG_W-1:0]          ifetch_tag_ram_dout;


// prefecth to data_ram    
logic [CACHE_WAY_NUM-1:0]                           ifetch_data_ram_csn;
logic [CACHE_WAY_NUM-1:0]                           ifetch_data_ram_wen;
logic [CACHE_WAY_NUM-1:0][CACHE_DATA_MASK_W-1:0]    ifetch_data_ram_web;
logic [CACHE_WAY_NUM-1:0][CACHE_INDEX_W-1:0]        ifetch_data_ram_addr;
logic [CACHE_WAY_NUM-1:0][CACHE_DATA_W-1:0]         ifetch_data_ram_din;
logic [CACHE_WAY_NUM-1:0][CACHE_DATA_W-1:0]         ifetch_data_ram_dout;
logic                    [CACHE_DATA_W-1:0]         ifetch_data_ram_dout_mux;

// fetch dout array
logic [CACHE_OFFSET_W-2-1:0]                        ifetch_data_ram_off;
logic [CACHE_OFFSET_W-2-1:0]                        rff_ifetch_data_ram_off;
logic [CACHE_DATA_DP_W-1:0][MEM_FETCH_W-1:0]        data_ram_dout_array;
logic [CACHE_DATA_DP_W-1:0][MEM_FETCH_W-1:0]        data_ram_din_array;

// struct declare
cache_line_t                cache_line, rff_cache_line;
cache_tag_t                 cache_tag;

// cache ways/sets
logic [CACHE_WAY_NUM-1:0]   cache_tag_val;
logic [CACHE_WAY_NUM-1:0]   cache_tag_hit;
// logic                       cache_hit;
logic                       cache_miss;

// direct router to cache
// todo
// need support disable cache
assign cache_mem_req_valid = core_mem_req_valid;
assign core_mem_req_ready  = cache_mem_req_ready;
assign cache_mem_req       = core_mem_req;

assign core_mem_resp_valid = cache_mem_resp_valid;
assign cache_mem_resp_ready= core_mem_resp_ready;
assign core_mem_resp       = cache_mem_resp;

// dispatch addr to cache line parameter
assign cache_line.tag      = cache_mem_req.req_addr[CACHE_TAG_MSB:CACHE_TAG_LSB];
assign cache_line.index    = cache_mem_req.req_addr[CACHE_INDEX_MSB:CACHE_INDEX_LSB];
assign cache_line.offset   = cache_mem_req.req_addr[CACHE_OFFSET_MSB:CACHE_OFFSET_LSB];

// state change
stdffrv #($bits(cache_state_t)) rff_cache_state_u (
    .clk(clk),
    .rstn(rstn),
    .rst_val(IDLE),
    .d(next_cache_state),
    .q(curt_cache_state) 
);   

// power on -> init cache
logic is_cache_init;
logic cache_init_en;
assign cache_init_en = ~is_cache_init & cache_fill_done;

stdffre #(1) rff_cache_init_u (
    .clk(clk),
    .rstn(rstn),
    .en(cache_init_en),
    .d(1'b1),
    .q(is_cache_init)
); 

// rff_cache_line
logic  cache_mem_req_handshaked;
logic  cache_mem_resp_handshaked;
logic  rff_cache_line_en;

assign cache_mem_req_ready       = is_idle | is_tag_cmp;
assign cache_mem_resp_valid      = is_tag_cmp & cache_hit;
assign cache_fill_done           = fill_fetch_resp_done;     

assign is_idle                   = curt_cache_state == IDLE;
assign is_tag_cmp                = curt_cache_state == TAG_CMP;
assign is_refill                 = curt_cache_state == REFILL;

assign cache_mem_req_handshaked  = cache_mem_req_valid & cache_mem_req_ready;
assign cache_mem_resp_handshaked = cache_mem_resp_valid & cache_mem_resp_ready;
assign rff_cache_line_en         = cache_mem_req_handshaked | cache_mem_resp_handshaked;
stdffre #($bits(cache_line_t)) rff_cache_line_u (
    .clk(clk),
    .rstn(rstn),
    .en(rff_cache_line_en),
    .d(cache_line),
    .q(rff_cache_line)
); 

// state transition
logic  cache_fill_done_en;
logic  cache_fill_done_d;
assign cache_fill_done_en = cache_fill_done | cache_mem_resp_valid;
assign cache_fill_done_d  = cache_fill_done | ~cache_mem_resp_valid;
stdffre #(1) rff_cache_fill_done_u (
    .clk(clk),
    .rstn(rstn),
    .en(cache_fill_done_en),
    .d(cache_fill_done_d),
    .q(rff_cache_fill_done)
);

logic tag_cmp_req;
logic next_tag_cmp_req;
logic refill_req_up;
logic refill_req_set;
logic refill_req_clr;
logic refill_req_d;
logic refill_req;
logic ifetch_req;
assign tag_cmp_req      = rff_cache_fill_done | cache_mem_req_valid;
assign next_tag_cmp_req = cache_hit & cache_mem_req_valid;
assign refill_req_set   = cache_mem_req_valid;
assign refill_req_clr   = refill_req;
assign refill_req_up    = refill_req_set | refill_req_clr;
assign refill_req_d     = refill_req_set | ~refill_req_clr;
assign ifetch_req       = (is_idle & tag_cmp_req) | (is_tag_cmp);

stdffre #(1) rff_refill_req_u (
    .clk(clk),
    .rstn(rstn),
    .en(refill_req_up),
    .d(refill_req_d),
    .q(refill_req)
);

always_comb  begin
    next_cache_state = curt_cache_state;
    case(curt_cache_state)
        IDLE  : begin
            if(~cfg_icache_en) begin
                next_cache_state = IDLE;
            end else begin
                next_cache_state = (~is_cache_init & refill_req) ? REFILL 
                                 : (is_cache_init & tag_cmp_req) ? TAG_CMP
                                 : IDLE;
            end  
        end
        TAG_CMP : begin
            next_cache_state = ~cache_hit ? REFILL 
                             : next_tag_cmp_req ? TAG_CMP
                             : IDLE;
        end
        REFILL : begin
            next_cache_state = cache_fill_done ? IDLE : REFILL;
        end
    endcase
end

// state operate
// idle
// ifetch
assign ifetch_tag_ram_csn     = {CACHE_WAY_NUM{~(cfg_icache_en & ifetch_req)}};
assign ifetch_tag_ram_wen     = {CACHE_WAY_NUM{1'b1}}; // icache fetch only read
assign ifetch_tag_ram_web     = {CACHE_WAY_NUM{1'b0}};
assign ifetch_tag_ram_din     = {CACHE_WAY_NUM{32'h0}};
assign ifetch_tag_ram_addr    = {CACHE_WAY_NUM{1'b0}};

// tag compare
generate
    for(genvar i = 0; i < CACHE_WAY_NUM; i = i + 1) begin : IFETCH_TAG_RAM_DOUT_GEN
        assign cache_tag_val[i] = ifetch_tag_ram_dout[i][CACHE_TAG_W-1];
        assign cache_tag_hit[i] = cache_tag_val[i] & (ifetch_tag_ram_dout[i][CACHE_TAG_W-3:0] == rff_cache_line.tag);
    end
endgenerate

// get data from data_ram
assign ifetch_data_ram_csn    = {CACHE_WAY_NUM{~(cfg_icache_en & ifetch_req)}};
assign ifetch_data_ram_wen    = {CACHE_WAY_NUM{1'b1}}; // icache fetch only read 
assign ifetch_data_ram_web    = {CACHE_WAY_NUM{1'b0}};
assign ifetch_data_ram_din    = {CACHE_WAY_NUM{32'h0}};
assign ifetch_data_ram_addr   = {CACHE_WAY_NUM{(is_idle & rff_cache_fill_done) ? rff_cache_line.index : cache_line.index}};
assign ifetch_data_ram_off    = (is_idle & rff_cache_fill_done) ? rff_cache_line.offset[CACHE_OFFSET_W-1:2] : cache_line.offset[CACHE_OFFSET_W-1:2];

// state change
stdffr #(CACHE_OFFSET_W-2) rff_ifetch_data_ram_off_u (
    .clk(clk),
    .rstn(rstn),
    .d(ifetch_data_ram_off),
    .q(rff_ifetch_data_ram_off) 
); 

always_comb begin
    ifetch_data_ram_dout_mux = '0;
    for(int j = 0; j < CACHE_WAY_NUM; j = j + 1) begin : IFETCH_DATA_RAM_DOUT_GEN
        if(cache_tag_hit[j]) begin
            ifetch_data_ram_dout_mux = ifetch_data_ram_dout[j];
        end
    end
end
assign cache_hit              = |cache_tag_hit;

// refill/prefill
assign refill_req_en          = is_tag_cmp & ~cache_hit;
assign prefill_req_en         = is_idle & refill_req;
assign fill_req_en            = refill_req_en | prefill_req_en;

assign is_fill_idle           = (curt_fill_state == FILL_IDLE);
assign is_fill_fetch_req      = (curt_fill_state == FILL_FETCH_REQ);
assign is_fill_fetch_resp     = (curt_fill_state == FILL_FETCH_RESP);

// state change
stdffrv #($bits(fill_state_t)) rff_fill_state_u (
    .clk(clk),
    .rstn(rstn),
    .rst_val(FILL_IDLE),
    .d(next_fill_state),
    .q(curt_fill_state) 
);  

always_comb  begin
    next_fill_state = curt_fill_state;
    case(curt_fill_state)
        FILL_IDLE  : begin
            next_fill_state = fill_req_en ? FILL_FETCH_REQ : FILL_IDLE;
        end
        FILL_FETCH_REQ : begin
            next_fill_state = fill_fetch_req_done ? FILL_FETCH_RESP : FILL_FETCH_REQ;         
        end
        FILL_FETCH_RESP : begin
            next_fill_state = fill_fetch_resp_done ? FILL_IDLE : FILL_FETCH_RESP;         
        end
    endcase
end

// rff_fill_fetch_req
cache_line_t rff_fill_cache_line;
cache_line_t fill_cache_line_d; 
logic        rff_fill_cache_line_en;
assign fill_cache_line_d.index = rff_fill_cache_line.index;
assign fill_cache_line_d.offset= rff_fill_cache_line.offset;
assign fill_cache_line_d.tag   = rff_fill_cache_line.tag + 1'b1;
stdffref #($bits(cache_line_t)) rff_fill_cache_line_u (
    .clk(clk),
    .rstn(rstn),
    .flush(fill_req_en),
    .flush_val(rff_cache_line),
    .en(rff_fill_cache_line_en),
    .d(fill_cache_line_d),
    .q(rff_fill_cache_line)
);

// rff mem busrt lenth
logic [CACHE_PREFILL_W-1:0] fill_burst, rff_fill_burst;

assign fill_burst = prefill_req_en ? {1'b1, {(CACHE_PREFILL_W-1){1'b0}}} : {1'b1, {(CACHE_REFILL_W-1){1'b0}}};

stdffre #(CACHE_PREFILL_W) rff_fill_burst_u (
    .clk(clk),
    .rstn(rstn),
    .en(fill_req_en),
    .d(fill_burst),
    .q(rff_fill_burst)
); 

// fetch bus
assign imem_req_valid       = is_fill_fetch_req;
assign imem_req.req_type    = MEM_READ;
assign imem_req.req_addr    = {rff_fill_cache_line.tag, {CACHE_INDEX_W{1'b0}}, {CACHE_OFFSET_W{1'b0}}};
assign imem_req.req_mask    = {MEM_MASK_W{1'b1}};
assign imem_req.req_data    = {MEM_DATA_W{1'b0}};
assign imem_req.req_burst   = rff_fill_burst;

assign imem_resp_ready      = 1'b1;

assign fill_fetch_req_done  = imem_req_valid & imem_req_ready;
assign fill_fetch_resp_done = imem_resp_valid & imem_resp.resp_last;

// fill_offset_wrptr
logic                        fill_offset_wrptr_en;
logic                        fill_offset_wrptr_flush;
logic [CACHE_OFFSET_W-2-1:0] fill_offset_wrptr_val;
logic [CACHE_OFFSET_W-2-1:0] fill_offset_wrptr;
logic [CACHE_OFFSET_W-2-1:0] next_fill_offset_wrptr;
logic                        fill_offset_full;

assign fill_offset_wrptr_flush = fill_req_en & is_fill_idle;
assign fill_offset_wrptr_val   = {(CACHE_OFFSET_W-2){1'b0}};
assign fill_offset_wrptr_en    = imem_resp_valid & is_fill_fetch_resp;
assign next_fill_offset_wrptr  = fill_offset_wrptr + 1'b1;
assign fill_offset_full        = imem_resp_valid & (fill_offset_wrptr == {(CACHE_OFFSET_W-2){1'b1}});
stdffref #(CACHE_OFFSET_W-2) rff_fill_offset_wrptr_u (
    .clk(clk),
    .rstn(rstn),
    .flush(fill_offset_wrptr_flush),
    .flush_val(fill_offset_wrptr_val),
    .en(fill_offset_wrptr_en),
    .d(next_fill_offset_wrptr),
    .q(fill_offset_wrptr)
);

// fill_index_wrptr
logic                     fill_index_wrptr_en;
logic                     fill_index_wrptr_flush;
logic [CACHE_INDEX_W-1:0] fill_index_wrptr_val;
logic [CACHE_INDEX_W-1:0] fill_index_wrptr;
logic [CACHE_INDEX_W-1:0] next_fill_index_wrptr;
logic                     fill_index_full;

assign fill_index_wrptr_flush = fill_req_en & is_fill_idle;
assign fill_index_wrptr_val   = {CACHE_INDEX_W{1'b0}};
// assign fill_index_wrptr_en    = imem_resp_valid & is_fill_fetch_resp;
assign fill_index_wrptr_en    = fill_offset_full & is_fill_fetch_resp;
assign next_fill_index_wrptr  = fill_index_wrptr + 1'b1;
assign fill_index_full        = (fill_index_wrptr == {CACHE_INDEX_W{1'b1}});

stdffref #(CACHE_INDEX_W) rff_fill_index_wrptr_u (
    .clk(clk),
    .rstn(rstn),
    .flush(fill_index_wrptr_flush),
    .flush_val(fill_index_wrptr_val),
    .en(fill_index_wrptr_en),
    .d(next_fill_index_wrptr),
    .q(fill_index_wrptr)
);

// fill_way_wrptr
logic                   fill_way_wrptr_en;
logic                   fill_way_wrptr_flush;
logic [CACHE_WAY_W-1:0] fill_way_wrptr_val;
logic [CACHE_WAY_W-1:0] fill_way_wrptr;
logic [CACHE_WAY_W-1:0] next_fill_way_wrptr;

assign fill_way_wrptr_flush   = fill_req_en & is_fill_idle & is_refill & ~is_cache_init;
assign fill_way_wrptr_val     = {CACHE_WAY_W{1'b0}};
assign fill_way_wrptr_en      = fill_index_full & fill_offset_full & is_fill_fetch_resp;
assign next_fill_way_wrptr    = fill_way_wrptr + 1'b1;
assign rff_fill_cache_line_en = fill_way_wrptr_en;
stdffref #(CACHE_WAY_W) rff_fill_way_wrptr_u (
    .clk(clk),
    .rstn(rstn),
    .flush(fill_way_wrptr_flush),
    .flush_val(fill_way_wrptr_val),
    .en(fill_way_wrptr_en),
    .d(next_fill_way_wrptr),
    .q(fill_way_wrptr) 
);

// sram fill operate
assign cache_tag.val                         = 1'b1;
assign cache_tag.dir                         = 1'b0;
assign cache_tag.tag                         = rff_fill_cache_line.tag;

logic  [CACHE_DATA_MASK_W-1:0] refill_data_ram_bmask; // byte mask
logic  [CACHE_DATA_DP_W-1:0]   refill_data_ram_wmask; // word mask
logic  [CACHE_OFFSET_W-2-1:0]  refill_data_ram_woffset; // word offset

assign refill_data_ram_woffset          = fill_offset_wrptr;
assign data_ram_din_array               = {CACHE_DATA_DP_W{imem_resp.resp_data}};

generate
    for(genvar i = 0; i < CACHE_DATA_DP_W; i = i + 1) begin : FILL_DATA_RAM_MASK_GEN
        assign refill_data_ram_wmask[i] = (refill_data_ram_woffset == i);
        assign refill_data_ram_bmask[(i+1)*4 - 1 -: 4] = {4{refill_data_ram_wmask[i]}};
    end
endgenerate

generate
    for(genvar i = 0; i < CACHE_WAY_NUM; i = i + 1) begin : FILL_CACHE_OPERATE_GEN
        assign refill_tag_ram_csn  [i]  = (fill_way_wrptr == i) ? ~fill_offset_wrptr_en : 1'b1;
        assign refill_data_ram_csn [i]  = (fill_way_wrptr == i) ? ~fill_offset_wrptr_en : 1'b1;

        assign refill_tag_ram_wen  [i]  = ~(fill_way_wrptr == i);
        assign refill_tag_ram_web  [i]  = {CACHE_TAG_MASK_W{~(fill_way_wrptr == i)}};
        assign refill_tag_ram_addr [i]  = 1'b0;
        assign refill_tag_ram_din  [i]  = {cache_tag.val, cache_tag.dir, cache_tag.tag};

        assign refill_data_ram_wen [i]  = ~(fill_way_wrptr == i);
        assign refill_data_ram_web [i]  = ~({CACHE_DATA_MASK_W{(fill_way_wrptr == i)}} & refill_data_ram_bmask);
        assign refill_data_ram_addr[i]  = fill_index_wrptr;
        assign refill_data_ram_din [i]  = data_ram_din_array;
    end
endgenerate

// mux sram operate
assign tag_ram_csn           = is_fill_idle ? ifetch_tag_ram_csn   : refill_tag_ram_csn;
assign tag_ram_wen           = is_fill_idle ? ifetch_tag_ram_wen   : refill_tag_ram_wen;
assign tag_ram_web           = is_fill_idle ? ifetch_tag_ram_web   : refill_tag_ram_web;
assign tag_ram_addr          = is_fill_idle ? ifetch_tag_ram_addr  : refill_tag_ram_addr;
assign tag_ram_din           = is_fill_idle ? ifetch_tag_ram_din   : refill_tag_ram_din;

assign data_ram_csn          = is_fill_idle ? ifetch_data_ram_csn  : refill_data_ram_csn;
assign data_ram_wen          = is_fill_idle ? ifetch_data_ram_wen  : refill_data_ram_wen;
assign data_ram_web          = is_fill_idle ? ifetch_data_ram_web  : refill_data_ram_web;
assign data_ram_addr         = is_fill_idle ? ifetch_data_ram_addr : refill_data_ram_addr;
assign data_ram_din          = is_fill_idle ? ifetch_data_ram_din  : refill_data_ram_din;

// put data_ram_dout into data_ram_dout_array
assign ifetch_data_ram_dout         = data_ram_dout;
assign ifetch_tag_ram_dout          = tag_ram_dout;

assign data_ram_dout_array          = ifetch_data_ram_dout_mux;
assign cache_mem_resp.resp_data     = data_ram_dout_array[rff_ifetch_data_ram_off];
assign cache_mem_resp.resp_last     = cache_mem_resp_valid;

endmodule