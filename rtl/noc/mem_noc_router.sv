module mem_noc_router
    import urv_cfg::*;
    import urv_typedef::*; 
#(
    parameter ENABLE_DEC_TAG= 0,
    parameter ENABLE_DEC_IDX= 1,
    parameter DEC_NUM       = (ENABLE_DEC_TAG & ~ENABLE_DEC_IDX) ? 2 : 4,
    parameter DEC_TAG_H     = 31,
    parameter DEC_TAG_L     = 16,
    parameter DEC_IDX_H     = 15,
    parameter DEC_IDX_L     = 12,
    parameter DEC_TAG_VAL   = {(DEC_TAG_H-DEC_TAG_L+1){1'b0}},
    parameter NOC_TID_W     = $clog2(DEC_NUM) + 1
)(
    input  logic                                            clk,
    input  logic                                            rstn,

    //noc master 0 
    input  logic                                            mn_req_valid,
    output logic                                            mn_req_ready,
    input  mem_req_t                                        mn_req,

    output logic                                            mn_resp_valid,
    input  logic                                            mn_resp_ready,
    output mem_resp_t                                       mn_resp,
    //noc slave 0 
    output logic        [DEC_NUM-1:0]                       sn_req_valid,
    input  logic        [DEC_NUM-1:0]                       sn_req_ready,
    output mem_req_t    [DEC_NUM-1:0]                       sn_req,

    input  logic        [DEC_NUM-1:0]                       sn_resp_valid,
    output logic        [DEC_NUM-1:0]                       sn_resp_ready,
    input  mem_resp_t   [DEC_NUM-1:0]                       sn_resp,

    output logic        [DEC_NUM-1:0][NOC_TID_W-1:0]        sn_tid
);
    localparam MEM_REQ_T_W   = $bits(mem_req_t);
    localparam MEM_RESP_T_W  = $bits(mem_resp_t);
    localparam DEC_TAG_W     = DEC_TAG_H - DEC_TAG_L + 1;
    localparam DEC_IDX_W     = DEC_IDX_H - DEC_IDX_L + 1;

    typedef enum logic {
        NOC_REQ  = 1'b0,
        NOC_RESP = 1'b1
    } noc_state_t;

    logic       [DEC_NUM-1:0][NOC_TID_W-1:0]    route_tid_t;//route_tid_t mean temp
    logic       [DEC_NUM-1:0][NOC_TID_W-1:0]    route_tid_or;//route_tid_t mean temp
    logic       [NOC_TID_W-1:0]                 route_tid;

    logic                                       curt_noc_state;
    logic                                       next_noc_state;

    logic       [DEC_TAG_W-1:0]                 dec_tag;
    logic       [DEC_IDX_W-1:0]                 dec_idx;
    logic                                       is_tag_match;
    logic                                       is_idx_match;

    logic       [DEC_NUM-1:0]                   route_valid;
    logic       [DEC_NUM-1:0]                   route_match;
    logic       [DEC_NUM-1:0]                   ff_route_match;
    logic                                       ff_route_match_en;
    logic                                       mn_req_handshaked;
    logic                                       mn_resp_handshaked;

    logic                                       is_noc_req;
    logic                                       is_noc_resp;

    logic                                       mn_req_ready_en;
    logic       [DEC_NUM-1:0]                   mn_sn_req_ready;

    mem_resp_t  [DEC_NUM-1:0]                   mn_sn_resp;
    mem_resp_t  [DEC_NUM-1:0]                   mn_sn_resp_or; 

    // obtain dec_tag and dec_idx
    assign dec_tag      = mn_req.req_addr[DEC_TAG_H:DEC_TAG_L];
    assign dec_idx      = mn_req.req_addr[DEC_IDX_H:DEC_IDX_L];
    assign is_tag_match = (dec_tag == DEC_TAG_VAL);

    // route match/select
    generate
        if(ENABLE_DEC_TAG && ~ENABLE_DEC_IDX) begin
            for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : ROUTE_MATCH_GEN
                if(i == (DEC_NUM - 1)) begin                
                    assign route_match[i] = ~is_tag_match;
                end else begin
                    assign route_match[i] = is_tag_match;
                end
            end
        end else if(~ENABLE_DEC_TAG && ENABLE_DEC_IDX) begin
            for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : ROUTE_MATCH_GEN
                if(i == (DEC_NUM - 1)) begin                
                    assign route_match[i] = (dec_idx >= i);
                end else begin
                    assign route_match[i] = (dec_idx == i);
                end
            end
        end else begin
            for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : ROUTE_MATCH_GEN
                if(i == (DEC_NUM - 1)) begin                
                    assign route_match[i] = ~is_tag_match | (dec_idx >= i);
                end else begin
                    assign route_match[i] = is_tag_match  & (dec_idx == i);
                end
            end
        end
    endgenerate

    // route match/select valid
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : ROUTE_VALID_GEN              
                assign route_valid[i] = mn_req_valid  & route_match[i];
        end
    endgenerate

    // route tid gen
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : ROUTE_TID_GEN
            if(i == (DEC_NUM - 1)) begin                
                assign route_tid_t[i]  = {DEC_IDX_W{route_match[i]}} & (DEC_NUM - 1);
            end else begin
                assign route_tid_t[i]  = {DEC_IDX_W{route_match[i]}} & i;
            end
        end
    endgenerate

    // route tid or gen
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : ROUTE_TID_OR_GEN
            if(i == 0) begin                
                assign route_tid_or[i]  = route_tid[i];
            end else begin
                assign route_tid_or[i]  = route_tid[i] | route_tid_or[i-1];
            end
        end
    endgenerate

    assign route_tid = route_tid_or[DEC_NUM-1];

    assign mn_req_handshaked = mn_req_valid  & mn_req_ready;
    assign mn_resp_handshaked= mn_resp_valid & mn_resp_ready & mn_resp.resp_last;

    assign ff_route_match_en   = (is_noc_req & mn_req_handshaked) | (is_noc_resp & mn_resp_handshaked);

    stdffrv #(1) ff_noc_state_u (
        .clk(clk),
        .rstn(rstn),
        .rst_val(NOC_REQ),
        .d(next_noc_state),
        .q(curt_noc_state) 
    );    

    stdffre #(DEC_NUM) ff_route_match_u (
        .clk(clk),
        .rstn(rstn),
        .en(ff_route_match_en),
        .d(route_match),
        .q(ff_route_match)
    );

    always_comb  begin
        next_noc_state = curt_noc_state;
        case(curt_noc_state)
            NOC_REQ  : begin
                next_noc_state = mn_req_handshaked  ? NOC_RESP : NOC_REQ;
            end
            NOC_RESP : begin
                next_noc_state = ~mn_resp_handshaked ? NOC_RESP 
                                : mn_req_handshaked   ? NOC_RESP
                                : NOC_REQ;            
            end
        endcase
    end

    assign is_noc_req = (curt_noc_state == NOC_REQ );
    assign is_noc_resp= (curt_noc_state == NOC_RESP);

    // sn_req_valid gen
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : SN_REQ_VALID_GEN          
                assign sn_req_valid[i] = (is_noc_req & route_valid[i])
                                       | (is_noc_resp & mn_resp_handshaked & route_valid[i]);
        end
    endgenerate

    // sn_req gen
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : SN_REQ_GEN           
                assign sn_req[i] = mn_req;
        end
    endgenerate

    assign mn_req_ready_en  = is_noc_req | (is_noc_resp & mn_resp_handshaked);

    // mn_req gen
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : MN_SN_REQ_READY_GEN          
                assign mn_sn_req_ready[i] = mn_req_ready_en & route_match[i] & sn_req_ready[i];
        end
    endgenerate

    assign mn_req_ready = |mn_sn_req_ready;

    // sn_resp_ready gen
    // generate
    //     for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : SN_RESP_READY_GEN          
    //             assign sn_resp_ready[i] = ff_route_match[i] & mn_resp_ready;
    //     end
    // endgenerate
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : SN_RESP_READY_GEN          
                assign sn_resp_ready[i] = mn_resp_ready;
        end
    endgenerate

    // mn_resp_valid
    logic [DEC_NUM-1:0] mn_sn_resp_valid;
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : MN_SN_RESP_VALID_GEN          
                assign mn_sn_resp_valid[i] = ff_route_match[i] & sn_resp_valid[i];
        end
    endgenerate

    assign mn_resp_valid = |mn_sn_resp_valid;

    // mn_resp_valid
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : MN_SN_RESP_GEN          
                assign mn_sn_resp[i] = {MEM_RESP_T_W{ff_route_match[i]}} & sn_resp[i];
        end
    endgenerate


    // mn_sn_resp or gen
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : MN_SN_RESP_OR_GEN
            if(i == 0) begin                
                assign mn_sn_resp_or[i]  = mn_sn_resp[i];
            end else begin
                assign mn_sn_resp_or[i]  = mn_sn_resp[i] | mn_sn_resp_or[i-1];
            end
        end
    endgenerate

    assign mn_resp = mn_sn_resp_or[DEC_NUM-1];

    // sn_tid
    generate
        for(genvar i = 0; i < DEC_NUM; i = i + 1) begin : SN_TID_GEN         
                assign sn_tid[i] = route_tid_t[i];
        end
    endgenerate

endmodule
