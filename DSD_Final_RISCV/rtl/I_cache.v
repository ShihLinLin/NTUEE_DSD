module I_cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================

    reg  [127:0] mem_rdata_r;
    wire [127:0] mem_rdata_w;
    reg         mem_ready_r;

    reg         valid_r_0 [0:15], valid_w_0 [0:15];
    reg         valid_r_1 [0:15], valid_w_1 [0:15];
    reg  [23:0] tag_r_0   [0:15], tag_w_0   [0:15];
    reg  [23:0] tag_r_1   [0:15], tag_w_1   [0:15];
    reg [127:0] data_r_0  [0:15], data_w_0  [0:15];
    reg [127:0] data_r_1  [0:15], data_w_1  [0:15];

    wire  [1:0] in_word_sel;   // cache 裡面第幾個 word
    wire  [3:0] in_block_sel;  // cache 裡面第幾個 block
    wire [23:0] in_tag;

    reg          mem_read_r, mem_read_w, mem_write_r, mem_write_w;
    reg   [27:0] mem_addr_r;
    wire  [27:0] mem_addr_w;
    reg  [127:0] mem_wdata_r;
    wire [127:0] mem_wdata_w;
    
    reg stall_r, stall_w;
    wire hit_0, hit_1;

    integer i;

//==== combinational circuit ==============================

    assign proc_stall = stall_w; 
    assign proc_rdata = mem_ready_r ? mem_rdata_r[in_word_sel*32 +: 32] : 
                            hit_0 ? 
                                data_r_0[in_block_sel][in_word_sel*32 +: 32] : 
                                data_r_1[in_block_sel][in_word_sel*32 +: 32];
    assign mem_rdata_w = mem_rdata;
                            
    assign mem_read  = mem_read_r && !mem_ready_r;
    assign mem_write = mem_write_r;
    assign mem_addr  = mem_addr_r;
    assign mem_wdata = mem_wdata_r;

    assign mem_addr_w = {in_tag, in_block_sel};   // new one
    assign mem_wdata_w = data_w_0[in_block_sel];

    assign in_word_sel  = proc_addr[ 1:0];
    assign in_block_sel = proc_addr[ 5:2];
    assign in_tag       = proc_addr[29:6];

    assign hit_0 = tag_r_0[in_block_sel] == in_tag && valid_r_0[in_block_sel];
    assign hit_1 = tag_r_1[in_block_sel] == in_tag && valid_r_1[in_block_sel];

//==== sequential circuit =================================

    // stall
    always@(*) begin
        stall_w = 1'b0;
        if (!mem_ready_r && proc_read ) begin
            if (!(hit_0 || hit_1)) begin
                stall_w = 1'b1;
            end
        end
    end
    
    // cache data & processor read data & read/write control
    always@(*) begin
        mem_read_w   = 1'b0;
        mem_write_w  = 1'b0;
        for (i = 0; i < 16; i = i + 1) begin
            valid_w_0[i] = valid_r_0[i];
            tag_w_0  [i] = tag_r_0  [i];
            data_w_0 [i] = data_r_0 [i];
            valid_w_1[i] = valid_r_1[i];
            tag_w_1  [i] = tag_r_1  [i];
            data_w_1 [i] = data_r_1 [i];
        end
        if (stall_r) begin
            if (mem_ready_r) begin
                    valid_w_1[in_block_sel] = 1'b1;
                    tag_w_1  [in_block_sel] = in_tag;
                    data_w_1 [in_block_sel] = mem_rdata_r;
                    valid_w_0[in_block_sel] = valid_r_1[in_block_sel];
                    tag_w_0  [in_block_sel] = tag_r_1  [in_block_sel];
                    data_w_0 [in_block_sel] = data_r_1 [in_block_sel];
                  
            end
            else begin 
                    mem_read_w = 1'b1;
        
            end
        end
     
        else if (!hit_0 && !hit_1) begin
                if (proc_read)
                    mem_read_w = 1'b1;
        end
    end

    always@( posedge clk ) begin
        if( proc_reset ) begin
            stall_r <= 1'b0;
            mem_read_r <= 1'b0;
            mem_write_r <= 1'b0;
            mem_addr_r <= 28'b0;
            mem_wdata_r <= 128'b0;
            mem_ready_r <= 1'b0;
            mem_rdata_r <= 127'b0;
            for (i = 0; i < 16; i = i + 1) begin
                valid_r_0[i] <= 1'b0;
                tag_r_0  [i] <= 26'b0;
                data_r_0 [i] <= 128'b0;
                valid_r_1[i] <= 1'b0;
                tag_r_1  [i] <= 26'b0;
                data_r_1 [i] <= 128'b0;
            end
        end
        else begin
            stall_r <= stall_w;
            mem_read_r <= mem_read_w;
            mem_write_r <= mem_write_w;
            mem_addr_r <= mem_addr_w;
            mem_wdata_r <= mem_wdata_w;
            mem_ready_r <= mem_ready;
            mem_rdata_r <= mem_rdata_w;
            for (i = 0; i < 16; i = i + 1) begin
                valid_r_0[i] <= valid_w_0[i];
                tag_r_0  [i] <= tag_w_0  [i];
                data_r_0 [i] <= data_w_0 [i];
                valid_r_1[i] <= valid_w_1[i];
                tag_r_1  [i] <= tag_w_1  [i];
                data_r_1 [i] <= data_w_1 [i];
            end
        end
    end

endmodule
