`timescale 1ns/10ps
module GSIM ( clk, reset, in_en, b_in, out_valid, x_out);
input   clk ;
input   reset ;
input   in_en;
output  out_valid;
input   [15:0]  b_in;
output  [31:0]  x_out;


reg [15:0] b_r [0:15];
reg [15:0] b_w [0:15];

reg [31:0] x_r [0:15];
reg [31:0] x_w [0:15];

reg signed [31:0] x_m3, x_m2, x_m1, x_1, x_2, x_3; 

integer i;
reg [3:0] count_r, count_w;
reg [3:0] count_r2, count_w2;
reg flip_r, flip_w;
// reg [6:0] count_big_r, count_big_w;
reg half_conv_r, half_conv_w;
reg conv_r, conv_w;
reg slow_mode_r, slow_mode_w;
reg out_valid_r, out_valid_w;
wire signed [31:0] x_cal;

assign out_valid = out_valid_r;
assign x_out = x_r[15];

// in_en reg
reg in_en_r;

// count
always @(*) begin
    if (slow_mode_r) begin
        count_w = count_r + flip_r;
    end
    else begin
        count_w = count_r + 1;
    end
end

// count
always @(*) begin
    if (slow_mode_r && !flip_r) begin
        count_w2 = count_r2;
    end
    else begin
        count_w2 = count_r;
    end
end

always @(*) begin
    flip_w = !flip_r;
end

// count_big
// always @(*) begin
//     if (count_r == 4'b1111) begin
//         count_big_w = count_big_r + 1;
//     end
//     else begin
//         count_big_w = count_big_r;
//     end
// end

// b
always @(*) begin
    if (slow_mode_r && !flip_r && !out_valid_r) begin
        // stay still
        for (i=0; i<16; i=i+1)
            b_w[i] = b_r[i];
    end
    else begin
        // shift one
        for (i=0; i<15; i=i+1)
            b_w[i] = b_r[i+1];
        b_w[15] = b_r[0];
    end
    if (in_en) begin
        b_w[15] = b_in;
    end
end

// x
always @(*) begin
    if (slow_mode_r && !flip_r && !out_valid_r) begin
        // stay still
        for (i=0; i<16; i=i+1)
            x_w[i] = x_r[i];
    end
    else begin
        // shift one
        for (i=0; i<15; i=i+1)
            x_w[i] = x_r[i+1];
        x_w[15] = x_r[0];
    end
    if (in_en) begin
        case(count_r)
            4'd0, 4'd15: x_w[15] = {{4{b_in[15]}}, b_in, 12'b0};
            4'd1, 4'd14: x_w[15] = {{7{b_in[15]}}, b_in,  9'b0};
            4'd2, 4'd13: x_w[15] = {{7{b_in[15]}}, b_in,  9'b0};
            4'd3, 4'd12: x_w[15] = {{7{b_in[15]}}, b_in,  9'b0};
            4'd4, 4'd11: x_w[15] = {{4{b_in[15]}}, b_in, 12'b0};
            4'd5, 4'd10: x_w[15] = {{4{b_in[15]}}, b_in, 12'b0};
            4'd6, 4'd9:  x_w[15] = {{4{b_in[15]}}, b_in, 12'b0};
            4'd7, 4'd8:  x_w[15] = {{4{b_in[15]}}, b_in, 12'b0};
        endcase
    end
    else if (!in_en_r && !slow_mode_r) begin
        x_w[14] = x_cal;
    end
    else if (slow_mode_r && flip_r && !out_valid_r)begin
        x_w[15] = x_cal;
    end
end

// output for calculation

always @(*) begin
    x_m3 = x_r[13];
    x_m2 = x_r[14];
    x_m1 = x_r[15];
    x_1  = x_r[1];
    x_2  = x_r[2];
    x_3  = x_r[3];
    if (count_r == 0) begin
        x_m3 = 0;
        x_m2 = 0;
        x_m1 = 0;
    end
    else if (count_r == 1) begin
        x_m3 = 0;
        x_m2 = 0;
    end
    else if (count_r == 2) begin
        x_m3 = 0;
    end
    else if (count_r == 13) begin
        x_3 = 0;
    end
    else if (count_r == 14) begin
        x_3 = 0;
        x_2 = 0;
    end
    else if (count_r == 15) begin
        x_3 = 0;
        x_2 = 0;
        x_1 = 0;
    end
end

// calculation
reg signed [15:0] b_cal;
reg signed [37:0] b_in_ext;
reg signed [32:0] x_1_sum;
reg signed [32:0] x_2_sum;
reg signed [37:0] x_3_sum;
reg signed [37:0] prod_sum_r;
reg signed [37:0] prod_sum;
reg signed [37:0] div_result;
reg signed [37:0] result;

always @(*) begin
    b_in_ext = b_cal <<< 16;  
    b_cal = b_r[0];
    x_1_sum = x_m1 + x_1;              
    x_2_sum = x_m2 + x_2;                  
    x_3_sum = x_m3 + x_3;                          
    prod_sum_r = b_in_ext + x_3_sum + (x_2_sum * 4'sb1010) + (x_1_sum * 5'sb01101);             
end

assign x_cal = result >>> 6;
always @(*) begin
    div_result = prod_sum * 3;  
    result = div_result + (div_result >>> 4) + (div_result >>> 8) + (div_result >>> 12) + (div_result >>> 16) + (div_result >>> 20) + (div_result >>> 24) + (div_result >>> 28) + (div_result >>> 32);  
end

// convs
always @(*) begin
    if (count_r2 == 15 && !in_en_r)
        half_conv_w = 1;
    else if (x_w[14][31:8] != x_r[15][31:8])
        half_conv_w = 0;
    else
        half_conv_w = half_conv_r;  
end
always @(*) begin
    if (count_r2 == 15 && slow_mode_r && flip_r)
        conv_w = 1;
    else if (x_w[15] != x_r[0] && flip_r)
        conv_w = 0;
    else
        conv_w = conv_r;
end

always @(*) begin
    slow_mode_w = slow_mode_r;
    if (half_conv_r == 1 && count_r2 == 15 && x_w[14][31:8] == x_r[15][31:8]) begin
        slow_mode_w = 1;
    end
end

// output valid
always @(*) begin
    out_valid_w = out_valid_r;
    if (conv_r == 1 && count_r2 == 15 && x_w[14] == x_r[15]) begin
        out_valid_w = 1;
    end
end

always @(posedge clk or posedge reset) begin
    if (reset) begin
        in_en_r     <= 0;
        count_r     <= 0;
        count_r2    <= 0;
        flip_r      <= 1;
        half_conv_r <= 0;
        conv_r      <= 0;
        slow_mode_r <= 0;
        out_valid_r <= 0;
        prod_sum    <= 0;
        for (i = 0; i < 16; i = i + 1) begin
            b_r[i] <= 0;
            x_r[i] <= 0;
        end
    end else begin
        in_en_r     <= in_en;
        count_r     <= count_w;
        count_r2    <= count_w2;
        flip_r      <= flip_w;
        half_conv_r <= half_conv_w;
        conv_r      <= conv_w;
        slow_mode_r <= slow_mode_w;
        out_valid_r <= out_valid_w;
        prod_sum    <= prod_sum_r;
        for (i = 0; i < 16; i = i + 1) begin
            b_r[i] <= b_w[i];
            x_r[i] <= x_w[i];
        end
    end
end


endmodule