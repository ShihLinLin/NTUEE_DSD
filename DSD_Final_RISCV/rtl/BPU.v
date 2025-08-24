module BPU(
    clk,
    rst_n,
    stall,
    B,
    Branch_Exe,
    PreWrong,
    BrPre
);
    //==== I/O ====
    input   clk;
    input   rst_n;
    input   stall;          
    input   B;              // 1 if the instr in IF is a branch
    //input   [1:0] pc_bits;  // Use 2 bit pc to xor GHR
    input   Branch_Exe;      // branch taken or not in Exe
    input   PreWrong;       // prove prediction wrong in Exe
    output reg BrPre;          // predict whether to branch in IF

    //==== Reg and Wire =====
    reg  [1:0] GHR_r, GHR_w;                   // Global History Register, 2-bit
    reg  [1:0] HT_r [0:3], HT_w [0:3];       // Pattern History Table, 4 entry * 2 bit state
    wire [1:0] HT_index;                // PHT index
    wire [1:0] cur_state;
    integer i;

    //assign index = pc_bits ^ history_r;
    assign HT_index = GHR_r[1:0];
    assign cur_state = HT_r[HT_index];

    //==== Parameters =====
    parameter weak_not_taken = 2'b00;
    parameter strong_not_taken = 2'b01;
    parameter weak_taken = 2'b10;
    parameter strong_taken = 2'b11;

    //==== Combinational ====
    always@(*) begin
        BrPre = 1'b0;
        if (B) begin
            case(cur_state)
                weak_not_taken:    BrPre = 1'b0;
                strong_not_taken:  BrPre = 1'b0;
                weak_taken:        BrPre = 1'b1;
                strong_taken:      BrPre = 1'b1;
            endcase
        end
    end

    //==== FSM ====
    always @(*) begin
        // default
        for (i = 0; i < 4; i = i + 1) begin
            HT_w[i] = HT_r[i];
        end

        GHR_w = GHR_r;

        if (PreWrong) begin
            // Update GHR
            GHR_w = {GHR_r[0], Branch_Exe};
            // Update FSM
            case (HT_r[HT_index])
                weak_not_taken:   HT_w[HT_index] = (Branch_Exe) ? weak_taken : strong_not_taken;
                strong_not_taken: HT_w[HT_index] = (Branch_Exe) ? weak_not_taken : strong_not_taken;
                weak_taken:       HT_w[HT_index] = (Branch_Exe) ? strong_taken : weak_not_taken;
                strong_taken:     HT_w[HT_index] = (Branch_Exe) ? strong_taken : weak_taken;
            endcase
        end
    end

    //==== Sequential ====
    always @(posedge clk) begin
        if (~rst_n) begin
            GHR_r <= 2'b00;
            for (i = 0; i < 4; i = i + 1) begin
                HT_r[i] <= weak_not_taken;  // initial value
            end
        end
        else if (~stall) begin
            GHR_r <= GHR_w;
            for (i = 0; i < 4; i = i + 1) begin
                HT_r[i] <= HT_w[i];
            end
        end
    end

endmodule











  






