`include "dadda.v"

module RISCV_Pipeline(
    clk,
    rst_n,
    // for mem_I
    ICACHE_ren,
    ICACHE_wen,
    ICACHE_addr,
    ICACHE_wdata,
    ICACHE_stall,
    ICACHE_rdata,
    // for mem_D
    DCACHE_ren,
    DCACHE_wen,
    DCACHE_addr,
    DCACHE_wdata,
    DCACHE_stall,
    DCACHE_rdata,
    // for TestBed
    PC
);

    input         clk, rst_n ;
    // for mem_I
    output        ICACHE_ren  ;
    output        ICACHE_wen  ;
    output [29:0] ICACHE_addr ;
    output [31:0] ICACHE_wdata;
    input         ICACHE_stall;
    input  [31:0] ICACHE_rdata;
    // for mem_D
    output        DCACHE_ren  ;
    output        DCACHE_wen  ;
    output [29:0] DCACHE_addr ;
    output [31:0] DCACHE_wdata;
    input         DCACHE_stall;
    input  [31:0] DCACHE_rdata;
    // for TestBed
    output [31:0] PC;

    //==== wire/reg definition ================================

    // IF stage
    reg ren_i_r, ren_i_w;
    reg reset_r;
    reg [31:0] pc_r, pc_w;
    reg [31:0] r_w [1:31];
    reg [31:0] r_r [1:31];
    wire stall;
    reg  [31:0] pc_If2Dec_r, pc_If2Dec_w;
    
    reg  [31:0] instr_r, instr_w;
    wire  [1:0] op_cpr;
    reg         half_instr_r, half_instr_w;

    wire Branch_If;
    reg  BrPre_If2Dec_r, BrPre_If2Dec_w;
    wire BrPre_out;
    // wire [31:0] pc_PreWrong1;
    // wire [31:0] pc_PreWrong2;

    // ID stage
    wire  [6:0] opcode_norm, opcode;
    wire  [1:0] opcode_cmp;
    wire        cmp;
    wire  [2:0] func3, func3_cmp;
    wire        cmp_rs;
    wire  [4:0] rs1_norm,  rs2_norm,  rd_norm;
    wire  [2:0] rs1_prime, rs2_prime, rd_prime;
    wire  [4:0] rs1,       rs2,       rd;

    wire signed [11:0] imm_i_type_norm;
    wire signed [11:0] imm_s_type;
    wire signed [20:0] imm_jal_norm;
    wire signed [11:0] imm_j_cmp;
    wire signed  [8:0] imm_b_cmp;
    wire signed [12:0] imm_beq;
    wire         [6:0] imm_cmp_00;
    wire signed  [5:0] imm_i_type_cmp;
    wire signed [11:0] imm_lw;
    wire signed [11:0] imm_sw;
    wire signed [11:0] imm_i_type;

    wire [31:0] rs1_data, rs2_data;
    // reg   [4:0] rs1_Dec2Exe_r, rs1_Dec2Exe_w;
    // reg   [4:0] rs2_Dec2Exe_r, rs2_Dec2Exe_w;
    reg  [31:0] pc_Dec2Exe_r, pc_Dec2Exe_w;
    wire        for1_from_EXE, for1_from_MEM, for2_from_EXE, for2_from_MEM;
    wire [31:0] rs1_data_for, rs2_data_for;

    reg BrPre_Dec2Exe_r, BrPre_Dec2Exe_w;

    // EXE stage
    reg  [31:0] A_r, A_w, B_r, B_w;
    reg   [4:0] rd_Dec2Exe_r, rd_Dec2Exe_w;
    reg   [2:0] ALU_op_r, ALU_op_w;  // we follow func3, and 011 for SUB
    //reg         branch_r, branch_w;
    //reg         jump_r, jump_w;
    reg         read_Dec2Exe_r,  read_Dec2Exe_w;
    reg         write_Dec2Exe_r, write_Dec2Exe_w;
    reg  [31:0] sw_value_Dec2Exe_r, sw_value_Dec2Exe_w;
    reg         SRA_r, SRA_w;
    reg         Mul_r, Mul_w;

    // forwarding unit
    // reg [1:0] forwardA, forwardB;
    reg [31:0] ALU_A, ALU_B;
    // branch
    reg branch_Dec2Exe_r, branch_Dec2Exe_w;
    reg bne_r, bne_w;
    reg  [31:0] pc_branch_Dec2Exe_r, pc_branch_Dec2Exe_w;
    // jump
    reg jal_Dec2Exe_r, jal_Dec2Exe_w;
    reg jalr_Dec2Exe_r, jalr_Dec2Exe_w;
    reg cmp_Dec2Exe_w, cmp_Dec2Exe_r;
    wire [31:0] C_mul;
    reg  [31:0] C_r, C_w;
    reg   [4:0] rd_Exe2Mem_r, rd_Exe2Mem_w;
    reg         read_Exe2Mem_r, read_Exe2Mem_w;
    reg         write_Exe2Mem_r, write_Exe2Mem_w;
    reg  [31:0] sw_value_Exe2Mem_r, sw_value_Exe2Mem_w;
    reg  [31:0] pc_Exe2Mem_r, pc_Exe2Mem_w;

    wire Branch_Exe;
    wire PreWrong;

    // MEM stage
    reg         DCACHE_ren, DCACHE_wen;
    reg   [4:0] rd_Mem2WB_r, rd_Mem2WB_w;
    reg  [31:0] rd_value_r, rd_value_w;
    // branch
    // reg branch_Exe2Mem_r, branch_Exe2Mem_w;
    reg [31:0] pc_branch_Exe2Mem_r, pc_branch_Exe2Mem_w;
    // wire branch_taken;
    // jump
    reg jal_Exe2Mem_r, jal_Exe2Mem_w;
    reg jalr_Exe2Mem_r, jalr_Exe2Mem_w;
    reg cmp_Exe2Mem_r, cmp_Exe2Mem_w;
    // reg jal_Mem2WB_r, jal_Mem2WB_w;
    // reg jalr_Mem2WB_r, jalr_Mem2WB_w;

    // reg  [31:0] pc_Mem2WB_r, pc_Mem2WB_w;
    

    integer i;

    // stall
    reg lw_hazard_r, lw_hazard_w;

    // jump
    wire jal_taken;
    wire jalr_taken;





    //==== local parameter definition =====================

    localparam opcode_r_type = 7'b0110011; // ADD  SUB AND  OR  XOR                 SLT  | MUL
    localparam opcode_i_type = 7'b0010011; // ADDI     ANDI ORI XORI SLLI SRAI SRLI SLTI
    localparam opcode_b_type = 7'b1100011; // BEQ BNE
    localparam opcode_jal    = 7'b1101111; // JAL
    localparam opcode_jalr   = 7'b1100111; // JALR
    localparam opcode_lw     = 7'b0000011; // LW
    localparam opcode_sw     = 7'b0100011; // SW

    localparam func3_add     = 3'b000;     // ADD SUB
    localparam func3_and     = 3'b111;     // AND
    localparam func3_or      = 3'b110;     // OR
    localparam func3_xor     = 3'b100;     // XOR
    localparam func3_sll     = 3'b001;     // SLL
    localparam func3_slt     = 3'b010;     // SLT
    localparam func3_sr      = 3'b101;     // SRL SRA

    localparam opcode_c_00   = 2'b00;      // LW   SW
    localparam opcode_c_01   = 2'b01;      // ADDI ANDI SRLI SRAI BEQZ BNEZ J  JAL
    localparam opcode_c_10   = 2'b10;      // ADD  MV   SLLI                JR JALR

    localparam func3_lw      = 3'b010;     // LW
    localparam func3_sw      = 3'b110;     // SW

    localparam func3_addi    = 3'b000;     // ADDI
    localparam func3_andi    = 3'b100;     // ANDI SRLI SRAI
    localparam func3_beqz    = 3'b110;     // BEQZ
    localparam func3_bnez    = 3'b111;     // BNEZ
    localparam func3_j       = 3'b101;     // J
    localparam func3_jal     = 3'b001;     // JAL

    localparam func4_add     = 4'b1001;    // ADD JALR
    localparam func4_mv      = 4'b1000;    // MV  JR
    localparam func3_slli    = 3'b100;     // SLLI

    localparam func2_andi    = 2'b10;      // ANDI
    localparam func2_srli    = 2'b00;      // SRLI
    localparam func2_srai    = 2'b01;      // SRAI

    //==== combinational circuit ==============================

    BPU bpu(
        .clk            ( clk ),
        .rst_n          ( rst_n ),
        .stall          ( stall ),
        .B              ( Branch_If ),
        .Branch_Exe     ( Branch_Exe ),
        .PreWrong       ( PreWrong ),
        .BrPre          ( BrPre_out )
    );

    assign PC = pc_r;
    assign stall = ICACHE_stall || DCACHE_stall;

    // Stage 1: Instruction Fetch
    // wire [31:0] C_r_jalr;
    // assign C_r_jalr = C_r & ~32'b1;
    assign Branch_If  = !half_instr_w && ((op_cpr == 2'b11) ? (ICACHE_rdata[30] & (~ICACHE_rdata[26])) /* x"1"100"0"11 */ : (instr_w[24] & instr_w[22]));
    //assign pc_PreWrong1 = pc_Dec2Exe_r; // if "PreWrong = 1: BrPre = 1 but Branch_Exe = 0" in EXE, assume "pc_r = beq" in IF, "pc_r = beq + 4" in next ID
    //assign pc_PreWrong2 = pc_branch_Dec2Exe_r - 3'd4; // if "PreWrong = 1: BrPre = 0 but Branch_Exe = 1" in EXE, assume "pc_r = branch target" in next ID

    assign ICACHE_addr = // (jal_taken || jalr_taken) ? C_r[31:2] :
                        // (jalr_taken)              ? C_r_jalr[31:2] :
                        // (PreWrong & !Branch_Exe)  ? pc_PreWrong1[31:2] : // PreWrong = 1: BrPre = 1 but Branch_Exe = 0
                        // (PreWrong & Branch_Exe)   ? pc_PreWrong2[31:2] : // PreWrong = 1: BrPre = 0 but Branch_Exe = 1
                                                       pc_r[31:2];
    assign ICACHE_ren = ren_i_r;
    assign ICACHE_wen = 1'b0;
    assign op_cpr = pc_r[1] ? ICACHE_rdata[9:8] : ICACHE_rdata[25:24];

    always@(*) begin
        BrPre_If2Dec_w = BrPre_out;
        pc_w = pc_r + 3'd4;
        ren_i_w = (reset_r & rst_n) ? 1'b1 : ren_i_r;
        pc_If2Dec_w = pc_r;
        
        if (reset_r) begin
            pc_w = pc_r;
        end
        else if (stall || lw_hazard_w) begin
            pc_w = pc_r;
            pc_If2Dec_w = pc_If2Dec_r;
            BrPre_If2Dec_w = BrPre_If2Dec_r;
        end
        else if (PreWrong) begin
            pc_If2Dec_w = 32'b0;
            if (!Branch_Exe) begin // BrPre = 1, Branch_Exe = 0
                pc_w = pc_Dec2Exe_r + (cmp_Dec2Exe_r ? 3'd2 : 3'd4);
            end else begin // BrPre = 0, Branch_Exe = 1
                pc_w = pc_branch_Dec2Exe_r;
            end
        end
        else if (jal_taken || jalr_taken) begin
            pc_w = C_w; // + 3'd4;
            // pc_If2Dec_w = C_w;
        end
        else if (BrPre_If2Dec_w) begin
            pc_w = $signed(pc_r) + ((op_cpr == 2'b11) ? 
                                    $signed({ICACHE_rdata[31], ICACHE_rdata[6:1], ICACHE_rdata[19:16], 1'b0}) : 
                                    $signed({instr_w[20], instr_w[30:29], instr_w[26], instr_w[19:18], instr_w[28:27], 1'b0}));
            pc_If2Dec_w = pc_r;
        end
        else if (half_instr_r) begin
            pc_w = pc_r + 2'd2;
            pc_If2Dec_w = pc_r - 2'd2;
        end
        else if (op_cpr != 2'b11 || pc_r[1]) begin // compressed
            pc_w = pc_r + 2'd2;
        end
    end

    always@(*) begin
        instr_w = ICACHE_rdata; // normal
        half_instr_w = 1'b0;
        if (stall || lw_hazard_w) begin
            instr_w = instr_r;
            half_instr_w = half_instr_r;
        end
        else if (PreWrong || jal_taken || jalr_taken) begin
            instr_w = 32'b0; //ICACHE_rdata;
            // half_instr_w = 1'b0;
        end
        else if (half_instr_r) begin
            half_instr_w = 1'b0;
            instr_w = {instr_r[31:16], ICACHE_rdata[31:16]};
        end
        else if (pc_r[1]) begin // higher 2 words
            if (op_cpr == 2'b11) begin
                half_instr_w = 1'b1;
            end
            instr_w = {ICACHE_rdata[15:0], 16'b0}; // higher
        end
    end

    // Stage 2: Registers Fetch & Instruction Decode

    assign opcode_norm = instr_r[30:24];
    assign opcode_cmp  = instr_r[25:24];
    assign cmp = (opcode_cmp != 2'b11);
    assign opcode = cmp ? {func3_cmp, opcode_cmp} : opcode_norm;

    assign func3     = instr_r[22:20];
    assign func3_cmp = instr_r[23:21];
    assign cmp_rs = ((opcode_cmp == 2'b00) /*&& instr_r[30] to exclude all-zero case*/) || 
                    ((opcode_cmp == 2'b01) && (func3_cmp != 3'b000));

    
    assign rs1_norm  = cmp ? rd_norm        : {instr_r[11: 8], instr_r[23]};
    assign rs2_norm  = cmp ? instr_r[30:26] : {instr_r[0],  instr_r[15:12]};
    assign rd_norm   =                        {instr_r[19:16], instr_r[31]};
    assign rs1_prime = {instr_r[17:16], instr_r[31]}; 
    assign rs2_prime =  instr_r[28:26];
    assign rd_prime  = instr_r[24] /*01*/ ? {instr_r[17:16], instr_r[31]} : /*lw*/ instr_r[28:26];
    assign rs1 = (cmp_rs) ? {2'b01, rs1_prime} : rs1_norm;
    assign rs2 = (cmp_rs) ? {2'b01, rs2_prime} : rs2_norm;
    assign rd  = (cmp_rs) ? {2'b01, rd_prime } : rd_norm;

    assign rs1_data = (rs1 == 5'b0) ? 32'b0 : r_w[rs1];
    assign rs2_data = (rs2 == 5'b0) ? 32'b0 : r_w[rs2];

    assign for1_from_EXE = (rd_Exe2Mem_w != 5'd0 && rd_Exe2Mem_w == rs1);
    assign for1_from_MEM = (rd_Mem2WB_w  != 5'd0 && rd_Mem2WB_w  == rs1);
    assign rs1_data_for = for1_from_EXE ? C_w : (for1_from_MEM ? rd_value_w : rs1_data);

    assign for2_from_EXE = (rd_Exe2Mem_w != 5'd0 && rd_Exe2Mem_w == rs2);
    assign for2_from_MEM = (rd_Mem2WB_w  != 5'd0 && rd_Mem2WB_w  == rs2);
    assign rs2_data_for = for2_from_EXE ? C_w : (for2_from_MEM ? rd_value_w : rs2_data);

    // Imms
    assign imm_s_type = {instr_r[7:1], instr_r[19:16], instr_r[31]};
    
    assign imm_cmp_00 = {instr_r[29], instr_r[20:18], instr_r[30], 2'b00};
    assign imm_lw = instr_r[24] ? imm_i_type_norm : imm_cmp_00;
    assign imm_sw = instr_r[24] ? imm_s_type : imm_cmp_00;

    assign imm_i_type_norm = {instr_r[7:0], instr_r[15:12]};
    assign imm_i_type_cmp = {instr_r[20], instr_r[30:26]};
    assign imm_i_type = cmp ? {{6{(instr_r[24] && !instr_r[23]) /*C.ADDI*/ ? imm_i_type_cmp[5] : 1'b0}}, imm_i_type_cmp}
                            : imm_i_type_norm;

    assign imm_jal_norm = {instr_r[7], instr_r[11:8], instr_r[23:20], instr_r[12], instr_r[6:0], instr_r[15:13], 1'b0};
    assign imm_j_cmp = {instr_r[20], instr_r[16], instr_r[18:17], instr_r[30], instr_r[31], instr_r[26], instr_r[19], instr_r[29:27], 1'b0};
    assign imm_beq = {instr_r[7], instr_r[31], instr_r[6:1], instr_r[19:16], 1'b0};
    assign imm_b_cmp = {instr_r[20], instr_r[30:29], instr_r[26], instr_r[19:18], instr_r[28:27], 1'b0};

    always@(*) begin
        // passby
        pc_Dec2Exe_w = pc_If2Dec_r;
        BrPre_Dec2Exe_w = BrPre_If2Dec_r;
        cmp_Dec2Exe_w = cmp;
        
        Mul_w = 1'b0;
        if (stall) begin
            A_w = A_r;
            B_w = B_r;
            rd_Dec2Exe_w = rd_Dec2Exe_r;
            ALU_op_w = ALU_op_r;
            //branch_w = branch_r;
            branch_Dec2Exe_w = branch_Dec2Exe_r;
            bne_w = bne_r;
            pc_branch_Dec2Exe_w = pc_branch_Dec2Exe_r;
            jal_Dec2Exe_w = jal_Dec2Exe_r;
            jalr_Dec2Exe_w = jalr_Dec2Exe_r;
            cmp_Dec2Exe_w = cmp_Dec2Exe_r;
            read_Dec2Exe_w  = read_Dec2Exe_r;
            write_Dec2Exe_w = write_Dec2Exe_r;
            sw_value_Dec2Exe_w  = sw_value_Dec2Exe_r;
            SRA_w = SRA_r;
            Mul_w = Mul_r;
            pc_Dec2Exe_w = pc_Dec2Exe_r;
            BrPre_Dec2Exe_w = BrPre_Dec2Exe_r;
            // rs1_Dec2Exe_w = rs1_Dec2Exe_r;
            // rs2_Dec2Exe_w = rs2_Dec2Exe_r;
        end
        else if (lw_hazard_w || PreWrong || jalr_taken || jal_taken || half_instr_r) begin
            // nop
            A_w = 32'b0;
            B_w = 32'b0;
            rd_Dec2Exe_w = 5'd0;
            ALU_op_w = 3'b000;
            //branch_w = 1'b0;
            branch_Dec2Exe_w = 1'b0;
            bne_w = 1'b0;
            pc_branch_Dec2Exe_w = 1'b0;
            jal_Dec2Exe_w = 1'b0;
            jalr_Dec2Exe_w = 1'b0;
            // cmp_Dec2Exe_w = 1'b0;
            read_Dec2Exe_w = 1'b0;
            write_Dec2Exe_w = 1'b0;
            sw_value_Dec2Exe_w = 32'b0;
            SRA_w = 1'b0;
            // rs1_Dec2Exe_w = 5'd0;
            // rs2_Dec2Exe_w = 5'd0;
            pc_Dec2Exe_w = 32'b0;
            BrPre_Dec2Exe_w = 1'b0;
        end
        else begin
            // for forwarding
            // rs1_Dec2Exe_w = rs1;
            // rs2_Dec2Exe_w = rs2;

            rd_Dec2Exe_w = 5'b0;
            //branch_w = 1'b0;
            branch_Dec2Exe_w = 1'b0;
            bne_w = (func3 == 3'b001);
            pc_branch_Dec2Exe_w = 32'b0;
            jal_Dec2Exe_w = 1'b0;
            jalr_Dec2Exe_w = 1'b0;
            // cmp_Dec2Exe_w = 1'b0;
            read_Dec2Exe_w = 1'b0;
            write_Dec2Exe_w = 1'b0;
            SRA_w = cmp ? instr_r[18] : instr_r[6];
            sw_value_Dec2Exe_w = rs2_data_for;
            casez(opcode)
            opcode_r_type: begin
                A_w = rs1_data_for;
                B_w = rs2_data_for;
                rd_Dec2Exe_w = rd;
                ALU_op_w = func3;
                if (func3 == func3_add) begin
                    if (instr_r[6] == 1'b1) // SUB
                        ALU_op_w = 3'b011;
                end
                if (instr_r[1]) begin
                    Mul_w = 1'b1;
                end
            end
            opcode_i_type, 7'b???0001, 7'b??0??10: begin
                A_w = rs1_data_for;
                B_w = imm_i_type;
                rd_Dec2Exe_w = rd;
                ALU_op_w = cmp ? {instr_r[23], instr_r[23] && instr_r[19], instr_r[23] || instr_r[25]} : func3;
            end
            opcode_b_type, 7'b??11?01: begin
                A_w = rs1_data_for;
                B_w = cmp ? 32'b0 : rs2_data_for;
                ALU_op_w = 3'b011; // SUB
                //branch_w = 1'b1;
                branch_Dec2Exe_w = 1'b1;
                bne_w = cmp ? instr_r[21] : (func3 == 3'b001);
                rd_Dec2Exe_w = 5'd0; // prevent writing back
                pc_branch_Dec2Exe_w = $signed(pc_If2Dec_r) + (cmp ? imm_b_cmp : imm_beq);
            end
            opcode_jal, 7'b???0101: begin
                A_w = pc_If2Dec_r;
                B_w = cmp ? imm_j_cmp : imm_jal_norm;
                rd_Dec2Exe_w = cmp ? (instr_r[23] ? 5'd0 : 5'd1) : rd; // rd = 0 for J
                ALU_op_w = func3_add;
                //jump_w = 1'b1;
                jal_Dec2Exe_w = 1'b1;
                // cmp_Dec2Exe_w = cmp;
            end
            opcode_jalr: begin
                A_w = rs1_data_for;
                B_w = imm_i_type_norm;
                rd_Dec2Exe_w = rd;
                ALU_op_w = func3_add;
                //jump_w = 1'b1;
                jalr_Dec2Exe_w = 1'b1;
            end
            opcode_lw, 7'b??01000: begin
                A_w = rs1_data_for;
                B_w = imm_lw;
                rd_Dec2Exe_w = rd;
                ALU_op_w = func3_add;
                read_Dec2Exe_w = 1'b1;
            end
            opcode_sw, 7'b??11000: begin
                A_w = rs1_data_for;
                B_w = imm_sw;
                ALU_op_w = func3_add;
                write_Dec2Exe_w = 1'b1;
            end
            7'b??1??10: begin
                if (rs2 == 5'b0) begin  // JR & JALR
                    A_w = rs1_data_for;
                    B_w = 32'b0;
                    rd_Dec2Exe_w = instr_r[20] /*JALR*/ ? 5'd1 : 5'd0;
                    ALU_op_w = func3_add;
                    jalr_Dec2Exe_w = 1'b1;
                    // cmp_Dec2Exe_w = cmp;
                end
                else begin  // MV & ADD
                    A_w = instr_r[20] /*ADD*/ ? rs1_data_for : 32'b0;
                    B_w = rs2_data_for;
                    rd_Dec2Exe_w = rd;
                    ALU_op_w = func3_add;
                end
                
            end
            default: begin
                A_w = rs1_data_for;
                B_w = imm_s_type;
                ALU_op_w = func3_add;
            end
            endcase
        end
    end

    // Stage 3: Execute

    dadda dadda1 (
		.A(ALU_A),
		.B(ALU_B),
		.C(C_mul)
	);

    assign jal_taken = jal_Dec2Exe_r;
    assign jalr_taken = jalr_Dec2Exe_r;

    assign Branch_Exe = (branch_Dec2Exe_r && (
                        (!bne_r && (ALU_A == ALU_B)) || //beq
                        ( bne_r && (ALU_A != ALU_B)))); //bne
    assign PreWrong = (Branch_Exe != BrPre_Dec2Exe_r) && branch_Dec2Exe_r;

    always@(*) begin
        // passby
        sw_value_Exe2Mem_w  = sw_value_Dec2Exe_r;
        ALU_A = A_r;
        ALU_B = B_r;
        if (stall) begin
            C_w = C_r;
            rd_Exe2Mem_w = rd_Exe2Mem_r;
            read_Exe2Mem_w  = read_Exe2Mem_r;
            write_Exe2Mem_w = write_Exe2Mem_r;
            sw_value_Exe2Mem_w  = sw_value_Exe2Mem_r;
            // branch_Exe2Mem_w = branch_Exe2Mem_r;
            pc_branch_Exe2Mem_w = pc_branch_Exe2Mem_r;
            jal_Exe2Mem_w = jal_Exe2Mem_r;
            jalr_Exe2Mem_w = jalr_Exe2Mem_r;
            cmp_Exe2Mem_w = cmp_Exe2Mem_r;
            pc_Exe2Mem_w = pc_Exe2Mem_r;
            lw_hazard_w = lw_hazard_r;
            
        /* end else if (branch_taken) begin // || jalr_taken || jal_taken) begin
            C_w = 32'b0;
            rd_Exe2Mem_w = 5'd0;
            read_Exe2Mem_w  = 1'b0;
            write_Exe2Mem_w = 1'b0;
            sw_value_Exe2Mem_w  = 32'b0;
            branch_Exe2Mem_w = 1'b0;
            pc_branch_Exe2Mem_w = 32'b0;
            jal_Exe2Mem_w = 1'b0;
            jalr_Exe2Mem_w = 1'b0;
            pc_Exe2Mem_w = 32'b0;
            lw_hazard_w = 1'b0; */
        end
        else begin // ALU operation
            // forwardA
            // if (rd_Exe2Mem_r != 5'd0 && rd_Exe2Mem_r == rs1_Dec2Exe_r)
            //     forwardA = 2'b10;
            // else if (rd_Mem2WB_r != 5'd0 &&
            //         !(rd_Exe2Mem_r != 5'd0 && rd_Exe2Mem_r == rs1_Dec2Exe_r) &&
            //         rd_Mem2WB_r == rs1_Dec2Exe_r)
            //     forwardA = 2'b01;
            // else
            //     forwardA = 2'b00;

            // forwardB
            // if (rd_Exe2Mem_r != 5'd0 && rd_Exe2Mem_r == rs2_Dec2Exe_r)
            //     forwardB = 2'b10;
            // else if (rd_Mem2WB_r != 5'd0 &&
            //         !(rd_Exe2Mem_r != 5'd0 && rd_Exe2Mem_r == rs2_Dec2Exe_r) &&  // This line might be redundant since it is a else if
            //         rd_Mem2WB_r == rs2_Dec2Exe_r)
            //     forwardB = 2'b01;
            // else
            //     forwardB = 2'b00;
            
            // ALU input
            // case (forwardA)
            //     2'b00: ALU_A = A_r;          // register file
            //     2'b10: ALU_A = rd_value_w;   // EX/MEM forward
            //     2'b01: ALU_A = rd_value_r;   // MEM/WB forward
            //     default: ALU_A = A_r;
            // endcase

            // case (forwardB)
            //     2'b00: ALU_B = B_r;
            //     2'b10: ALU_B = rd_value_w;
            //     2'b01: ALU_B = rd_value_r;
            //     default: ALU_B = B_r;
            // endcase

            // sw_value
            // if (write_Dec2Exe_r) begin
            //     case (forwardB)
            //         2'b00: sw_value_Exe2Mem_w = sw_value_Dec2Exe_r;
            //         2'b10: sw_value_Exe2Mem_w = rd_value_w;
            //         2'b01: sw_value_Exe2Mem_w = rd_value_r;
            //         default: sw_value_Exe2Mem_w = sw_value_Dec2Exe_r;
            //     endcase
            // end

            if (Mul_r) begin
                C_w = C_mul;
            end
            else begin
                case(ALU_op_r)
                func3_add: begin
                    C_w = ALU_A + ALU_B;
                end
                3'b011: begin
                    C_w = ALU_A - ALU_B;
                end
                func3_and: begin
                    C_w = ALU_A & ALU_B;
                end
                func3_or: begin
                    C_w = ALU_A | ALU_B;
                end
                func3_xor: begin
                    C_w = ALU_A ^ ALU_B;
                end
                func3_sll: begin
                    C_w = ALU_A << ALU_B;
                end
                func3_slt: begin
                    C_w = ($signed(ALU_A) < $signed(ALU_B)) ? 32'b1 : 32'b0;
                end
                func3_sr: begin
                    if (SRA_r) begin
                        C_w = $signed(ALU_A) >>> ALU_B;
                    end
                    else begin
                        C_w = ALU_A >> ALU_B;
                    end
                end
                endcase
            end

            // load-word stall
            lw_hazard_w = (read_Dec2Exe_r && (
                    (rd_Dec2Exe_r == rs1 && rs1 != 0) || (rd_Dec2Exe_r == rs2 && rs2 != 0)) && !lw_hazard_r);  // switch on for only one cycle
            
            // passby
            pc_Exe2Mem_w = pc_Dec2Exe_r;
            rd_Exe2Mem_w = rd_Dec2Exe_r;
            read_Exe2Mem_w  = read_Dec2Exe_r;
            write_Exe2Mem_w = write_Dec2Exe_r;
            
            // branch_Exe2Mem_w = branch_Dec2Exe_r;
            pc_branch_Exe2Mem_w = pc_branch_Dec2Exe_r;
            jal_Exe2Mem_w = jal_Dec2Exe_r;
            jalr_Exe2Mem_w = jalr_Dec2Exe_r;
            cmp_Exe2Mem_w = cmp_Dec2Exe_r;
        end
        
    end
    
    // Stage 4: Memory Access
    assign DCACHE_addr = C_r[31:2];
    assign DCACHE_wdata = {sw_value_Exe2Mem_r[0+:8], sw_value_Exe2Mem_r[8+:8], sw_value_Exe2Mem_r[16+:8], sw_value_Exe2Mem_r[24+:8]};
    always@(*) begin
        
        DCACHE_ren = 1'b0;
        DCACHE_wen = 1'b0;
        if (read_Exe2Mem_r) begin
            DCACHE_ren = 1'b1;
        end
        else if (write_Exe2Mem_r) begin
            DCACHE_wen = 1'b1;
        end
        // passby
        rd_Mem2WB_w = rd_Exe2Mem_r;
        rd_value_w = (jal_Exe2Mem_r || jalr_Exe2Mem_r) ? pc_Exe2Mem_r + (cmp_Exe2Mem_r ? 3'd2 : 3'd4) : 
                        read_Exe2Mem_r ? {DCACHE_rdata[0+:8], DCACHE_rdata[8+:8], DCACHE_rdata[16+:8], DCACHE_rdata[24+:8]} : C_r;
        if (stall) begin
            rd_Mem2WB_w = rd_Mem2WB_r;
            rd_value_w = rd_value_r;
            
        end
        
    end
    // Stage 5: Write Back

    always@(*) begin
        for(i=1; i<32; i=i+1) begin
            r_w[i] = r_r[i];
        end
        if (!stall) begin
            if (rd_Mem2WB_r != 5'b0) begin
                r_w[rd_Mem2WB_r] = rd_value_r;
            end
        end
    end

    //==== sequential circuit =================================

    always@(posedge clk) begin
        if (~rst_n) begin
            // reset
            pc_r <= 0;
            reset_r <= 1'b1;
            ren_i_r <= 1'b0;
            for(i=1; i<32; i=i+1)
                r_r[i] <= 32'b0;
            instr_r <= 32'b0;
            half_instr_r <= 1'b0;
            A_r <= 32'b0;
            B_r <= 32'b0;
            rd_Dec2Exe_r <= 5'b0;
            ALU_op_r <= 3'b0;
            //branch_r <= 1'b0;
            //jump_r <= 1'b0;
            read_Dec2Exe_r  <= 1'b0;
            write_Dec2Exe_r <= 1'b0;
            sw_value_Dec2Exe_r <= 32'b0;
            SRA_r <= 1'b0;
            Mul_r <= 1'b0;
            C_r <= 32'b0;
            rd_Exe2Mem_r <= 5'b0;
            read_Exe2Mem_r  <= 1'b0;
            write_Exe2Mem_r <= 1'b0;
            sw_value_Exe2Mem_r <= 32'b0;
            rd_Mem2WB_r <= 5'b0;
            rd_value_r <= 32'b0;

            // rs1_Dec2Exe_r <= 5'd0;
            // rs2_Dec2Exe_r <= 5'd0;

            branch_Dec2Exe_r <= 1'b0;
            // branch_Exe2Mem_r <= 1'b0;
            bne_r <= 1'b0;
            pc_branch_Dec2Exe_r <= 32'b0;
            pc_branch_Exe2Mem_r <= 32'b0;

            jal_Dec2Exe_r <= 1'b0;
            jal_Exe2Mem_r <= 1'b0;
            // jal_Mem2WB_r <= 1'b0;
            jalr_Dec2Exe_r <= 1'b0;
            jalr_Exe2Mem_r <= 1'b0;
            // jalr_Mem2WB_r <= 1'b0;
            cmp_Dec2Exe_r <= 1'b0;
            cmp_Exe2Mem_r <= 1'b0;

            pc_If2Dec_r <= 32'b0;
            pc_Dec2Exe_r <= 32'b0;
            pc_Exe2Mem_r <= 32'b0;
            // pc_Mem2WB_r <= 32'b0;
            
            lw_hazard_r <= 1'b0;
            BrPre_If2Dec_r <= 1'b0;
            BrPre_Dec2Exe_r <= 1'b0;
        end
        else begin
            pc_r <= pc_w;
            reset_r <= 1'b0;
            ren_i_r <= ren_i_w;
            for(i=1; i<32; i=i+1)
                r_r[i] <= r_w[i];
            instr_r <= instr_w;
            half_instr_r <= half_instr_w;
            A_r <= A_w;
            B_r <= B_w;
            rd_Dec2Exe_r <= rd_Dec2Exe_w;
            ALU_op_r <= ALU_op_w;
            //branch_r <= branch_w;
            //jump_r <= jump_w;
            read_Dec2Exe_r  <= read_Dec2Exe_w;
            write_Dec2Exe_r <= write_Dec2Exe_w;
            sw_value_Dec2Exe_r <= sw_value_Dec2Exe_w;
            SRA_r <= SRA_w;
            Mul_r <= Mul_w;
            C_r <= C_w;
            rd_Exe2Mem_r <= rd_Exe2Mem_w;
            read_Exe2Mem_r  <= read_Exe2Mem_w;
            write_Exe2Mem_r <= write_Exe2Mem_w;
            sw_value_Exe2Mem_r <= sw_value_Exe2Mem_w;
            rd_Mem2WB_r <= rd_Mem2WB_w;
            rd_value_r <= rd_value_w;

            // rs1_Dec2Exe_r <= rs1_Dec2Exe_w;
            // rs2_Dec2Exe_r <= rs2_Dec2Exe_w;

            branch_Dec2Exe_r <= branch_Dec2Exe_w;
            // branch_Exe2Mem_r <= branch_Exe2Mem_w;
            bne_r <= bne_w;
            pc_branch_Dec2Exe_r <= pc_branch_Dec2Exe_w;
            pc_branch_Exe2Mem_r <= pc_branch_Exe2Mem_w;

            jal_Dec2Exe_r <= jal_Dec2Exe_w;
            jal_Exe2Mem_r <= jal_Exe2Mem_w;
            // jal_Mem2WB_r <= jal_Mem2WB_w;
            jalr_Dec2Exe_r <= jalr_Dec2Exe_w;
            jalr_Exe2Mem_r <= jalr_Exe2Mem_w;
            // jalr_Mem2WB_r <= jalr_Mem2WB_w;
            cmp_Dec2Exe_r <= cmp_Dec2Exe_w;
            cmp_Exe2Mem_r <= cmp_Exe2Mem_w;

            pc_If2Dec_r <= pc_If2Dec_w;
            pc_Dec2Exe_r <= pc_Dec2Exe_w;
            pc_Exe2Mem_r <= pc_Exe2Mem_w;
            // pc_Mem2WB_r <= pc_Mem2WB_w;
            
            lw_hazard_r <= lw_hazard_w;
            BrPre_If2Dec_r <= BrPre_If2Dec_w;
            BrPre_Dec2Exe_r <= BrPre_Dec2Exe_w;
        end
    end

endmodule