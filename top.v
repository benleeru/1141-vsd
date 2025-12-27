`include "../sim/data_array/data_array_wrapper.v"
`include "../sim/tag_array/tag_array_wrapper.v"

module top (
    input           clk,
    input           rst,

    //=================================================================
    // IO Interface for connecting to IM
    //=================================================================
    // AR channel
    output [31:0]   ARADDR_IM,
    output          ARVALID_IM,
    input           ARREADY_IM,
    // R channel
    input  [127:0]  RDATA_IM,
    input           RVALID_IM,
    output          RREADY_IM,

    //=================================================================
    // IO Interface for connecting to DM
    //=================================================================
    // AR channel
    output [31:0]   ARADDR_DM,
    output          ARVALID_DM,
    input           ARREADY_DM,
    // R channel
    input  [127:0]  RDATA_DM,
    input           RVALID_DM,
    output          RREADY_DM,

    // AW channel
    output [31:0]   AWADDR_DM,
    output          AWVALID_DM,
    input           AWREADY_DM,

    // W channel
    output [127:0]  WDATA_DM,
    output [15:0]   WSTRB_DM,
    output          WVALID_DM,
    input           WREADY_DM
);

// 1. 宣告 Pipeline Registers

    // --- IF/ID Stage Registers ---
    reg [31:0] if_id_pc;
    reg [31:0] if_id_inst;

    // --- ID/EX Stage Registers ---
    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_rs1_data;
    reg [31:0] id_ex_rs2_data;
    reg [31:0] id_ex_imm_ext;
    reg [4:0]  id_ex_rd_addr;
    reg [2:0]  id_ex_funct3;
    reg [6:0]  id_ex_funct7;
    // 控制訊號
    reg        id_ex_reg_write;
    reg        id_ex_mem_read;
    reg        id_ex_mem_write;
    reg        id_ex_mem_to_reg;
    reg        id_ex_alu_src;
    reg [3:0]  id_ex_alu_op;

    // ---EX/MEM Stage Registers ---
    reg [31:0] ex_mem_alu_out;
    reg [31:0] ex_mem_store_data;
    reg [31:0] ex_mem_rd_addr;
    // 控制訊號
    reg        ex_mem_reg_write;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg        ex_mem_mem_to_reg;

    // ---MEM/WB Stage Registers ---
    reg [31:0] mem_wb_rd_addr;
    reg [31:0] mem_wb_alu_out;
    reg [31:0] mem_wb_mem_data;
    // 控制訊號
    reg        mem_wb_reg_write;
    reg        mem_wb_mem_to_reg;

//  2. 宣告 Wire (連接模組用)

    // IF Stage Wires
    reg  [31:0] pc;
    wire [31:0] pc_next;
    wire [31:0] instr_fetched;

    // ID Stage Wires
    wire [31:0] rs1_data_out, rs2_data_out;
    wire [31:0] imm;
    //Decoder 輸出的控制訊號
    wire        ctrl_reg_write; 
    wire        ctrl_mem_read;
    wire        ctrl_mem_write; 
    wire        ctrl_mem_to_reg; 
    wire        ctrl_alu_src;
    wire [3:0]  ctrl_alu_op;

    // EX Stage Wires
    wire [31:0] alu_input_a;
    wire [31:0] alu_input_b;
    wire [31:0] alu_result;

    // MEM Stage Wires

    // WB Stage Wires
    wire [31:0] wb_write_data;

//  3. 各 stage 
//  IF Stage 
    assign pc_next = pc + 32'd4;
    always @(posedge clk or posedge rst) begin
        if (rst) pc <= 32'b0;
        else  pc <= pc_next;
    end
    // --- 連接 IM 介面 (簡化複雜的 AXI) ---
    assign ARADDR_IM  = pc;
    assign ARVALID_IM = 1'b1;
    assign RREADY_IM  = 1'b1;
    // 讀回來的指令
    assign instr_fetched = RDATA_IM[31:0];
    // --- IF/ID Pipeline Register 更新 ---
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            if_id_pc <= 32'b0;
            if_id_inst <= 32'b0;
        end
        else begin
            if_id_pc <= pc;
            if_id_inst <= instr_fetched;
        end
    end

//  ID Stage (解碼)
    // 實例化 decoder
    Decoder decoder_inst(
        .inst       (if_id_inst),
        // 輸出控制訊號
        .reg_write  (ctrl_reg_write),
        .alu_src    (ctrl_alu_src),
        .mem_read   (ctrl_mem_read),
        .mem_write  (ctrl_mem_write),
        .mem_to_reg (ctrl_mem_to_reg),
        .imm_type   (ctrl_imm_type),
        .alu_op     (ctrl_alu_op)
    );
    RegFile regfile_inst(
        .clk        (clk),
        .rst        (rst),
        .rs1_addr   (if_id_inst[19:15]),
        .rs2_addr   (if_id_inst[24:20]),
        .rd_addr    (mem_wb_rd_addr), 
        .wdata      (wb_write_data),
        .reg_write  (mem_wb_reg_write),
        .rs1_data   (rs1_data_out),
        .rs2_data   (rs2_data_out)
    );
    ImmGen immgen_inst(
        .inst(if_id_inst), // imm 的位置各不相同
        .imm(imm)          // 剛剛宣告 imm; <--- 這裡為什麼接 "imm_out"？
    );
    // --- ID/EX Pipeline Register 更新 ---
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            id_ex_pc            <= 32'b0; 
            id_ex_rs1_data      <=  5'b0; 
            id_ex_rs2_data      <=  5'b0; 
            id_ex_imm_ext       <= 32'b0; 
            id_ex_rd_addr       <= 32'b0;
            //reg [2:0]  id_ex_funct3; ??為什麼不用清空
            //reg [6:0]  id_ex_funct7; ??
            // 清空控制訊號
            id_ex_reg_write     <=  1'b0; 
            id_ex_mem_read      <=  1'b0; 
            id_ex_mem_write     <=  1'b0;
            id_ex_mem_to_reg    <=  1'b0; 
            id_ex_alu_src       <=  1'b0; 
            id_ex_alu_op        <=  1'b0;
        end 
        else begin 
            id_ex_pc            <= if_id_pc;
            id_ex_rs1_data      <= rs1_data_out;
            id_ex_rs2_data      <= rs2_data_out;
            id_ex_imm_ext       <= imm;
            id_ex_rd_addr       <= if_id_inst[11:7]; // rd
            // 控制訊號
            id_ex_mem_read   <= ctrl_mem_read;   // 從 Decoder 的 output 接過來
            id_ex_mem_write  <= ctrl_mem_write;
            id_ex_mem_to_reg <= ctrl_mem_to_reg;
            id_ex_alu_src    <= ctrl_alu_src;
            id_ex_reg_write  <= ctrl_reg_write;
            id_ex_alu_op     <= ctrl_alu_op;
        end
    end

// EX stage
    // ALU Mux (2-to-1 MUX)
    assign alu_input_a = id_ex_rs1_data;
    assign alu_input_b = (id_ex_alu_src) ? id_ex_imm_ext : id_ex_rs2_data;
    // 實例化 ALU
    ALU ALU_inst(
        .alu_input_a(alu_input_a),
        .alu_input_b(alu_input_b),
        .alu_op(id_ex_alu_op),
        .alu_result(alu_result)
    );
    // --- EX/MEM Pipeline Register 更新 ---
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            ex_mem_alu_out      <= 32'b0;
            ex_mem_store_data   <= 32'b0;
            ex_mem_rd_addr      <= 32'b0;
            // 清空控制訊號
            ex_mem_reg_write    <=  1'b0;
            ex_mem_mem_read     <=  1'b0;
            ex_mem_mem_write    <=  1'b0;
            ex_mem_mem_to_reg   <=  1'b0;
        end 
        else begin
            ex_mem_alu_out      <= alu_result;
            ex_mem_store_data   <= id_ex_rs2_data; // store data 是 rs2 的值 bypass
            ex_mem_rd_addr      <= id_ex_rd_addr;
            // 控制訊號
            ex_mem_reg_write    <=  id_ex_reg_write;
            ex_mem_mem_read     <=  id_ex_mem_read;
            ex_mem_mem_write    <=  id_ex_mem_write;
            ex_mem_mem_to_reg   <=  id_ex_mem_to_reg;
        end 
    end

// MEM stage
    // --- 連接 DM 介面 (這裡大概率會是錯的 debug 要很小心這裡!!) ---
    // 讀取邏輯 (Load)
    assign ARADDR_DM  = ex_mem_alu_out;     // 地址來自 ALU
    assign ARVALID_DM = ex_mem_mem_read;    // 只有 Load 指令才讀
    assign RREADY_DM  = 1'b1;               // 永遠準備好
    // 寫入邏輯 (Store)
    assign AWADDR_DM  = ex_mem_alu_out;     // 寫地址
    assign AWVALID_DM = ex_mem_mem_write;   // 只有 Store 指令才寫
    assign WDATA_DM   = {96'b0, ex_mem_store_data}; // 寫資料 (補滿128bit)
    assign WVALID_DM  = ex_mem_mem_write;
    assign WREADY_DM  = 1'b1; // 假設
    assign WSTRB_DM   = (ex_mem_mem_write) ? 16'hFFFF : 16'h0; // 簡單全開，之後要依據 SW/SH/SB 改
    // --- MEM/WB Pipeline Register 更新 ---
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mem_wb_alu_out      <= 32'b0; 
            mem_wb_mem_data     <= 32'b0; 
            mem_wb_rd_addr      <= 32'b0;
            // 清空控制訊號
            mem_wb_reg_write    <= 1'b0; 
            mem_wb_mem_to_reg   <= 1'b0;
        end 
        else begin
            mem_wb_alu_out      <= ex_mem_alu_out; 
            mem_wb_mem_data     <= RDATA_DM[31:0]; // 假設資料在低 32-bit 
            mem_wb_rd_addr      <= ex_mem_rd_addr;
            // 控制訊號
            mem_wb_reg_write    <= ex_mem_reg_write; 
            mem_wb_mem_to_reg   <= ex_mem_mem_to_reg;
        end
    end

// WB stage
    // Write Back MUX
    assign wb_write_data = (mem_wb_mem_to_reg) ? mem_wb_mem_data : mem_wb_alu_out;
    // 在 WB Stage 執行 存回暫存器，已經接線正確地實例化 RegFile 了，不要在 top 裡畫蛇添足
endmodule

module Decoder(
    input      [31:0] inst,
    output reg        reg_write,
    output reg        alu_src,
    output reg        mem_read,
    output reg        mem_write,
    output reg        mem_to_reg,
    output reg        imm_type,
    output reg [3:0]  alu_op
);
    wire [6:0] opcode = inst[6:0];      // wire opcode = inst[6:0];  // <--- 兇手在這裡！！！
    wire [2:0] funct3 = inst[14:12];    // [2:0] 忘記寫會出大事 !!!
    wire [6:0] funct7 = inst[31:25];    // [6:0] 忘記寫會出大事 !!!
    // 定義 RISC-V Spec 參數 
    localparam OP_R_TYPE = 7'b0110011; // ADD, SUB...
    localparam OP_I_TYPE = 7'b0010011; // ADDI...
    localparam OP_LOAD   = 7'b0000011; // LW...
    localparam OP_STORE  = 7'b0100011; // SW...

    // alu_op -> 定義 ALU 的動作
    localparam ALU_ADD = 4'b0000;
    localparam ALU_SUB = 4'b0001; // 預留給以後用

    always @(*) begin
        // 預設值 (避免 Latch，這很重要！) ?????
        reg_write =     1'b0;
        alu_src =       1'b0;
        mem_read =      1'b0;
        mem_write =     1'b0;
        mem_to_reg =    1'b0;
        imm_type =      1'b0;
        alu_op =        ALU_ADD;

        case (opcode) 
            OP_R_TYPE: begin
                reg_write = 1'b1;
                alu_src   = 1'b0; // 用 rs2
                // 這裡要判斷 funct7 是 ADD 還是 SUB，Phase 1 只有 ADD
                alu_op    = ALU_ADD;
            end
            OP_I_TYPE: begin // ADDI x1, x2, 10
                reg_write = 1'b1;
                alu_src   = 1'b1; // 用 Imm
                alu_op    = ALU_ADD;
            end
            OP_LOAD: begin // LW x1, 0(x2)
                reg_write  = 1'b1;
                mem_read   = 1'b1;
                mem_to_reg = 1'b1; // 來自 Memory
                alu_src    = 1'b1; // 算地址用 Imm
                alu_op     = ALU_ADD; // 地址 = rs1 + imm
            end
            OP_STORE: begin // SW x1, 0(x2)
                mem_write = 1'b1;
                alu_src   = 1'b1; // 算地址用 Imm
                alu_op    = ALU_ADD; // 地址 = rs1 + imm
            end
            default: begin
                // 未定義指令，保持全 0 或設一個錯誤旗標
            end
        endcase
    end
endmodule

module RegFile(
    input            clk,      
    input            rst,        
    input   [4:0]   rs1_addr,  // 不是 32-bits
    input   [4:0]   rs2_addr,   
    input   [4:0]   rd_addr,    
    input   [31:0]   wdata,     
    input            reg_write,  
    output  [31:0]   rs1_data,   
    output  [31:0]   rs2_data
);
    reg [31:0] reg_file [0:31];
    integer i; // 用來跑迴圈 reset 用的
    // 寫入邏輯 (Sequential - 只信任 clk edge)
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            // 全部清零
            for (i=0; i<32; i=i+1) 
                reg_file[i] <= 32'b0;
        end 
        else begin 
            // 目標地址 rd_addr 不可以是 0 (x0 永遠是 0，不能被改寫!)
            if (reg_write && rd_addr != 5'b0) begin
                reg_file[rd_addr] <= wdata;
            end
        end
    end
    // Reading is just Looking -> 下游 Don't Care 就好了 
    assign rs1_data = reg_file[rs1_addr];
    assign rs2_data = reg_file[rs2_addr];
endmodule

// 把imm 判斷邏輯直接寫在 decoder裡面會比較省面積嗎，幾乎不會
module ImmGen(
    input      [31:0] inst,
    output reg [31:0] imm
);
    // 切出 Opcode 來判斷是什麼 Type
    wire [6:0] opcode = inst[6:0];

    // I-Type: inst[31:20]
    wire [31:0] imm_i = {{20{inst[31]}}, inst[31:20]};
  
    // S-Type: inst[31:25] + inst[11:7]
    wire [31:0] imm_s = {{20{inst[31]}}, inst[31:25], inst[11:7]};

    // B-Type (Branch): 比較複雜，位置很亂，且最後一位補 0
    // 順序: bit 12, bit 10:5, bit 4:1, bit 11, 0
    wire [31:0] imm_b = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0};
    
    // U-Type (LUI, AUIPC): 放在高位 [31:12]，低位補 0
    wire [31:0] imm_u = {inst[31:12], 12'b0};

    // J-Type (JAL): 也是很亂
    // 順序: bit 20, bit 10:1, bit 11, bit 19:12, 0
    wire [31:0] imm_j = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], 1'b0};

    always @(*) begin
        case (opcode) 
            // I-Type 指令 (ADDI, LW, JALR 等)
            7'b0010011, // OP-IMM (ADDI...)
            7'b0000011, // LOAD (LW...)
            7'b1100111: // JALR
                imm = imm_i;
            // S-Type 指令 (SW 等)
            7'b0100011: // STORE
                imm = imm_s;
            // B-Type 指令 (BEQ, BNE 等) - Phase 2 會用到
            7'b1100011: // BRANCH
                imm = imm_b;           
            // U-Type 指令 (LUI, AUIPC)
            7'b0110111, // LUI
            7'b0010111: // AUIPC
                imm = imm_u;
            // J-Type 指令 (JAL)
            7'b1101111: // JAL
                imm = imm_j;
            default: 
                imm = 32'b0; // R-Type 不需要立即數，輸出 0 即可
        endcase 
    end
endmodule

module ALU(
    input      [31:0] alu_input_a,
    input      [31:0] alu_input_b,
    input      [3:0]  alu_op,
    output reg [31:0] alu_result,
    output            zero  // 順便做出來，以後 Branch 會感謝你
);
    // alu_op
    localparam ALU_ADD = 4'b0000;
    localparam ALU_SUB = 4'b0001; // 預留給以後用
    localparam ALU_AND = 4'b0010; // Phase 1 先不用，預留
    localparam ALU_OR  = 4'b0011; // Phase 1 先不用，預留

    // Zero Flag 的邏輯
    // 這是 Combinational 的，只要 result 是 0，這個燈就亮
    // 雖然是用 assign，但它其實是一個 32-input 的 NOR gate
    assign zero = (alu_result == 32'b0);

    always @(*) begin
        alu_result = 32'b0; 
        case (alu_op)
            ALU_ADD:
                alu_result = alu_input_a + alu_input_b;
            ALU_SUB:
                alu_result = alu_input_a - alu_input_b;
            default:
                alu_result = 32'b0;
        endcase
    end
endmodule
