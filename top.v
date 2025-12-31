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
    reg [4:0]  id_ex_rs1_addr; // Forwarding Unit 檢查用
    reg [4:0]  id_ex_rs2_addr;
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
    reg [4:0]  ex_mem_rd_addr;
    reg [4:0]  ex_mem_rs1_addr; // Forwarding Unit 檢查用
    reg [4:0]  ex_mem_rs2_addr;
    // 控制訊號
    reg        ex_mem_reg_write;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg        ex_mem_mem_to_reg;

    // ---MEM/WB Stage Registers ---
    reg [4:0]  mem_wb_rd_addr;
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
    wire        branch_taken;
    wire [31:0] branch_target;

    // ID Stage Wires
    wire [31:0] rs1_data_out;
    wire [31:0] rs2_data_out;
    wire [31:0] imm; 
    wire [6:0]  id_opcode = if_id_inst[6:0];    // if_id_inst 切出 opcode 給 BranchResolutionUnit 用
    wire [2:0]  id_funct3 = if_id_inst[14:12];  // if_id_inst 切出 funct3 給 BranchResolutionUnit 用
    // Decoder 輸出的控制訊號
    wire        ctrl_reg_write; 
    wire        ctrl_mem_read;
    wire        ctrl_mem_write; 
    wire        ctrl_mem_to_reg; 
    wire        ctrl_alu_src;
    wire [3:0]  ctrl_alu_op;
    // HazardDetectionUnit 
    wire        pc_write;
    wire        if_id_write;
    wire        ctrl_flush;

    // EX Stage Wires
    wire [31:0] alu_input_a;
    wire [31:0] alu_input_b;
    wire [31:0] alu_result;
    // Forwarding 輸出的控制訊號
    wire [1:0]  forward_a;
    wire [1:0]  forward_b;

    // MEM Stage Wires

    // WB Stage Wires
    wire [31:0] wb_write_data;

//  3. 各 stage 
//  IF Stage 
    // 這是組合邏輯 (Combinational) -> 算路徑，隨插即用，只要 branch_taken 一變，pc_next 馬上變
    assign pc_next = (branch_taken) ? branch_target : (pc + 32'd4) ;
    // 這是時序邏輯 (Sequential) -> 只有敲鐘才換位置，根據剛剛算好的 pc_next，在鐘響瞬間跳過去
    always @(posedge clk or posedge rst) begin
        if      (rst)        pc <= 32'b0;
        else if (pc_write)   pc <= pc_next;  // 只有在 pc_write = 1 時才更新
        else                 pc <= pc;       // (保持原值，也就是 Stall)
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
            if_id_pc    <= 32'b0;
            if_id_inst  <= 32'b0;
        end
        else if (branch_taken) begin
            // RISC-V NOP 指令是 ADDI x0, x0, 0 , 機器碼是 32'h00000013
            if_id_inst  <= 32'h00000013; // 強制塞入 NOP (Flush)
            // 既然指令都變成 NOP 了，這個 PC 值其實沒人會在乎
            if_id_pc    <= 32'b0;        // PC 清零或保持都沒差
        end
        else if (if_id_write)  begin
            if_id_pc    <= pc;
            if_id_inst  <= instr_fetched;
        end
        else begin // if_id_write = 0 , 插氣泡 , 保持原值
            if_id_pc <= if_id_pc;
            if_id_inst <= if_id_inst;
        end
    end

//  ID Stage (解碼)
    // 實例化 
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
        .w_data     (wb_write_data),
        .reg_write  (mem_wb_reg_write),
        .rs1_data   (rs1_data_out),
        .rs2_data   (rs2_data_out)
    );
    ImmGen immgen_inst(
        .inst(if_id_inst), // imm 的位置各不相同
        .imm(imm)          // 剛剛宣告 imm; <--- 這裡為什麼接 "imm_out"？
    );
    HazardDetectionUnit HazardDetectionUnit_inst(
        .if_id_inst(if_id_inst),
        .id_ex_mem_read(id_ex_mem_read),
        .id_ex_rd_addr(id_ex_rd_addr),
        .branch_taken(branch_taken),
        .ex_mem_mem_read(ex_mem_mem_read),
        .ex_mem_rd_addr(ex_mem_rd_addr),
        .ex_mem_reg_write(ex_mem_reg_write),
        .mem_wb_reg_write(mem_wb_reg_write),
        .pc_write(pc_write),
        .if_id_write(if_id_write),
        .ctrl_flush(ctrl_flush)
    );
    // ID Forwarding MUX rs1_data_resolved & rs2_data_resolved
    wire [4:0] id_rs1_addr = if_id_inst[19:15];
    wire [4:0] id_rs2_addr = if_id_inst[24:20];
    reg [31:0] id_rs1_data_resolved;
    reg [31:0] id_rs2_data_resolved;
    always @(*) begin
        // 優先權 1: Forward from EX stage (ALU to Branch - Aggressive Forwarding)
        // 這是 critical path & stall 的 trade-off：如果上一條是 ALU 運算，直接 forward，不用 Stall！因為for loop 一直 stall 反而不划算 
        if (id_ex_reg_write && (id_ex_rd_addr != 0) && (id_ex_rd_addr == id_rs1_addr)) begin
            id_rs1_data_resolved = alu_result; 
        end
        // 優先權 2: Forward from MEM stage (Load Stall 後的資料，或是 ALU 運算)
        else if (ex_mem_reg_write && (ex_mem_rd_addr != 0) && (ex_mem_rd_addr == id_rs1_addr)) begin
            if (ex_mem_mem_read) id_rs1_data_resolved = RDATA_DM[31:0]; // 假設 Load Stall 結束，資料從 DM 回來了
            else                 id_rs1_data_resolved = ex_mem_alu_out;
        end
        // 優先權 3: Forward from WB stage
        else if (mem_wb_reg_write && (mem_wb_rd_addr != 0) && (mem_wb_rd_addr == id_rs1_addr)) begin
            id_rs1_data_resolved = wb_write_data;
        end
        // 預設: 讀 RegFile
        else begin
            id_rs1_data_resolved = rs1_data_out;
        end
    end
    always @(*) begin
        if (id_ex_reg_write && (id_ex_rd_addr != 0) && (id_ex_rd_addr == id_rs2_addr)) begin
            id_rs2_data_resolved = alu_result;
        end
        else if (ex_mem_reg_write && (ex_mem_rd_addr != 0) && (ex_mem_rd_addr == id_rs2_addr)) begin
            if (ex_mem_mem_read) id_rs2_data_resolved = RDATA_DM[31:0];
            else                 id_rs2_data_resolved = ex_mem_alu_out;
        end
        else if (mem_wb_reg_write && (mem_wb_rd_addr != 0) && (mem_wb_rd_addr == id_rs2_addr)) begin
            id_rs2_data_resolved = wb_write_data;
        end
        else begin
            id_rs2_data_resolved = rs2_data_out;
        end
    end

    BranchResolutionUnit BranchResolutionUnit_inst(
        .funct3(id_funct3),
        .opcode(id_opcode),
        .imm(imm),                       // 來自 ImmGen
        .pc(if_id_pc),                   // 來自 IF/ID PC
        .rs1_data(id_rs1_data_resolved), // [重要] 接上 Forwarding 修正後的資料
        .rs2_data(id_rs2_data_resolved), // [重要] 接上 Forwarding 修正後的資料
        .branch_taken(branch_taken),     // 輸出給 IF Stage 做 Flush
        .branch_target(branch_target)    // 輸出給 IF Stage 更新 PC
    );
    // --- ID/EX Pipeline Register 更新 ---
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            id_ex_pc            <= 32'b0; 
            id_ex_rs1_data      <=  5'b0; 
            id_ex_rs2_data      <=  5'b0; 
            id_ex_imm_ext       <= 32'b0; 
            id_ex_rd_addr       <= 32'b0;
            id_ex_rs1_addr      <=  5'b0;
            id_ex_rs2_addr      <=  5'b0;
            // 清空控制訊號
            id_ex_reg_write     <=  1'b0; 
            id_ex_mem_read      <=  1'b0; 
            id_ex_mem_write     <=  1'b0;
            id_ex_mem_to_reg    <=  1'b0; 
            id_ex_alu_src       <=  1'b0; 
            id_ex_alu_op        <=  1'b0;
        end 
        else if (ctrl_flush) begin
            // 把所有控制訊號歸零，模擬一個 NOP
            // PC 或 Data 保持 0 或不變都沒差，重點是控制訊號要是 0
            id_ex_reg_write     <=  1'b0;  // (不寫暫存器) → 安全
            id_ex_mem_read      <=  1'b0;  // (不讀記憶體，避免 Side effect) → 安全
            id_ex_mem_write     <=  1'b0;  // (不寫記憶體) → 安全
            id_ex_mem_to_reg    <=  1'b0;  // reg_write = 0 , mem_to_reg 選擇誰不重要
            id_ex_alu_src       <=  1'b0; 
            id_ex_alu_op        <=  1'b0;  // (做加法) → 沒差，反正算出來的結果會被丟到垃圾桶。
        end
        else begin 
            id_ex_pc            <= if_id_pc;
            id_ex_rs1_data      <= rs1_data_out;
            id_ex_rs2_data      <= rs2_data_out;
            id_ex_imm_ext       <= imm;
            id_ex_rd_addr       <= if_id_inst[11:7];  // rd  地址
            id_ex_rs1_addr      <= if_id_inst[19:15]; // rs1 地址
            id_ex_rs2_addr      <= if_id_inst[24:20]; // rs2 地址
            // 控制訊號
            id_ex_mem_read      <= ctrl_mem_read;   // 從 Decoder 的 output 接過來
            id_ex_mem_write     <= ctrl_mem_write;
            id_ex_mem_to_reg    <= ctrl_mem_to_reg;
            id_ex_alu_src       <= ctrl_alu_src;
            id_ex_reg_write     <= ctrl_reg_write;
            id_ex_alu_op        <= ctrl_alu_op;
        end
    end

// EX stage
    // 實例化 ForwardingUnit
    ForwardingUnit ForwardingUnit_inst(
        .rs1_addr(id_ex_rs1_addr),       // 記得剛剛在 step 1 加的
        .rs2_addr(id_ex_rs2_addr),
        .ex_mem_rd_addr(ex_mem_rd_addr),
        .ex_mem_reg_write(ex_mem_reg_write),
        .mem_wb_rd_addr(mem_wb_rd_addr),
        .mem_wb_reg_write(mem_wb_reg_write),
        .forward_a(forward_a), // Forwarding or not 的控制訊號
        .forward_b(forward_b)
    );
    // ALU Input MUX
    reg [31:0] alu_input_a_mux; // 為了在 always 裡寫加一個 reg
    reg [31:0] rs2_resolved;    // 處理 rs2 的 Forwarding (產生一個修正後的 rs2)
    assign alu_input_a = alu_input_a_mux; 
    assign alu_input_b = (id_ex_alu_src) ? id_ex_imm_ext : rs2_resolved;
    always @(*) begin
        case (forward_a)
            2'b00:      alu_input_a_mux = id_ex_rs1_data; // alu_input_a = id_ex_rs1_data;
            2'b01:      alu_input_a_mux = ex_mem_alu_out;
            2'b10:      alu_input_a_mux = wb_write_data;  // 接這條線才萬無一失 mem_wb_alu_out;
            default:    alu_input_a_mux = id_ex_rs1_data;
        endcase
        case (forward_b)
            2'b00:      rs2_resolved = id_ex_rs2_data; // assign alu_input_b = (id_ex_alu_src) ? id_ex_imm_ext : id_ex_rs2_data;
            2'b01:      rs2_resolved = ex_mem_alu_out;
            2'b10:      rs2_resolved = wb_write_data;  // 接這條線才萬無一失 // mem_wb_alu_out;
            default:    rs2_resolved = id_ex_rs2_data;
        endcase
    end
    // 實例化 ALU
    ALU ALU_inst(
        .alu_input_a(alu_input_a),
        .alu_input_b(alu_input_b),
        .alu_op(id_ex_alu_op),
        .alu_result(alu_result),
        .zero()
    );
    // --- EX/MEM Pipeline Register 更新 ---
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            ex_mem_alu_out      <= 32'b0;
            ex_mem_store_data   <= 32'b0;
            ex_mem_rd_addr      <= 32'b0;
            ex_mem_rs1_addr     <=  5'b0;
            ex_mem_rs2_addr     <=  5'b0;
            // 清空控制訊號
            ex_mem_reg_write    <=  1'b0;
            ex_mem_mem_read     <=  1'b0;
            ex_mem_mem_write    <=  1'b0;
            ex_mem_mem_to_reg   <=  1'b0;
        end 
        else begin
            ex_mem_alu_out      <= alu_result;
            ex_mem_store_data   <= rs2_resolved;   // store data 是 rs2 值 bypass ， 也要考慮 forwarding !
            ex_mem_rd_addr      <= id_ex_rd_addr;  // rd  地址
            ex_mem_rs1_addr     <= id_ex_rs1_addr; // rs1 地址
            ex_mem_rs2_addr     <= id_ex_rs2_addr; // rs2 地址
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
            mem_wb_rd_addr      <=  5'b0;
            // 清空控制訊號
            mem_wb_reg_write    <=  1'b0; 
            mem_wb_mem_to_reg   <=  1'b0;
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
    wire [6:0] opcode = inst[6:0];      
    wire [2:0] funct3 = inst[14:12];    
    wire [6:0] funct7 = inst[31:25];    
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
                mem_write  = 1'b1;
                alu_src    = 1'b1; // 算地址用 Imm
                alu_op     = ALU_ADD; // 地址 = rs1 + imm
            end
            default: begin
                // 未定義指令，保持全 0 或設一個錯誤旗標
            end
        endcase
    end
endmodule

module RegFile(
    input          clk,      
    input          rst,        
    input   [4:0]  rs1_addr,  // 不是 32-bits
    input   [4:0]  rs2_addr,   
    input   [4:0]  rd_addr,    
    input   [31:0] w_data,     
    input          reg_write,  
    output  [31:0] rs1_data,   
    output  [31:0] rs2_data
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
                reg_file[rd_addr] <= w_data;
            end
        end
    end
    assign rs1_data = (reg_write && (rd_addr != 5'b0) && (rd_addr == rs1_addr)) ? w_data : reg_file[rs1_addr];
    assign rs2_data = (reg_write && (rd_addr != 5'b0) && (rd_addr == rs2_addr)) ? w_data : reg_file[rs2_addr];
endmodule

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

module ForwardingUnit(
    input [4:0] rs1_addr,           // ex 階段 需 forward
    input [4:0] rs2_addr,   
    input [4:0] ex_mem_rd_addr,     // 上個指令
    input       ex_mem_reg_write,
    input [4:0] mem_wb_rd_addr,     // 上上個指令
    input       mem_wb_reg_write,
    output reg [1:0] forward_a,     // 控制 rs1
    output reg [1:0] forward_b      // 控制 rs2
);
    localparam no_hazard  = 2'b00; 
    localparam mem_hazard = 2'b01;  // MEM Forward (最優先) data_hazard_occur_at_mem_stage
    localparam wb_hazard  = 2'b10;  // WB  Forward (次優先) data_hazard_occur_at_wb_stage
    
    always @(*) begin
        forward_a = no_hazard; // 預設：不轉發
        if(ex_mem_reg_write && (ex_mem_rd_addr!=0) && (ex_mem_rd_addr==rs1_addr))
            forward_a = mem_hazard;
        else if(mem_wb_reg_write && (mem_wb_rd_addr!=0) && (mem_wb_rd_addr==rs1_addr))
            forward_a = wb_hazard;
        
        forward_b = no_hazard; // 預設：不轉發
        if(ex_mem_reg_write && (ex_mem_rd_addr!=0) && (ex_mem_rd_addr==rs2_addr))
            forward_b = mem_hazard;
        else if(mem_wb_reg_write && (mem_wb_rd_addr!=0) && (mem_wb_rd_addr==rs2_addr))
            forward_b = wb_hazard;
    end
endmodule 

module HazardDetectionUnit(
    input        id_ex_mem_read,
    input [31:0] if_id_inst,
    input [4:0]  id_ex_rd_addr,

    // 檢查前面指令有沒有 LW data hazard
    input        ex_mem_mem_read,
    input        ex_mem_rd_addr,
    // 檢查前面指令有沒有 單純拿 ALU 算出來的東西的 data hazard
    input        ex_mem_reg_write,
    input        mem_wb_reg_write,
    // 輸出控制訊號
    output reg pc_write,
    output reg if_id_write,
    output reg ctrl_flush
); 
    wire [6:0] opcode   = if_id_inst[6:0];
    wire [4:0] rs1_addr = if_id_inst[19:15];
    wire [4:0] rs2_addr = if_id_inst[24:20];
    // [修正] 判斷這是不是一條 Branch 指令 (B-Type)
    // 只要是 Branch，我們就要特別小心處理資料相依
    wire is_branch = (opcode == 7'b1100011); 
    // 補充：其實 JALR (1100111) 也需要讀 rs1 來算跳轉目標
    // 如果你要更嚴謹，可以把 JALR 也加進去：
    wire is_jump_reg = (opcode == 7'b1100111);

    always @(*) begin
        // 預設值：不 Stall
        pc_write    = 1'b1;
        if_id_write = 1'b1;
        ctrl_flush  = 1'b0;
        
        // ==========================================================
        // 條件 A: 標準 Load-Use Hazard (Distance 1)
        // ==========================================================
        // 上一條指令 (EX) 是 Load，且寫入目標等於目前的 rs1 或 rs2。
        // 這會擋住所有需要用 rs1/rs2 的指令 (包含 ADD, SUB, 以及 Branch)
        if (id_ex_mem_read && (id_ex_rd_addr == rs1_addr || id_ex_rd_addr == rs2_addr)) begin
            pc_write    = 1'b0;
            if_id_write = 1'b0;
            ctrl_flush  = 1'b1;
        end
        
        // ==========================================================
        // 條件 B: Branch Specific Load-Use Hazard (Distance 2)
        // ==========================================================
        // 上上條指令 (MEM) 是 Load，且目前的 ID 是 Branch。
        // 因為我們為了 Ranking 1 決定不從 MEM 拉線回 ID (怕太慢)，
        // 所以必須在這裡多 Stall 一個 Cycle，等 Load 走到 WB 階段。
        else if (ex_mem_mem_read && (ex_mem_rd_addr == rs1_addr || ex_mem_rd_addr == rs2_addr) && is_branch) begin
            // [關鍵] 這裡用 is_branch，而不是 branch_taken！
            pc_write    = 1'b0;
            if_id_write = 1'b0;
            ctrl_flush  = 1'b1;
        end

    end
endmodule

module BranchResolutionUnit ( 
    input [2:0]   funct3,   
    input [6:0]   opcode,        
    input [31:0]  imm,              // 來自 ImmGen
    input [31:0]  pc,               // 來自 IF/ID PC (為了算 JAL/Branch target)
    input [31:0]  rs1_data,         // 來自 forwarding MUX 選擇過 ???
    input [31:0]  rs2_data,
    output        branch_taken,     // 告訴 Top Level 要不要 Flush
    output [31:0] branch_target     // 告訴 PC Mux 下一跳去哪
);
    // 優化：branch_target共用同一個加法器
    wire [31:0] base_addr;
    assign base_addr = (opcode == 7'b1100111) ? rs1_data : pc;
    assign branch_target = base_addr + imm;

    always @(*) begin
        branch_taken = 1'b0;  // 預設：沒 mispredict（predict not taken）
        case (opcode)
            7'b1101111: branch_taken = 1'b1; // JAL
            7'b1100111: branch_taken = 1'b1; // JALR: rd=PC+4, PC=imm+rs1
            7'b1100011: begin
                case (funct3)
                    3'b000: branch_taken = (rs1_data == rs2_data); // BEQ
                    3'b001: branch_taken = (rs1_data != rs2_data); // BNE
                    3'b100: branch_taken = ($signed(rs1_data) <  $signed(rs2_data)); // BLT
                    3'b101: branch_taken = ($signed(rs1_data) >= $signed(rs2_data)); // BGE
                    3'b110: branch_taken = (rs1_data <  rs2_data); // BLTU (Unsigned)
                    3'b111: branch_taken = (rs1_data >= rs2_data); // BGEU (Unsigned)
                    default: branch_taken = 1'b0;
                endcase
            end
            default: branch_taken = 1'b0;
        endcase
end
endmodule