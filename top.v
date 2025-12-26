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
    reg [4:0]  id_ex_rs1_data;
    reg [4:0]  id_ex_rs2_data;
    reg [31:0] id_ex_imm_ext;
    reg [4:0]  id_ex_rd_addr;
    reg [2:0]  id_ex_funct3;
    reg [6:0]  id_ex_funct7;
    // 控制訊號
    reg        id_ex_reg_write;
    reg        id_ex_mem_read;
    reg        id_ex_mem_write;
    reg        mem_to_reg;
    reg        alu_src;
    reg [3:0]  alu_op;

    // ---EX/MEM Stage Registers ---
    reg [31:0] ex_mem_alu_out;
    reg [4:0]  ex_mem_store_data;
    reg [31:0] ex_mem_rd_addr;
    // 控制訊號
    reg        ex_mem_reg_write;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg        ex_mem_mem_to_reg;

    // ---MEM/WB Stage Registers ---
    reg        mem_wb_rd_addr;
    // 控制訊號
    reg        mem_wb_mem_to_reg;

//  2. 宣告 Wire (連接模組用)

    // IF Stage Wires
    wire [31:0] pc;
    wire [31:0] pc_next;
    wire [31:0] instr_fetched;

    // ID Stage Wires
    wire [31:] rs1_data_out, rs2_data_out;
    wire [31:0] imm;
    //Decoder 輸出的控制訊號
    wire ctrl_reg_write, ctrl_mem_read, ctrl_mem_write, 
         ctrl_mem_to_reg, ctrl_alu_src;
    wire [3:0] ctrl_alu_op;

    // EX Stage Wires
    wire [31:0] alu_input_a;
    wire [31:0] alu_input_b;
    wire [31:0] alu_result;

    // MEM Stage Wires

    // WB Stage Wires
    wire [31:0] wb_write_data;

//  3. 各 stage 
//  IF Stage (取指)
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
        .opcode     (if_id_inst[6:0]),
        // 輸出控制訊號
        .reg_write  (ctrl_reg_write),
        .alu_src    (ctrl_alu_src),
        .mem_read   (ctrl_mem_read),
        .mem_write  (ctrl_mem_write),
        .mem_to_reg (ctrl_mem_to_reg),
        .imm_type   (ctrl_imm_type),
        .alu_op     (ctrl_alu_op)
    );
    // 實例化 RegFile
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
    // 實例化 ImmGen
    ImmGen immgen_inst(
        .imm(if_id_inst), // imm 的位置各不相同
        .imm(imm_out)
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
            id_ex_imm_ext       <= imm_out;
            id_ex_rd_addr       <= if_id_inst[11:7]; // rd
            // 控制訊號
            id_ex_reg_write     <= reg_write;
            id_ex_mem_read      <= alu_src;
            id_ex_mem_write     <= mem_read;
            id_ex_mem_to_reg    <= mem_write;
            id_ex_alu_src       <= mem_to_reg;
            id_ex_alu_op        <= alu_op;
        end
    end

// EX stage
    // ALU Mux (2-to-1 MUX)
    assign alu_input_a = id_ex_rs1_data;
    assign alu_input_b = (id_ex_alu_src) ? id_ex_imm_ext : id_ex_rs2_data;
    // 實例化 ALU
    ALU ALU_inst(
        .src1(alu_input_a),
        .src2(alu_input_b),
        .alu_op(id_ex_alu_op),
        .result(alu_result)
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
    // 在 WB Stage 執行 存回暫存器，沒有操控信號嗎，這樣寫可以嗎
    always @(*) begin
        regfile [mem_wb_rd_addr] = wb_write_data;
    end
endmodule

module Decoder(
    input   [6:0] opcode,
    output        reg_write,
    output        alu_src,
    output        mem_read,
    output        mem_write,
    output        mem_to_reg,
    output        imm_type,
    output  [3:0] alu_op
);
    always @(*) begin
        case (opcode) 
            
        endcase
    end
endmodule
/*
    Decoder decoder_inst(
        .opcode     (if_id_inst[6:0]),
        // 輸出控制訊號
        .reg_write  (ctrl_reg_write),
        .alu_src    (ctrl_alu_src),
        .mem_read   (ctrl_mem_read),
        .mem_write  (ctrl_mem_write),
        .mem_to_reg (ctrl_mem_to_reg),
        .imm_type   (ctrl_imm_type),
        .alu_op     (ctrl_alu_op)
    );
*/
module RegFile();

endmodule

module Imm_Gen();

endmodule

module ALU();

endmodule
