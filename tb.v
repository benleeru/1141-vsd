`timescale 1ns/10ps
`include "top.v" // 記得確認你的 top 檔名

/*
x1 必須是 0x01000000。
這證明你的 ALU_AUIPC 機制成功運作，正確抓到了 PC 和 Imm。
如果看到 4，代表 MUX 邏輯有錯。

x3 (Return Address) 必須是 16 (0x10)。
這證明 JALR 的寫回功能正確。

x4 必須是 88 (0x58)。
這證明 CPU 成功跳到了 0x1000。

請觀察波形中的 pc 訊號，在執行 JALR 後，pc 應該直接變成 0x1000。
如果 pc 變成 0x1001，代表你的 Masking (& ~1) 沒生效。
*/

module tb;
    reg clk;
    reg rst;
    
    // 介面訊號
    wire [31:0] ARADDR_IM, ARADDR_DM, AWADDR_DM;
    wire ARVALID_IM, ARVALID_DM, AWVALID_DM, WVALID_DM;
    reg  [127:0] RDATA_IM, RDATA_DM;
    wire [127:0] WDATA_DM;
    
    // 擴大記憶體空間以容納跳轉測試
    reg [31:0] IM_MEM [0:4095]; 
    reg [31:0] DM_MEM [0:4095]; 
    integer i;

    // 實例化 CPU
    top CPU (
        .clk(clk), .rst(rst),
        .ARADDR_IM(ARADDR_IM), .ARVALID_IM(ARVALID_IM), .ARREADY_IM(1'b1),
        .RDATA_IM(RDATA_IM), .RVALID_IM(1'b1), .RREADY_IM(),
        .ARADDR_DM(ARADDR_DM), .ARVALID_DM(ARVALID_DM), .ARREADY_DM(1'b1),
        .RDATA_DM(RDATA_DM), .RVALID_DM(1'b1), .RREADY_DM(),
        .AWADDR_DM(AWADDR_DM), .AWVALID_DM(AWVALID_DM), .AWREADY_DM(1'b1),
        .WDATA_DM(WDATA_DM), .WVALID_DM(WVALID_DM), .WREADY_DM(1'b1), .WSTRB_DM()
    );

    always #5 clk = ~clk;

    // 記憶體模型 (Byte Address -> Word Index)
    // 這裡我們模擬簡單的 Memory，忽略未對齊存取 (因為 CPU 應該要自己對齊)
    always @(posedge clk) begin
        if (AWVALID_DM && WVALID_DM) begin
            DM_MEM[AWADDR_DM[13:2]] <= WDATA_DM[31:0];
        end
    end
    
    always @(*) begin
        // IM read
        if (ARVALID_IM) RDATA_IM = {96'b0, IM_MEM[ARADDR_IM[13:2]]};
        else            RDATA_IM = 0;
        
        // DM read
        if (ARVALID_DM) RDATA_DM = {96'b0, DM_MEM[ARADDR_DM[13:2]]};
        else            RDATA_DM = 0;
    end

    initial begin
        clk = 0; rst = 1;
        // 初始化 NOP
        for (i=0; i<4096; i=i+1) begin IM_MEM[i]=32'h00000013; DM_MEM[i]=0; end

        // =============================================================
        //  Phase 1 Strict Verification Program
        // =============================================================
        
        // 1. [AUIPC Test] 測試你的 MUX 優化是否正確
        // PC=0: AUIPC x1, 0x1000 (x1 = 0 + 0x01000000)
        // 預期: x1 = 0x01000000 (如果 MUX 錯選成 JAL 的 4，這裡會變 4)
        IM_MEM[0] = 32'h01000097; 

        // 2. [LUI Test] 基礎大數載入
        // PC=4: LUI x2, 0x1 (x2 = 0x00001000)
        IM_MEM[1] = 32'h00001137;

        // 3. [JALR Masking Test] 測試最低位遮罩
        // 準備一個奇數地址： x2 = x2 + 1 = 0x00001001
        // PC=8: ADDI x2, x2, 1
        IM_MEM[2] = 32'h00110113; 
        
        // 嘗試跳轉到 x2 (0x1001)
        // 預期：CPU 應該將其修正為 0x1000 (Idx 1024)，而不是 0x1001
        // 並且將 Return Address (PC+4=16) 寫入 x3
        // PC=12: JALR x3, 0(x2)
        IM_MEM[3] = 32'h000101E7;

        // --- 這裡應該被跳過 (PC 16~...) ---
        
        // 4. [Target Address] 
        // 這是地址 0x1000 (Word Index 1024)
        // 如果 JALR 沒切 bit 0，可能會抓不到這行，或者抓錯
        // PC=4096 (0x1000): ADDI x4, x0, 88 (Flag)
        IM_MEM[1024] = 32'h05800213; 

        // 結束 (NOP loop)
        IM_MEM[1025] = 32'h00000013;

        #20 rst = 0;
        
        // 等待足夠時間讓 CPU 執行到遠端跳轉
        #2000;
        
        $display("Strict Test Finished. Check Waveforms.");
        $finish;
    end
endmodule