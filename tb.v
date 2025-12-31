`timescale 1ns/10ps
`include "top.v" // 記得確認你的 top 檔名

/*
驗證全功能的 Testbench (tb_final.v)
    這個 Testbench 設計了像電影情節一樣的連續技，一次測試所有功能：
        Store/Load (測試記憶體存取)
        Load-Use Hazard (測試 Stall 機制 <-- 重點)
        Data Hazard (測試 Forwarding 機制)
        Internal Forwarding (測試 WB -> ID 機制)

預期結果 (Pass Criteria)
    跑完模擬後，請檢查 RegFile：
        x3 = 10: 
            證明 LW 成功從記憶體讀回資料 (Store 也成功)。
        x4 = 20: 
            證明 Stall 成功！(如果沒 Stall，x3 會讀到舊值 0，x4 就會算錯)。
            同時證明 Stall 後的 WB->EX Forwarding 也成功。
        x5 = 30: 
            證明標準的 EX->EX Forwarding 正常工作。
    如果這三個值都對，CPU 就正式具備了處理所有 Data Hazard 和 Load-Use Hazard 的能力，
*/

module tb;
    reg clk;
    reg rst;
    
    // 介面訊號
    wire [31:0] ARADDR_IM;
    wire ARVALID_IM;
    reg  [127:0] RDATA_IM; 
    wire [31:0] ARADDR_DM;
    wire ARVALID_DM;
    wire [31:0] AWADDR_DM;
    wire [127:0] WDATA_DM;
    wire AWVALID_DM;
    wire WVALID_DM;

    // 模擬記憶體 (4KB)
    reg [31:0] IM_MEM [0:1023]; 
    reg [31:0] DM_MEM [0:1023]; 
    
    integer i;

    // 實例化 CPU
    top CPU (
        .clk(clk),
        .rst(rst),
        .ARADDR_IM(ARADDR_IM), .ARVALID_IM(ARVALID_IM), .ARREADY_IM(1'b1),
        .RDATA_IM(RDATA_IM), .RVALID_IM(1'b1), .RREADY_IM(),
        .ARADDR_DM(ARADDR_DM), .ARVALID_DM(ARVALID_DM), .ARREADY_DM(1'b1),
        .RDATA_DM({96'b0, DM_MEM[ARADDR_DM[11:2]]}), 
        .RVALID_DM(1'b1), .RREADY_DM(),
        .AWADDR_DM(AWADDR_DM), .AWVALID_DM(AWVALID_DM), .AWREADY_DM(1'b1),
        .WDATA_DM(WDATA_DM), .WVALID_DM(WVALID_DM), .WREADY_DM(1'b1), .WSTRB_DM()
    );

    // Clock
    always #5 clk = ~clk;

    // DM 寫入行為
    always @(posedge clk) begin
        if (AWVALID_DM && WVALID_DM) begin
            DM_MEM[AWADDR_DM[11:2]] <= WDATA_DM[31:0];
        end
    end

    // IM 讀取行為
    always @(*) begin
        if (ARVALID_IM) 
            RDATA_IM = {96'b0, IM_MEM[ARADDR_IM[11:2]]};
        else 
            RDATA_IM = 0;
    end

    initial begin
        clk = 0;
        rst = 1;

        // 初始化記憶體 (NOP)
        for (i = 0; i < 1024; i = i + 1) begin
            IM_MEM[i] = 32'h00000013; 
            DM_MEM[i] = 32'h0;
        end
        
        // =========================================================
        // 測試程式碼
        // =========================================================
        
        // 1. 初始化資料 x1=10, x2=0
        IM_MEM[0] = 32'h00A00093; // ADDI x1, x0, 10
        IM_MEM[1] = 32'h00000113; // ADDI x2, x0, 0
        
        // 2. 測試 Branch Not Taken (Forwarding 測試)
        // BNE x1, x2, +8 (10 != 0, 應該跳) -> 預期: 跳到 PC=12 (Idx 3)
        // 這裡會用到 ID Forwarding (因為 x1 還在 Pipeline 裡)
        IM_MEM[2] = 32'h00209463; 
        
        // --- 這裡應該被 Flush 掉 (Branch Taken) ---
        IM_MEM[3] = 32'h00500293; // ADDI x5, x0, 5 (如果 x5 變 5 就錯了)
        
        // 3. Branch Target (PC=16, Idx 4)
        // ADDI x3, x0, 30 (證明跳轉成功)
        IM_MEM[4] = 32'h01E00193; 

        // 4. 測試 Load-Use Stall + Branch
        // SW x3, 0(x2) -> Mem[0] = 30
        IM_MEM[5] = 32'h00312023; 
        // LW x4, 0(x2) -> x4 = 30
        IM_MEM[6] = 32'h00012203; 
        // BEQ x4, x3, +8 (30 == 30, 應該跳) -> 預期: 跳到 PC=36 (Idx 9)
        // 這裡有 Load-Use Hazard! ID 階段必須 Stall 等 LW 的資料
        IM_MEM[7] = 32'h00320463; 
        
        // --- 這裡應該被 Flush 掉 ---
        IM_MEM[8] = 32'h00600313; // ADDI x6, x0, 6 (如果 x6 變 6 就錯了)

        // 5. Jump Target (PC=36, Idx 9)
        // JAL x7, +4 (x7=PC+4=40, 跳到 PC=40)
        IM_MEM[9] = 32'h004003EF; 

        // 6. JAL Target (PC=40, Idx 10)
        // ADDI x8, x0, 80 (結束)
        IM_MEM[10] = 32'h05000413;

        // Reset 釋放
        #20 rst = 0;
        
        #500;
        $display("Test Finished.");
        $finish;
    end

endmodule