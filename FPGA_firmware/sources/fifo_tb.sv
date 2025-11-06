`timescale 1ns / 1ns
/* verilator lint_off DECLFILENAME */
/* verilator lint_off UNOPTFLAT */
/* verilator lint_off SYNCASYNCNET */

module simple_fifo_tb;

    // 1. กำหนด Parameters ตามโจทย์
    localparam SIZE_BIT_DEPTH = 3;
    localparam DATA_WIDTH = 8;
    localparam FIFO_DEPTH = (1<<SIZE_BIT_DEPTH) - 1; 

    // 2. สัญญาณสำหรับ Test Bench
    logic clk;
    logic rst;

    // Write Interface
    logic wr_en;
    logic [DATA_WIDTH-1:0] din;
    logic almost_full;
    logic full;

    // Read Interface
    logic rd_en;
    logic almost_empty;
    logic empty_n;
    logic [DATA_WIDTH-1:0] dout;
    
    // Fill Count
    logic [SIZE_BIT_DEPTH-1:0] fifo_filled;

    // ตัวแปรสำหรับเก็บข้อมูลที่คาดหวัง (Reference Model: Queue)
    logic [DATA_WIDTH-1:0] expected_data_q[$];
    logic [DATA_WIDTH-1:0] data_in_val = 0;
    integer errors = 0;

    // 3. Instantiate DUT (Device Under Test)
    simple_fifo
    #(.SIZE_BIT_DEPTH(SIZE_BIT_DEPTH), .DATA_WIDTH(DATA_WIDTH), .IS_FIRST_WORD_FALLTHROUGH(1))
    dut
    (
        .clk(clk),
        .rst(rst),
        .fifo_filled(fifo_filled),
        .wr_en(wr_en),
        .din(din),
        .almost_full(almost_full),
        .full(full),
        .rd_en(rd_en),
        .almost_empty(almost_empty),
        .empty_n(empty_n),
        .dout(dout)
    );

    // 4. Clock Generation
    initial begin
        clk = 0;
        forever #1 clk = ~clk; // Clock period = 10ns (100MHz)
    end
    
    // 5. Task สำหรับการทำงานพื้นฐาน
    task automatic reset_dut;
        $display("-------------------------------------------");
        $display(">> Starting Reset...");
        rst = 1;
        wr_en = 0;
        rd_en = 0;
        din = 'z;
        repeat(5) @(posedge clk);
        rst = 0;
        data_in_val = 0;
        expected_data_q.delete(); // Clear reference queue
        $display(">> Reset Complete.");
        $display("-------------------------------------------");
    endtask

    task automatic write_data(input int count);
        $display(">> Starting Write %0d item(s).", count);
        repeat(count) begin
            if (full) begin
                $display("!!! WARNING: Write rejected at $time, FIFO is FULL.");
            end else begin
                data_in_val++;
                wr_en = 1;
                din = data_in_val;
                expected_data_q.push_back(din); // Add to reference queue
                $display("@%0t: WRITE din = %0X (Filled: %0d)", $time, din, fifo_filled + 1);

            end
            @(posedge clk);
            wr_en = 0;
        end
    endtask

    task automatic read_data(input int count);
        logic [DATA_WIDTH-1:0] expected_val;
        $display(">> Starting Read %0d item(s).", count);
        repeat(count) begin
           if (!empty_n) begin
                $display("!!! WARNING: Read attempt failed at $time, FIFO is EMPTY.");
            end else begin
                rd_en = 1;
                expected_val = expected_data_q.pop_front(); // Get from reference queue
                @(posedge clk);
                
                // Verification Check
                if (dout !== expected_val) begin
                    $display("!!! ERROR @%0t: READ MISMATCH! Got %0X, Expected %0X (Filled: %0d)", 
                             $time, dout, expected_val, fifo_filled - 1);
                    errors++;
                end else begin
                    $display("@%0t: READ dout = %0X (Filled: %0d)", $time, dout, fifo_filled - 1);
                end
            end
            rd_en = 0;
        end
    endtask
    
    // Task สำหรับ Write/Read พร้อมกัน
    task automatic simultaneous_op(input int w_count, input int r_count);
        logic [DATA_WIDTH-1:0] expected_val; 
        $display(">> Starting Simultaneous Write/Read (W=%0d, R=%0d)", w_count, r_count);
        // กำหนดให้ w_count เป็นตัวกำหนดรอบในการทำซ้ำ เพื่อให้เห็นผลกระทบของสัญญาณควบคุม
        repeat(w_count) begin 
            // Write Logic
            data_in_val++;
            wr_en = 1;
            din = data_in_val;
            expected_data_q.push_back(din);
            $display("@%0t: WRITE din = %0X", $time, din);
            
            // Read Logic
            if (r_count > 0) begin // อ่านได้ถ้าไม่ว่างและยังเหลือรอบ
                rd_en = 1;
                r_count--; // ลดจำนวนการอ่านที่เหลือ
                @(posedge clk);
                
                // Verification Check
                expected_val = expected_data_q.pop_front();
                if (dout !== expected_val) begin
                    $display("!!! ERROR @%0t: SIMULTANEOUS READ MISMATCH! Got %0X, Expected %0X", 
                             $time, dout, expected_val);
                    errors++;
                end else begin
                    $display("@%0t: READ dout = %0X", $time, dout);
                end
            end else begin
                $display("@%0t: Read Blocked (R_COUNT=0)", $time);
                rd_en = 0;
            end
        
            // เคลียร์สัญญาณควบคุมในรอบถัดไป
            wr_en = 0;
            rd_en = 0;
        end
    endtask


    // 6. Test Scenario (Initial Block)
    initial begin
        $display("=================================================");
        $display("== simple_fifo Test Bench (Depth: %0d, Width: %0d) ==", FIFO_DEPTH, DATA_WIDTH);
        $display("=================================================");
        
        // --- Scenario 1: Reset State ---
        reset_dut;
        assert(fifo_filled == 0 && !empty_n && almost_empty && !full && !almost_full) 
            $info("TEST 1: Initial state after reset is correct."); 
        else $error("TEST 1: Initial state after reset FAILED.");

        // --- Scenario 2: Normal Write & Read ---
        $display("--- Scenario 2: Normal Write & Read (1 item) ---");
        write_data(1);
        assert(fifo_filled == 1 && empty_n && almost_empty && !full && !almost_full) 
            $info("TEST 2: State after 1 Write is correct."); 
        else $error("TEST 2: State after 1 Write FAILED.");
        read_data(1);
        assert(fifo_filled == 0 && !empty_n && almost_empty && !full && !almost_full) 
            $info("TEST 2: State after 1 Read is correct."); 
        else $error("TEST 2: State after 1 Read FAILED.");
        
        // --- Scenario 3: Fill FIFO to FULL and Almost Full ---
        $display("--- Scenario 3: Fill FIFO to FULL (4 items) ---");
        write_data(FIFO_DEPTH); // Write until full
        assert(fifo_filled == FIFO_DEPTH && empty_n && !almost_empty && full && almost_full) 
            $info("TEST 3: Full state is correct."); 
        else $error("TEST 3: Full state FAILED.");
            
        // --- Scenario 4: Write on FULL (Overflow attempt) ---
        $display("--- Scenario 4: Write on FULL (Should be rejected) ---");
        wr_en = 1; din = 99; // Attempt to write rejected data
        @(posedge clk);
        assert(fifo_filled == FIFO_DEPTH && full) 
            $info("TEST 4: Write on FULL is rejected."); 
        else $error("TEST 4: Write on FULL FAILED (Pointer advanced).");
        wr_en = 0;
        
        // --- Scenario 5: Empty FIFO to EMPTY and Almost Empty ---
        $display("--- Scenario 5: Empty FIFO (3 reads) ---");
        read_data(FIFO_DEPTH - 2); // Read until 2 items left
        assert(fifo_filled == 2 && empty_n && !almost_empty && !full && !almost_full) 
            $info("TEST 5a: State (Filled=2) is correct."); 
        else $error("TEST 5a: State (Filled=2) FAILED.");
        read_data(1); // Read 1 item (2) -> Filled=1 (Almost Empty)
        assert(fifo_filled == 1 && empty_n && almost_empty && !full && !almost_full) 
            $info("TEST 5b: Almost Empty state is correct."); 
        else $error("TEST 5b: Almost Empty state FAILED.");
        read_data(1); // Read 1 item (3) -> Filled=0 (Empty)
        assert(fifo_filled == 0 && !empty_n && almost_empty && !full && !almost_full) 
            $info("TEST 5c: Empty state is correct."); 
        else $error("TEST 5c: Empty state FAILED.");
            
        // --- Scenario 6: Read on EMPTY (Underflow attempt) ---
        $display("--- Scenario 6: Read on EMPTY (Should be ignored) ---");
        rd_en = 1;
        @(posedge clk);
        assert(fifo_filled == 0 && !empty_n) 
            $info("TEST 6: Read on EMPTY is ignored."); 
        else $error("TEST 6: Read on EMPTY FAILED (Pointer advanced).");
        rd_en = 0;
        
        // --- Scenario 7: Simultaneous Write & Read (Normal) ---
        $display("--- Scenario 7: Simultaneous Write & Read (Filled=0 -> Filled=0) ---");
        // Write 2 items, Read 2 items simultaneously (W/R on 2 cycles)
        simultaneous_op(2, 2); 
        assert(fifo_filled == 0 && !empty_n) 
            $info("TEST 7: Simultaneous W/R (Filled=0 -> 0) is correct."); 
        else $error("TEST 7: Simultaneous W/R (Filled=0 -> 0) FAILED.");
        
        // --- Scenario 8: Simultaneous Write & Read (FULL) ---
        $display("--- Scenario 8: Simultaneous Write & Read (FULL -> FULL) ---");
        write_data(FIFO_DEPTH); // Fill it up again (4 items)
        simultaneous_op(FIFO_DEPTH, FIFO_DEPTH); // W=4, R=4 (Should remain full on W cycles, then empty)
        assert(fifo_filled == FIFO_DEPTH && full) 
            $info("TEST 8: Simultaneous W/R (FULL -> FULL) is correct."); 
        else $error("TEST 8: Simultaneous W/R (FULL -> FULL) FAILED.");
        read_data(FIFO_DEPTH); // Empty it out

        // --- Scenario 9: Mixed Operations (W=3, R=1, W=2, R=4) ---
        $display("--- Scenario 9: Mixed Operations ---");
        write_data(FIFO_DEPTH - 1); // Filled to almost full
        read_data(1);  // Filled=2
        write_data(2); // Filled=4 (FULL)
        read_data(FIFO_DEPTH);  // Filled=0 (EMPTY)
        assert(fifo_filled == 0 && !empty_n) 
            $info("TEST 9: Mixed Operations are correct."); 
        else $error("TEST 9: Mixed Operations FAILED.");

        @(posedge clk);
        
        $display("-------------------------------------------");
        if (errors == 0) begin
            $display(">> Test Bench FINISHED successfully. No errors found.");
        end else begin
            $display(">> Test Bench FINISHED with %0d ERROR(s).", errors);
        end
        $display("-------------------------------------------");
        $stop; // Stop simulation
    end

    // 7. Optional: Monitor signals for easy debugging
    // initial begin
    //     $monitor("@%0t | clk:%0b rst:%0b | W:%0b D_in:%0X F:%0b AF:%0b | R:%0b D_out:%0X E_n:%0b AE:%0b | Filled:%0d | Q_size:%0d", 
    //              $time, clk, rst, wr_en, din, full, almost_full, rd_en, dout, empty_n, almost_empty, fifo_filled, expected_data_q.size());
    // end

    // 8. Optional: Generate VCD file for waveform viewing
	initial begin
   		$dumpfile("simple_fifo.vcd");
   		$dumpvars(1);
	end
endmodule
