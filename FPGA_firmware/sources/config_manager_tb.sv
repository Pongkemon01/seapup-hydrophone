`timescale 1ns / 1ns

// --------------------------------------------------------------------------------
// Copyright 2019-2020 Akrapong Patchararungruang.
// --------------------------------------------------------------------------------
// This file is a part of Zeabus hydrophone subsystem. It was built for the Zeabus
// autonomous underwater vehicle (AUV). The AUV project is funded by the Faculty
// of Engineering, Kasetsart University.
// --------------------------------------------------------------------------------
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, 
//    this list of conditions and the following disclaimer in the documentation 
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors 
//    may be used to endorse or promote products derived from this software without 
//    specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// --------------------------------------------------------------------------------

/* Module to store and manage all settings such as trigger level and poten value.
 * It also parse the configuration setting data from host through FX3S slave FIFO.
 * The input data have the format as
 *  1 bytes: ID (fixed as 0xDC)
 *  1 bytes: Configuration fields
 *      The configuraton fields are Bit-fields indicates which configuration to set.
 *      This field also indicates what data would follow. Each bit has meaning as:
 *          bit 7 - 4: Pinger frequency index the list of frequency is shown below
 *          bit 3: 1 = enable trigger level setting
 *          bit 2: 1 = enable amplifier gains setting
 *          bit 1 - bit 0: (Reserved)
 *  2 bytes: New trigger level. (Exists only when the 3rd bit of the prefix is set)
 *  4 bytes: New amplifier gain for each channel. Each channel can have it gain different
 *      from others. (Thie field exists only when 2nd bit of the prefix is set)
 *
 * Pinger frequency table (for IQ demodulation)
 * +---------+-----------+
 * |  index  | frequency |
 * +---------+-----------+
 * | 4'b0000 |   25kHz   |
 * | 4'b0001 |   26kHz   |
 * | 4'b0010 |   27kHz   |
 * | 4'b0011 |   28kHz   |
 * | 4'b0100 |   29kHz   |
 * | 4'b0101 |   30kHz   |
 * | 4'b0110 |   31kHz   |
 * | 4'b0111 |   32kHz   |
 * | 4'b1000 |   33kHz   |
 * | 4'b1001 |   34kHz   |
 * | 4'b1010 |   35kHz   |
 * | 4'b1011 |   36kHz   |
 * | 4'b1100 |   37kHz   |
 * | 4'b1101 |   38kHz   |
 * | 4'b1110 |   39kHz   |
 * | 4'b1111 |   40kHz   |
 * +---------+-----------+
 */

 /* verilator lint_off UNUSEDSIGNAL */
 
module config_manager_tb;

	// ----------------------------------------------------------------
	// Parameters and Defines
	// ----------------------------------------------------------------
	localparam FIFO_SIZE_BITS = 4;  // SIZE_BIT_DEPTH for simple_fifo
	localparam DATA_BUS_WIDTH = 16; // Din width for config_manager
	localparam FIFO_DATA_WIDTH = DATA_BUS_WIDTH; 

	// ----------------------------------------------------------------
	// Signals (Wires and Regs)
	// ----------------------------------------------------------------
	logic clk;
	logic rst;

	// Signals for simple_fifo (as the driver/producer)
	logic wr_en;
	logic [FIFO_DATA_WIDTH-1:0] din_fifo;
	logic almost_full_fifo;
	logic full_fifo;
	logic [FIFO_SIZE_BITS-1:0] fifo_filled;

	// Signals for config_manager (as the consumer)
	logic rd_en; // Read enable for simple_fifo / data_valid for config_manager
	logic almost_empty_fifo;
	logic empty_n_fifo;
	logic [FIFO_DATA_WIDTH-1:0] dout_fifo; // din for config_manager
	
	// Output signals from config_manager (DUT outputs)
	logic update_poten;
	//logic config_d_oe;
	logic [3:0] pinger_freq;
	logic [15:0] trigger_level;
	logic [7:0] poten1_value;
	logic [7:0] poten2_value;
	logic [7:0] poten3_value;
	logic [7:0] poten4_value;
	
	// ----------------------------------------------------------------
	// Instances of DUTs (Design Under Test)
	// ----------------------------------------------------------------
	
	// Instantiate simple_fifo (เป็นตัวป้อนข้อมูล)
	simple_fifo #(
		.SIZE_BIT_DEPTH (FIFO_SIZE_BITS),
		.DATA_WIDTH (FIFO_DATA_WIDTH),
		.IS_FIRST_WORD_FALLTHROUGH (0)
	) uut_fifo (
		.clk (clk),
		.rst (rst),
		.fifo_filled (fifo_filled),
		.wr_en (wr_en),
		.din (din_fifo),
		.almost_full (almost_full_fifo),
		.full (full_fifo),
		.rd_en (rd_en),
		.almost_empty (almost_empty_fifo),
		.empty_n (empty_n_fifo),
		.dout (dout_fifo)
	);

	// Instantiate config_manager (เป็นตัวรับข้อมูล)
	config_manager #(
		.config_prefix (8'hDC),
		.rst_delay (2)
	) uut_config_manager (
		.clk (clk),
		.rst (rst),
		.update_poten (update_poten),
		.din (dout_fifo),
		.data_valid (empty_n_fifo), // FIFO not empty (data available) acts as data_valid
		.config_d_oe (rd_en),       // config_d_oe (read enable) connects to FIFO's rd_en
		.pinger_freq (pinger_freq),
		.trigger_level (trigger_level),
		.poten1_value (poten1_value),
		.poten2_value (poten2_value),
		.poten3_value (poten3_value),
		.poten4_value (poten4_value)
	);

	// ----------------------------------------------------------------
	// Clock Generation
	// ----------------------------------------------------------------
	initial begin
		clk = 1'b0;
		forever #1 clk = ~clk; // 1ns clock period (1GHz)
	end

	// ----------------------------------------------------------------
	// Stimulus Generation (การสร้างลำดับการป้อนข้อมูล)
	// ----------------------------------------------------------------
	initial begin
		$dumpfile("config_manager.vcd");
		$dumpvars(1);

		// 1. Initial Reset
		rst = 1'b1;
		wr_en = 1'b0;
		din_fifo = 16'h0000;
		rd_en = 1'b0; 
		#5 rst = 1'b0; // Active High Reset

		$display("Time=%0t: Start Test. Waiting for reset to finish...", $time);
		repeat (5) @(posedge clk); // Wait for reset_delay + a few cycles

		// ----------------------------------------------------------------
		// Test Case 1: Full Configuration Burst (ป้อนข้อมูลครบรวดเดียว)
		// Enable Trigger Level (bit 3) และ Amplifier Gains (bit 2)
		// Configuration data (5 cycles total: Prefix, Trig_L, Poten1_2, Poten3_4)
		// ----------------------------------------------------------------
		
		$display("------------------------------------------------");
		$display("Time=%0t: --- Test Case 1: Full Config (Burst) ---", $time);

		// Data 1: Prefix and fields (ID=0xDC, Fields=0b11xx_xxxx)
		// Pinger Freq Index = 4'b1001 (34kHz)
		// Enable Trigger (bit 3) = 1
		// Enable Poten (bit 2) = 1
		// Data: 16'hDC9C (0xDC for ID, 0xF4 for Fields: 1001_1100)
		din_fifo = 16'hDC9C; 
		wr_en = 1'b1;
		@(posedge clk); 
		wr_en = 1'b0;
		$display("Time=%0t: Wrote Prefix: 0x%H", $time, din_fifo);

		// Data 2: New Trigger Level (2 bytes)
		din_fifo = 16'd10000; // New trigger level
		wr_en = 1'b1;
		@(posedge clk); 
		wr_en = 1'b0;
		$display("Time=%0t: Wrote Trigger Level: %0d", $time, din_fifo);

		// Data 3: Potentiometer 1 and 2 (4 bytes, packed into 16 bits)
		// Poten 1 = 8'd200, Poten 2 = 8'd150
		din_fifo = {8'd200, 8'd150}; 
		wr_en = 1'b1;
		@(posedge clk); 
		wr_en = 1'b0;
		$display("Time=%0t: Wrote Poten 1&2: 0x%H", $time, din_fifo);

		// Data 4: Potentiometer 3 and 4 (4 bytes, packed into 16 bits)
		// Poten 3 = 8'd100, Poten 4 = 8'd50
		din_fifo = {8'd100, 8'd50}; 
		wr_en = 1'b1;
		@(posedge clk); 
		wr_en = 1'b0;
		$display("Time=%0t: Wrote Poten 3&4: 0x%H", $time, din_fifo);

		// Wait for config_manager to process the full configuration
		repeat (10) @(posedge clk); 
		
		$display("Time=%0t: Config Manager Outputs after Burst:", $time);
		$display("  pinger_freq: %0d (Expected: 9)", pinger_freq);
		$display("  trigger_level: %0d (Expected: 10000)", trigger_level);
		$display("  poten1_value: %0d (Expected: 200)", poten1_value);
		$display("  poten2_value: %0d (Expected: 150)", poten2_value);
		$display("  poten3_value: %0d (Expected: 100)", poten3_value);
		$display("  poten4_value: %0d (Expected: 50)", poten4_value);
		$display("  update_poten: %0b (Expected: 1)", update_poten);

		// ----------------------------------------------------------------
		// Test Case 2: Paced Configuration (ป้อนแบบเว้นช่วง)
		// Enable Trigger Level ONLY (bit 3)
		// Configuration data (3 cycles total: Prefix, Trig_L)
		// ----------------------------------------------------------------
		
		repeat (5) @(posedge clk); // Delay before next test
		$display("------------------------------------------------");
		$display("Time=%0t: --- Test Case 2: Trigger Only (Paced) ---", $time);

		// Data 1: Prefix and fields (ID=0xDC, Fields=0b0001_xxxx)
		// Pinger Freq Index = 4'b0010 (27kHz)
		// Enable Trigger (bit 3) = 1
		// Enable Poten (bit 2) = 0
		// Data: 16'hDC28 (0xDC for ID, 0x28 for Fields: 0010_1000)
		din_fifo = 16'hDC28; 
		wr_en = 1'b1;
		@(posedge clk); 
		wr_en = 1'b0;
		$display("Time=%0t: Wrote Prefix: 0x%H", $time, din_fifo);

		repeat (5) @(posedge clk); // Pause (config_manager should wait)
		
		// Data 2: New Trigger Level (2 bytes)
		din_fifo = 16'd5000; 
		wr_en = 1'b1;
		@(posedge clk); 
		wr_en = 1'b0;
		$display("Time=%0t: Wrote Trigger Level: %0d", $time, din_fifo);

		repeat (10) @(posedge clk); // Wait for config_manager to finish

		$display("Time=%0t: Config Manager Outputs after Paced Test:", $time);
		$display("  pinger_freq: %0d (Expected: 2)", pinger_freq);
		$display("  trigger_level: %0d (Expected: 5000)", trigger_level);
		$display("  poten1_value: %0d (Expected: 200 - Unchanged)", poten1_value);
		$display("  poten2_value: %0d (Expected: 150 - Unchanged)", poten2_value);
		$display("  poten3_value: %0d (Expected: 100 - Unchanged)", poten3_value);
		$display("  poten4_value: %0d (Expected: 50 - Unchanged)", poten4_value);
		$display("  update_poten: %0b (Expected: 0)", update_poten);


		// ----------------------------------------------------------------
		// Test Case 3: Invalid Prefix
		// ----------------------------------------------------------------

		repeat (10) @(posedge clk); // Delay
		$display("------------------------------------------------");
		$display("Time=%0t: --- Test Case 3: Invalid Prefix ---", $time);

		// Invalid Prefix: 16'hABCD (Should be 16'hDCxx)
		din_fifo = 16'hABCD; 
		wr_en = 1'b1;
		@(posedge clk); 
		wr_en = 1'b0;
		$display("Time=%0t: Wrote Invalid Prefix: 0x%H", $time, din_fifo);

		repeat (5) @(posedge clk); // Wait for config_manager to reject and return to WAIT_PREFIX

		$display("Time=%0t: Config Manager Outputs after Invalid Prefix:", $time);
		$display("  pinger_freq: %0d (Expected: 2 - Unchanged)", pinger_freq);
		$display("  trigger_level: %0d (Expected: 5000 - Unchanged)", trigger_level);


		// ----------------------------------------------------------------
		// End Simulation
		// ----------------------------------------------------------------

		@(posedge clk);
		$display("------------------------------------------------");
		$display("Time=%0t: Simulation finished.", $time);
		$finish;
	end

endmodule
