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
/* verilator lint_off UNUSEDSIGNAL */
module packetizer_tb;
	localparam	total_data = 10000, clk_toggle_period = 1, clk_per_strobe = 64;

	logic [63:0] in_data[0:total_data-1];		// Sampling data
	logic [63:0] d_in;		// Input data
	logic [63:0] t_d_out;	// Output from trigger
	logic [15:0] d_out;		// Output data
	logic [15:0] latched_output; // Latched output data to simulate output buffer
	logic [15:0] level;		// Trigger level
	logic rst;				// Reset (active high)
	logic trigged;			// Trigger armed status
	bit clk;				// System clock
	logic input_strobe;				// Strobe signal for all channels
	logic output_strb;		// Strobe output
	integer strobe_count;
	logic sending, packetize_strobe, pkt_end;
	logic fifo_rd_en, empty_n, almost_empty, full, almost_full;
	logic [16:0] fifo_dout;
	
	// Debug output
	//logic [4:0]debug_main_state;
	//wire [1:0]debug_sub_state;
	//wire debug_strobe_d;
	
	integer out_file, cycle_count;
	
	// Module under test
    logic trigger_event;

	always_ff @(posedge clk) begin
		if(rst) begin
			latched_output <= 16'h0000;
		end
		else begin
			if( packetize_strobe ) begin
				latched_output <= d_out;
			end
		end
	end

    hydrophone_trigger_fifo #( .HEAD_TAIL_BIT_DEPTH(4) ) 
        trigger_backlog(
        .rst(rst),                      // system reset (active high)
        .clk(clk),                  // Master clock

        .din( d_in ),               // data input (concatenation of 4 16-bit data with channel 1 first)
        .din_strobe(input_strobe),       // Strobe from ADC
        .trigger_event(trigger_event),  // Event from trigger activation

        .dout(t_d_out),            // data output
        .dout_strobe(output_strb), // Strobe to read from trigger FIFO
        .trigged(trigged)               // indicates that the data is part of packet of trigged signal
    );

    hydrophone_simple_trigger simple_trigger (
        .rst(rst),                      // system reset (active high)
		.enable(1),					// Enable trigger module
        .din( d_in ),               // data input (concatenation of 4 16-bit data with channel 1 first)
        .din_strobe(input_strobe),       // Strobe from ADC
	    .trigger_level(level),		// level of the trigger in 16-bit signed integer in format Q13.2
	    .trigged(trigger_event)			// indicates that the data is part of packet of trigged signal
    );

	packetizer pt(
		// Debug ports
		//.debug_main_state(debug_main_state),
		//.debug_sub_state( debug_sub_state),
		//.debug_strb_d( debug_strobe_d),
		// Input ports
		.din( t_d_out),			// Data input from each channel
		.trigged(trigged),			// Indicates that the system can detect valid data
		.din_strobe(output_strb),	// Clock to latch the input data (at logic 1 + posedge clk)
		
		// Output ports
		.dout(d_out),	// Output data
		.sending(sending),		// 0 = idle, 1 = sending a packet
		.dout_strobe(packetize_strobe),	// Clock to latch the output data (at each posedge)
		.pkt_end(pkt_end),		// 0 = nothing, 1 = current data is the last word
		
		// Control ports
		.rst(rst),				// System reset (active high)
		.clk(clk),			// System clock

		.out_full(almost_full)			// Output buffer is almost full
    );

	logic [10:0] fifo_filled;
	// Output FIFO
	simple_fifo  #(
		.SIZE_BIT_DEPTH(11),
		.DATA_WIDTH(17),
		.IS_FIRST_WORD_FALLTHROUGH(0)
	) output_fifo (
		.clk(clk),					// signal clock (64 MHz)
		.rst(rst),					// system reset (active high)

		// FIFO fill count (number of elements in the FIFO)
		.fifo_filled(fifo_filled),	

		// Write interface
		.wr_en(packetize_strobe),		// write enable
		.din({pkt_end, d_out}),			// data input
		.almost_full(almost_full),			// fifo almost full indicator, indicating that only one space is left or full
		.full(full),			   			// fifo full indicator

		// Read interface
		.rd_en(fifo_rd_en),					// read enable
		.almost_empty(almost_empty),				// fifo almost empty indicator, indicating that only one data is left or empty
		.empty_n(empty_n),			   			// fifo empty indicator (0 = empty, 1 = not empty)
		.dout(fifo_dout)							// data output
	);
	
	initial begin
		$readmemh( "data.hex", in_data );
		out_file = $fopen("output.hex"); // open file
		//$fmonitor(out_file, "%d : %016X,%b,%b,%b", cycle_count, d_out, packetize_strobe, trigged, rst);
		//$monitor("%d : %016X,%b,%b", d_out, trigged, rst);
		level = 16'h28f; // 0x07D0
		strobe_count = 0;
		rst = 1; 
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 1;
		#clk_toggle_period rst = 0;
	end
	
	final begin
		$fclose(out_file);  // close the file
	end
	
	// Generate output waveform
	always_ff @(posedge clk)begin
		if( rst ) begin
			fifo_rd_en <= 0;
		end
		else if( empty_n ) begin
			fifo_rd_en <= 1;
		end
		else begin
			fifo_rd_en <= 0;
		end
	end

	initial begin
   		$dumpfile("packetizer.vcd");
   		$dumpvars(1);
	end
	always_ff @(posedge clk) begin
		if (!rst) begin
			$fdisplay(out_file, "%d : %016X,%b,%b,%b", cycle_count, d_out, packetize_strobe, trigged, rst);
		end
	end
	// Clk gen
	always begin
        #clk_toggle_period clk = ~clk;
	end
	
	// Apply sample data every clock edge
	always_ff @(posedge clk) begin
		d_in <= in_data[cycle_count];
	end

	always_comb begin
		if (strobe_count == clk_per_strobe) begin
			input_strobe = 1;
		end
		else begin
			input_strobe = 0;
		end
	end

	// stop the simulation total_data and close the file
	// i.e. store only total_data values in file
	always_ff @(posedge clk) begin
		if (cycle_count == total_data) 
		begin
			$stop;
		end
		else
		begin
			if(rst) begin
				cycle_count <= 0;
				strobe_count <= 0;
			end 
			else begin
				strobe_count <= strobe_count + 1;
				if( strobe_count == clk_per_strobe ) begin
					strobe_count <= 0;
					cycle_count <= cycle_count + 1;
				end
			end
		end
	end
endmodule
