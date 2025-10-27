`timescale 1ns / 1ns // time-unit = 1 ns, precision = 1 ns

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

module hydrophone_trigger_tb;
	localparam	total_data = 10000, clk_toggle_period = 1, str_strobe = 3;

	logic [63:0] in_data[0:total_data-1];		// Sampling data
	logic [63:0] d_in;		// Input data
	logic [63:0] d_out;		// Output data
	logic [15:0] level;		// Trigger level
	logic rst;				// Reset (active high)
	logic trigged;			// Trigger armed status
	bit clk;				// System clock
	logic input_strobe;				// Strobe signal for all channels
	logic output_strb;		// Strobe output
	integer strobe_count;
	
	integer out_file = 0;
	logic [63:0] cycle_count;
	
	// Module under test
    // Concat all channels
    logic trigger_event;

    hydrophone_trigger_fifo #( .HEAD_TAIL_BIT_DEPTH(4) ) 
        trigger_backlog(
        .rst(rst),                      // system reset (active high)
        .clk(clk),                  // Master clock

        .din( d_in ),               // data input (concatenation of 4 16-bit data with channel 1 first)
        .din_strobe(input_strobe),       // Strobe from ADC
        .trigger_event(trigger_event),  // Event from trigger activation

        .dout(d_out),            // data output
        .dout_strobe(output_strb), // Strobe to read from trigger FIFO
        .trigged(trigged)               // indicates that the data is part of packet of trigged signal
    );

    hydrophone_simple_trigger simple_trigger (
        .rst(rst),                      // system reset (active high)
    //    .clk(clk),                  // Master clock
		.enable(1),					// Enable trigger module
        .din( d_in ),               // data input (concatenation of 4 16-bit data with channel 1 first)
        .din_strobe(input_strobe),       // Strobe from ADC
	    .trigger_level(level),		// level of the trigger in 16-bit signed integer in format Q13.2
	    .trigged(trigger_event)			// indicates that the data is part of packet of trigged signal
    );

	initial begin
		$readmemh( "data.hex", in_data );
		out_file = $fopen("output.hex"); // open file
		//$fmonitor(out_file, "%d : %016X,%b,%b,%b", cycle_count, d_out, output_strb, trigged, rst);
		//$monitor("%d : %016X,%b,%b", d_out, trigged, rst);
		level = 16'd655;
		strobe_count = 0;
		cycle_count = 0;
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
	initial begin
   		$dumpfile("test.vcd");
   		$dumpvars(1);
	end

	// Clk gen
	always begin
        #clk_toggle_period clk = ~clk;
	end

	always_ff @(posedge clk) begin
		if (!rst) begin
			$fdisplay(out_file, "%d : %016X,%b,%b,%b", cycle_count, d_out, output_strb, trigged, rst);
		end
	end
	
	// Apply sample data every clock edge
	/* verilator lint_off WIDTHTRUNC */
	always_ff @(posedge clk) begin
		//d_in <= cycle_count;
		d_in <= in_data[cycle_count];
	end

	always_comb begin
		if (strobe_count > str_strobe) begin
			input_strobe = 1;
		end
		else begin
			input_strobe = 0;
		end
	end

	// stop the simulation total_data and close the file
	// i.e. store only total_data values in file
	always_ff @(posedge clk) begin
		if (cycle_count == total_data) begin
			$stop;
		end
		else begin
			if(rst) begin
				cycle_count <= 1;
				strobe_count <= 0;
			end 
			else begin
				strobe_count <= strobe_count + 1;
				if(strobe_count >= 15) begin
					strobe_count <= 0;
					cycle_count <= cycle_count + 1;
				end
			end
		end
	end

	
endmodule
