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
	localparam	total_data = 680, clk_toggle_period = 1, str_strobe = 2, end_strobe = 5;

	reg [63:0] in_data[0:total_data-1];		// Sampling data
	reg [63:0] d_in;		// Input data
	wire [63:0] d_out;		// Output data
	reg [15:0] level;		// Trigger level
	reg rst;				// Reset (active high)
	wire trigged;			// Trigger armed status
	reg clk;				// System clock
	reg strobe;				// Strobe signal for all channels
	wire output_strb;		// Strobe output
	integer strobe_count;

	wire trigger_rdy, trigger_fifo_rdy, trigger_strobe;
	wire [63:0] abs_data;
	wire [15:0] abs_trig;
	
	integer out_file = 0;
	reg [63:0] cycle_count;
	
	// Module under test
    // Concat all channels
    wire trigger_event;

    hydrophone_trigger_backlog #( .PRETRIG_SAMPLING(25), .POSTTRIG_SAMPLING(25) ) 
        trigger_backlog(
        .rst(rst),                      // system reset (active high)
        .clk(clk),                  // Master clock

        .rdy(trigger_rdy),              // Debug signal
        .fifo_rdy(trigger_fifo_rdy),    // Debug signal

        .d_in( d_in ),               // data input (concatenation of 4 16-bit data with channel 1 first)
        .input_strobe(strobe),       // Strobe from ADC
        .trigger_event(trigger_event),  // Event from trigger activation

        .d_out(d_out),            // data output
        .output_strobe(trigger_strobe), // Strobe to read from trigger FIFO
        .trigged(trigged)               // indicates that the data is part of packet of trigged signal
    );

    hydrophone_simple_trigger simple_trigger (
	    .abs_data(abs_data),
	    .abs_trig(abs_trig),

        .rst(rst),                      // system reset (active high)
        .clk(clk),                  // Master clock
		.enable(1),					// Enable trigger module
        .d_in( d_in ),               // data input (concatenation of 4 16-bit data with channel 1 first)
        .input_strobe(strobe),       // Strobe from ADC
	    .trigger_level(level),		// level of the trigger in 16-bit signed integer in format Q13.2
	    .trigged(trigger_event)			// indicates that the data is part of packet of trigged signal
    );

	initial
	begin
		$readmemh( "data.hex", in_data );
		out_file = $fopen("output.hex"); // open file
		$fmonitor(out_file, "%d : %016X,%b,%b,%b", cycle_count, d_out, output_strb, trigged, rst);
		//$monitor("%d : %016X,%b,%b", d_out, trigged, rst);
		level = 16'd655;
		strobe = 0;
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
	
	// Clk gen
	initial
	begin
	   clk = 0;
	end
	always
	begin
        #clk_toggle_period clk = ~clk;
	end
	
	// Apply sample data every clock edge
	always @(negedge clk)
	begin
		d_in = cycle_count;
	end

	// stop the simulation total_data and close the file
	// i.e. store only total_data values in file
	always @(negedge clk)
	begin
		if (cycle_count == total_data) 
		begin
			$fclose(out_file);  // close the file
			$stop;
		end
		else
		begin
			if(rst)
			begin
				cycle_count = 0;
				strobe_count = 0;
			end 
			else
			begin
				strobe_count = strobe_count + 1;
				if(strobe_count > 8)
				begin
					strobe_count = 0;
					cycle_count = cycle_count + 1;
				end

				if( strobe_count >= str_strobe && strobe_count <= end_strobe )
				begin
					strobe = 1;
				end
				else
				begin
				    strobe = 0;
				end
			end
		end
	end

	
endmodule