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

module hydrophone_trigger_w_filter_tb;
	logic [63:0] d_in;		// Input data
	logic [63:0] d_out;		// Output data
	logic [15:0] level;		// Trigger level
	logic rst;				// Reset (active high)
	logic trigged;			// Trigger armed status
	bit clk;				// System clock
	logic input_strobe;		// Strobe signal for all channels
	logic output_strb;		// Strobe output
	
	integer out_file = 0;
	logic [63:0] cycle_count;

    logic [15:0] adc1_1_out;
    logic [15:0] adc1_2_out;
    logic [15:0] adc2_1_out;
    logic [15:0] adc2_2_out;                     // Data output from ADC modules in Q13.2 format
    logic adc_strb_1, adc_strb_2, adc_strb_3, adc_strb_4;    // Strobe signal from ADC interfaces

	// Module under test
    // Concat all channels
    logic trigger_event;

	assign input_strobe = adc_strb_1 & adc_strb_2 & adc_strb_3 & adc_strb_4;
    assign d_in = { adc2_2_out, adc2_1_out, adc1_2_out, adc1_1_out };
    //
    // 3 stages pipeline
    //
    adc_interface adc1(
        // Interface to hardware
        .d_in(0),                     // Data channel from ADC chip
        .overflow(0),               // overflow flag from ADC chip

        // Control signals
        .clk(clk),                  // System clock.
        .rst(rst),                      // Synchronous reset (active high)

        // Output data
        .d0_out(adc1_1_out),            // Output from each ADC channel
        .strobe_0(adc_strb_1),
        .d1_out(adc1_2_out),
        .strobe_1(adc_strb_2)
    );

    //
    // 3 stages pipeline
    //
    adc_interface adc2(
        // Interface to hardware
        .d_in(0),                     // Data channel from ADC chip
        .overflow(0),               // overflow flag from ADC chip

        // Control signals
        .clk(clk),                  // System clock.
        .rst(rst),                      // Synchronous reset (active high)

        // Output data
        .d0_out(adc2_1_out),            // Output from each ADC channel
        .strobe_0(adc_strb_3),
        .d1_out(adc2_2_out),
        .strobe_1(adc_strb_4)
    );

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
		out_file = $fopen("output.hex"); // open file
		//$fmonitor(out_file, "%d : %016X,%b,%b,%b", cycle_count, d_out, output_strb, trigged, rst);
		//$monitor("%d : %016X,%b,%b", d_out, trigged, rst);
		level = 16'd655;
		cycle_count = 0;
		rst = 1; 
		#12 rst = 0;
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
        #1 clk = ~clk;
	end

	always_ff @(posedge clk) begin
		if (!rst) begin
			$fdisplay(out_file, "%d : %016X,%b,%b,%b", cycle_count, d_out, output_strb, trigged, rst);
		end
	end
	
	// stop the simulation total_data and close the file
	// i.e. store only total_data values in file
	always_ff @(posedge clk) begin
		if (cycle_count == 100000) begin
			$stop;
		end
		else begin
			if(rst) begin
				cycle_count <= 0;
			end 
			else begin
				if(output_strb) begin
					cycle_count <= cycle_count + 1;
				end
			end
		end
	end

	
endmodule
