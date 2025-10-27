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

/* verilator lint_off DECLFILENAME */

// Helper module to get absolute value
module absolute( input logic [15:0] in, output logic [15:0] out );
	assign out = in[15] ? -in : in;
endmodule

// Backlog buffer
module backlog_buffer
#(
	parameter SIZE_BIT_DEPTH = 6,	// Bit depth of the size of the buffer (2^SIZE_BIT_DEPTH)
	parameter DATA_WIDTH = 64		// Width of the data (in bits)
) (
	input logic clk,					// signal clock (64 MHz)
	input logic rst,					// system reset (active high)
	input logic wr_en,					// write enable
	input logic rd_en,					// read enable
	input logic [DATA_WIDTH-1:0] din,	// data input

	output logic [DATA_WIDTH-1:0] dout	// data output
);
	logic [DATA_WIDTH-1:0] mem [(2**SIZE_BIT_DEPTH)-1:0];	// Memory array
	logic [SIZE_BIT_DEPTH-1:0] ptr;							// Write and read pointers

	// Write operation
	always_ff @(posedge clk) begin
		if( rst ) begin
			ptr <= 0;
		end
		else if( wr_en ) begin
			mem[ptr] <= din;
			ptr <= ptr + 1;
		end
	end

	// Read operation
	assign dout = (rst || !rd_en) ? 0 : mem[ptr];
endmodule

// Main trigger module
// Outut data is valid at the rising edge of the next clock after the "trigged" signal
module hydrophone_trigger_fifo
#(
	parameter HEAD_TAIL_BIT_DEPTH = 6	// Bit-depth of the header and tail of d_in samples before a trigged points and after a trigged period
) (
	input logic clk,				// signal clock (64 MHz)
	input logic rst,				// system reset (active high)
	input logic [63:0] din,			// data input (concatenation of 4 16-bit data) all in format Q13.2
	input logic din_strobe,			// Combined strobe signal from all channels
	input logic trigger_event,		// Trigger condition met
	output logic [63:0] dout,		// data output  all in format Q13.2
	output logic dout_strobe,		// Combined strobe signal from all channels plus edge detected
	output logic trigged			// indicates that the data is part of packet of trigged signal
);
	// Constants
	// Number of samples before the trigger period
	localparam max_h_counter = (2**HEAD_TAIL_BIT_DEPTH) - 1;
	// Number of samples after the trigger period.  It should be equal to twice of head samples.
	localparam max_t_counter = (2**(HEAD_TAIL_BIT_DEPTH + 1)) - 1;

	// Variables
	logic [HEAD_TAIL_BIT_DEPTH-1:0] h_counter;	// Fifo backlog data counter
	logic [HEAD_TAIL_BIT_DEPTH:0] t_counter;	// Counter for packet tailing
	logic output_enable;						// Indicates that the output is enabled
	logic strb_d;								// Delay line of strobe signal to detect rising edge
	logic data_strobe;							// Combined strobe signal with edge detection
	
	// Generate combined strobe signals
	assign data_strobe = din_strobe & ~strb_d & ~rst;
	assign dout_strobe = output_enable & data_strobe;

	// Delay line for edge detection
	always_ff @(posedge clk) begin
		if( rst )begin
			strb_d <= 0;
		end
		else begin
			strb_d <= din_strobe;
		end
	end

	// Main state machine
	always_ff @(posedge clk) begin
		if( rst ) begin
			// Reset signal asserted. Just initialize state
			h_counter <= max_h_counter;
			t_counter <= 0;
			trigged <= 1'b0;
			output_enable <= 1'b0;
		end
		else begin
			if( data_strobe ) begin // clk freq. is higher than data rate, so we use the Strobe signal indicates each datum
				// After reset, we need to fill the FIFO first
				if( h_counter != 0 ) begin
					h_counter <= h_counter - 1;
					output_enable <= 1'b0;	// Disable FIFO reading until having enough backlog
				end
				else begin
					output_enable <= 1'b1;		// Enable FIFO reading after having enough backlog
					if( trigger_event ) begin
					   // Trigged
					   trigged <= 1;
					   t_counter <= max_t_counter;	// Reset the tail counter
					end
					else begin
						if( t_counter == 0 ) begin
							trigged <= 0;
						end
						else begin
							// Still retain the "trigged" condition
							t_counter <= t_counter - 1;
						end
					end
				end
			end
		end
	end

	// FIFO instance
	backlog_buffer #(
		.SIZE_BIT_DEPTH(HEAD_TAIL_BIT_DEPTH),
		.DATA_WIDTH(64)
	) fifo_inst (
		.clk(clk),
		.rst(rst),
		.wr_en(data_strobe),
		.rd_en(output_enable),
		.din(din),
		.dout(dout)
	);

endmodule

// Simple hydrophone trigger mode
module hydrophone_simple_trigger (
//	input logic clk,					// signal clock (64 MHz)
	input logic rst,					// system reset (active high)
	input logic enable,					// enable trigger funtion (aka. start of the capture function)
	input logic [63:0] din,				// data input (concatenation of 4 16-bit data) all in format Q13.6
	input logic din_strobe,				// Combined strobe signal from all channels
	input logic [15:0] trigger_level,	// level of the trigger in 16-bit signed integer in format Q13.2
	output logic trigged				// indicates that the data is part of packet of trigged signal
);

	// Variables
	logic [63:0] abs_d_in;				// Magnetude (aka. absolute) values of d_in
	logic [15:0] abs_trigger;			// Magnetude of trigger level

	// Absolute implementation
	absolute abs1( .in(din[15:0]), .out(abs_d_in[15:0]) );
	absolute abs2( .in(din[31:16]), .out(abs_d_in[31:16]) );
	absolute abs3( .in(din[47:32]), .out(abs_d_in[47:32]) );
	absolute abs4( .in(din[63:48]), .out(abs_d_in[63:48]) );
	absolute abs5( .in(trigger_level), .out(abs_trigger) );
	
	// Generate trigged signal
	always_comb begin
		if( rst ) begin
			trigged = 0;
		end
		else begin
			if( enable && din_strobe && ( ( abs_d_in[15:0] >= abs_trigger ) ||
				( abs_d_in[31:16] >= abs_trigger ) ||
				( abs_d_in[47:32] >= abs_trigger ) ||
				( abs_d_in[63:48] >= abs_trigger ) )
			) begin
				// Trigged
				trigged = 1;
			end
			else begin
				trigged = 0;
			end
		end
	end
endmodule
