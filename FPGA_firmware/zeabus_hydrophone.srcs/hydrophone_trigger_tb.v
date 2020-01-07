`timescale 100ns / 100ns // time-unit = 1 ns, precision = 1 ps

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
	localparam	total_data = 10000, clk_toggle_period = 5;

	reg [63:0] in_data[0:total_data-1];		// Sampling data
	reg [63:0] d_in;		// Input data
	wire [63:0] d_out;		// Output data
	reg [15:0] level;		// Trigger level
	reg rst;				// Reset (active high)
	wire trigged;			// Trigger armed status
	reg clk;				// System clock
	
	integer out_file, cycle_count = 0;
	
	// Module under test
	hydrophone_trigger ht( .rst(rst), .clk(clk), .trigger_level(level), .d_out(d_out), .d_in(d_in), .trigged(trigged) );
	
	initial
	begin
		$readmemh( "data.hex", in_data );
		out_file = $fopen("output.hex"); // open file
		$fmonitor(out_file, "%d : %016X,%b,%b", cycle_count, d_out, trigged, rst);
		//$monitor("%d : %016X,%b,%b", d_out, trigged, rst);
		level = 16'd2000; // 0x07D0
		rst = 1; #clk_toggle_period rst = 0;
	end
	
	// Clk gen
	always
	begin
		clk = 1'b1; 
		#clk_toggle_period; // high for clk_toggle_period

		clk = 1'b0;
		#clk_toggle_period; // low for clk_toggle_period
	end
	
	// Apply sample data every clock edge
	always @(posedge clk)
	begin
		d_in = in_data[cycle_count];
	end

	// stop the simulation total_data and close the file
	// i.e. store only total_data values in file
	always @(posedge clk)
	begin
		if (cycle_count == total_data) 
		begin
			$fclose(out_file);  // close the file
			$stop;
		end
		else
		begin
			if(rst)
				cycle_count = 0;
			else
				cycle_count = cycle_count + 1;
		end
	end

	
endmodule