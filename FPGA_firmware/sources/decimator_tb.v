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

module decimator_tb;
	localparam	total_data = 1000, clk_toggle_period = 1, clk_per_strobe = 64;

	reg rst;				// Reset (active high)
	reg clk;				// System clock
	reg strobe;				// Strobe signal for all channels
	wire output_strb;		// Strobe output
    integer counter;
	
    strobe_decimator #( .MAX_COUNTING(5) ) 
        de ( .clk(clk), .rst(rst), .in_strobe(strobe), .out_strobe(output_strb) );
	
	initial
	begin
        counter = 0;
		strobe = 0;
        clk = 0;
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
	always
	begin
		#clk_toggle_period clk = ~clk; 
	end
	
    reg t_count;
    initial
    begin
        t_count = 0;
    end
    always @(negedge clk)
    begin
        t_count = ~t_count;
    end
	// gen strobe
	always @(negedge t_count)
	begin
		strobe = ~strobe;
        counter = counter + 1;
        if(counter >= total_data)
            $stop;
	end
endmodule
