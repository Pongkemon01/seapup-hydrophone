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

// This module selectively enables some data strobe to lower effective sampling rate
module strobe_decimator #( MAX_COUNTING = 100 ) (
    input clk,          // System clock
    input rst,          // Reset (active high)
    input in_strobe,
    output out_strobe
);
    localparam MAX_COUNT_MINUS_1 = MAX_COUNTING - 1;

    reg [15:0] counter;
    wire stable_strobe;
    reg strobe_activate;
	reg strb_d, strb_dd;		// Delay line of strobe signal to detect rising edge

    assign stable_strobe = strb_d & ~strb_dd;
    assign out_strobe = in_strobe & strobe_activate;

    initial
    begin
        strb_d <= 0;
        strb_dd <= 0;
        counter <= 0;
        strobe_activate <= 0;
    end

	// Delay line for edge detection
	always @(negedge clk)
	begin
		if( rst )
		begin
			strb_d <= 0;
			strb_dd <= 0;
		end
		else
		begin
			strb_dd <= strb_d;
			strb_d <= in_strobe;
		end
	end

    always @(negedge clk)
    begin
        if( rst )
        begin
            counter <= 0;
            strobe_activate <= 0;
        end
        else
        begin
            if( stable_strobe )
            begin
                if( counter == 0 )
                begin
                    strobe_activate <= 1;
                    counter <= MAX_COUNT_MINUS_1;
                end
                else
                begin
                    strobe_activate <= 0;
                    counter <= counter - 1;
                end
            end
        end
    end
endmodule