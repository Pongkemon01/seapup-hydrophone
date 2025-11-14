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

module simple_fifo
#(
	parameter SIZE_BIT_DEPTH = 6,	// Bit depth of the size of the buffer (2^SIZE_BIT_DEPTH)
	parameter DATA_WIDTH = 64,		// Width of the data (in bits)
	parameter ALMOST_FULL_THRESHOLD = 1,	// Number of free spaces left to assert almost_full
	parameter ALMOST_EMPTY_THRESHOLD = 1,	// Number of data left to assert almost_empty
	parameter IS_FIRST_WORD_FALLTHROUGH = 1	// If set to 1, the first word written to an empty FIFO will appear on the output immediately
) (
    input logic clk,					// signal clock (64 MHz)
    input logic rst,					// system reset (active high)

	// FIFO fill count (number of elements in the FIFO)
	output logic [SIZE_BIT_DEPTH-1:0] fifo_filled,

    // Write interface
    input logic wr_en,					// write enable
    input logic [DATA_WIDTH-1:0] din,	// data input
    output logic almost_full,			// fifo almost full indicator, indicating that only one space is left or full
    output logic full,			        // fifo full indicator

    // Read interface
    input logic rd_en,					// read enable
	output logic almost_empty,			// fifo almost empty indicator, indicating that only one data is left or empty
    output logic empty_n,			    // fifo empty indicator (0 = empty, 1 = not empty)
    output logic [DATA_WIDTH-1:0] dout	// data output
);
    localparam FIFO_SIZE = (1<<SIZE_BIT_DEPTH);

    // Memory array and pointers
    logic [DATA_WIDTH-1:0] fifo [(FIFO_SIZE-1):0];                          // Memory array
    logic [SIZE_BIT_DEPTH-1:0] first_ptr;                               	// Write pointer
    logic [SIZE_BIT_DEPTH-1:0] last_ptr;                                	// Read pointer

	// Verify the threshold parameters
	logic [SIZE_BIT_DEPTH-1:0] almost_full_threshold_val, almost_empty_threshold_val;
	assign	almost_full_threshold_val  = (ALMOST_FULL_THRESHOLD >= FIFO_SIZE) ? FIFO_SIZE - 1 : ALMOST_FULL_THRESHOLD;
	assign	almost_empty_threshold_val = (ALMOST_EMPTY_THRESHOLD >= FIFO_SIZE) ? FIFO_SIZE - 1 : ALMOST_EMPTY_THRESHOLD;

	// Helpers to improve readability
    logic [(SIZE_BIT_DEPTH-1):0]	last_plus_one, first_plus_one;
	assign	first_plus_one = first_ptr + {{(SIZE_BIT_DEPTH-1){1'b0}}, 1'b1};
	assign	last_plus_one  = last_ptr + {{(SIZE_BIT_DEPTH-1){1'b0}}, 1'b1};

	//==============================================================
	// FIFO status logic

	// Generate status signals combinationally
	assign almost_full 	= (fifo_filled >= (FIFO_SIZE - almost_full_threshold_val - 1));
	assign almost_empty = (fifo_filled <= almost_empty_threshold_val);
	assign empty_n     	= (first_ptr != last_ptr);
	assign full        	= (first_plus_one == last_ptr);
	assign fifo_filled  = first_ptr - last_ptr;

    //==============================================================
    // FIFO write logic

    // Write Pointer (first_ptr)
	always_ff @( posedge clk ) begin
		if( rst ) begin
			first_ptr <= 0;
		end 
        else if( wr_en ) begin
			// Cowardly refuse to overflow
			if( !full || rd_en ) 
				first_ptr <= first_plus_one;
		end
	end

	// FIFO Memory Write
	always_ff @( posedge clk ) begin
		if ( wr_en && ( !full || rd_en ) ) // Write our new value only when we have space
			fifo[first_ptr] <= din;
	end

    //==============================================================
    // FIFO read logic

	// Read Pointers (last_ptr and next_ptr)
	always_ff @( posedge clk ) begin
		if( rst )
			last_ptr <= 0;
        else if( rd_en ) begin
			if ( empty_n || wr_en ) 
				last_ptr <= last_plus_one;
		end
	end

	// FIFO Memory Read
	always_ff @( posedge clk ) begin
		if( rd_en || IS_FIRST_WORD_FALLTHROUGH ) begin
			if( empty_n ) // Read our value only when we have data
				dout <= fifo[last_ptr];
			else if( wr_en ) // On underflow, if we are also writing, read the new data
				dout <= din;
		end
	end

	//==============================================================
// Formal verification properties
`ifdef	FORMAL

`ifdef	SIMPLE_FIFO
`define	ASSUME	assume
`else
`define	ASSUME	assert
`endif

    //
    // Assumptions about our input(s)
    //
    //
	logic f_past_valid = 1'b0, 

	//
	// Underflows are a very real possibility, should the user wish to read from this
	// FIFO while it is empty.  Our parent module will need to deal with this.
	//
	// always @(posedge i_clk)
	//	`ASSUME((!almost_empty)||(!i_rd)||(i_reset));
    //
    // Assertions about our outputs
    //
    //

	always_ff @(posedge clk)
		f_past_valid <= 1'b1;

	always_comb begin
        if (!f_past_valid)
            `ASSUME(rst);
    end

	logic [SIZE_BIT_DEPTH-1:0]	f_fill, f_next;
	assign	f_fill = first_ptr - last_ptr;
	assign	f_next = last_ptr + 1'b1;
	always_ff @(posedge clk) begin
		assert(f_fill == fifo_filled);
		if (f_fill == 0) begin
			assert(almost_empty);
			assert(!empty_n);
		end 
        else begin
			assert(!almost_empty);
			assert(empty_n);
		end

		if (f_fill == {(SIZE_BIT_DEPTH){1'b1}})
			assert(almost_full);
		else
			assert(!almost_full);

		assert(last_plus_one == f_next);
	end

	always_ff @(posedge clk)
	if (f_past_valid)
	begin
		if (!$past(rst)) begin
			// Underflow detection
			if (($past(rd_en))&&($past(fifo_filled == 0)))
			begin
				// This core doesn't report underflow errors,
				// but quietly ignores them
				//
				// assert(o_err);
				//
				// On an underflow, we need to be careful not
				// to advance the pointer.
				assert(last_ptr == $past(last_ptr));
			end
			//
			// Overflow detection
			if (($past(wr_en))&&(!$past(rd_en))
					&&($past(almost_full)))
			begin
				// Make sure we didn't advance our write
				// pointer on overflow
				assert(first_ptr == $past(first_ptr));
			end
		end
	end

`endif

endmodule
