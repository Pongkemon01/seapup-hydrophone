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

// Cascading FIFO for longer prefix
module CascadedFIFO64bit #( 
	parameter DEPTH = 20,
	parameter BACKLOG_SIZE = 10000
) (
    input clk,
    input rst,
    input wr_en,
    input rd_en,
    input [63:0] d_in,  // 17-bit data (16-bit sampling + 1-bit pktend)

    output [63:0] d_out,  // 17-bit data (16-bit sampling + 1-bit pktend)
    output is_empty,       // Fifo is empty
    output is_almost_full  // Fifo is almost full
);

	localparam ALMOST_FULL_TH = 512 - (BACKLOG_SIZE % 512);
	localparam ALMOST_FULL_FIFO = (DEPTH - (BACKLOG_SIZE / 512) - 1);

    //************************************************************
    // IP Instances
    //************************************************************
    // Connecting logic
    wire [63:0] din[0:DEPTH - 1];
    wire [63:0] dout[0:DEPTH - 1];
    wire rden[0:DEPTH - 1], wren[0:DEPTH - 1];
    wire empty[0:DEPTH - 1], almost_empty[0:DEPTH - 1];
    wire full[0:DEPTH - 1], almost_full[0:DEPTH - 1];

    // Generate "DEPTH" instances and assign port connections
    genvar x;

    generate
    for( x = 0; x < DEPTH; x = x + 1 )
    begin
    // FIFO36E1: 36Kb FIFO (First-In-First-Out) Block RAM Memory
    //           7 Series
    // Xilinx HDL Language Template, version 2021.2

    FIFO36E1 #(
        .ALMOST_EMPTY_OFFSET(13'h0080),    // Sets the almost empty threshold
        .ALMOST_FULL_OFFSET(ALMOST_FULL_TH), // Sets almost full threshold to 20 available entries
		.DATA_WIDTH(72),                   // Sets data width to 4-72
		.DO_REG(1),                        // Enable output register (1-0) Must be 1 if EN_SYN = FALSE
		.EN_ECC_READ("FALSE"),             // Enable ECC decoder, FALSE, TRUE
		.EN_ECC_WRITE("FALSE"),            // Enable ECC encoder, FALSE, TRUE
		.EN_SYN("FALSE"),                  // Specifies FIFO as Asynchronous (FALSE) or Synchronous (TRUE)
		.FIFO_MODE("FIFO36_72"),              // Sets mode to "FIFO36" or "FIFO36_72"
		.FIRST_WORD_FALL_THROUGH("TRUE"),  // Sets the FIFO FWFT to FALSE, TRUE
		.INIT(72'h000000000000000000),     // Initial values on output port
		.SIM_DEVICE("7SERIES"),            // Must be set to "7SERIES" for simulation behavior
		.SRVAL(72'h000000000000000000)     // Set/Reset value for output port
    )
    fifo_departure_inst (
        // ECC Signals: 1-bit (each) output: Error Correction Circuitry ports
        .DBITERR(),                    // 1-bit output: Double bit error status
        .ECCPARITY(),                  // 8-bit output: Generated error correction parity
        .SBITERR(),                    // 1-bit output: Single bit error status
        // Read Data: 64-bit (each) output: Read output data
        .DO(dout[x]),                   // 64-bit output: (Data output + pkt_end associated with that word)
        .DOP(),                        // 8-bit output: Parity data output
        // Status: 1-bit (each) output: Flags and other FIFO status outputs
        .ALMOSTEMPTY(almost_empty[x]), // 1-bit output: Almost empty flag
        .ALMOSTFULL(almost_full[x]),   // 1-bit output: Almost full flag
        .EMPTY(empty[x]),              // 1-bit output: Empty flag
        .FULL(full[x]),                // 1-bit output: Full flag
        .RDCOUNT(),                    // 13-bit output: Read count
        .RDERR(),                      // 1-bit output: Read error
        .WRCOUNT(),                    // 13-bit output: Write count
        .WRERR(),                      // 1-bit output: Write error
        // ECC Signals: 1-bit (each) input: Error Correction Circuitry ports
        .INJECTDBITERR(1'b0),              // 1-bit input: Inject a double bit error input
        .INJECTSBITERR(1'b0),
        // Read Control Signals: 1-bit (each) input: Read clock, enable and reset input signals
        .RDCLK(clk),                   // 1-bit input: Read clock
        .RDEN(rden[x]),                // 1-bit input: Read enable
        .REGCE(1'b1),                  // 1-bit input: Clock enable
        .RST(rst),                     // 1-bit input: Reset
        .RSTREG(1'b0),                 // 1-bit input: Output register set/reset
        // Write Control Signals: 1-bit (each) input: Write clock and enable input signals
        .WRCLK(clk),                   // 1-bit input: Rising edge write clock.
        .WREN(wren[x]),                // 1-bit input: Write enable
        // Write Data: 64-bit (each) input: Write input data
        .DI(din[x]),                  // 64-bit input: ( Data input + Packet ending signal)
        .DIP(8'b0)                     // 2-bit input: Parity input
    );
    // End of FIFO36E1_inst instantiation
    end
    endgenerate

    // Connect all generated FIFO instance to cascade their depth
    // Referred to UG473 in Xilinx web
    assign din[0] = d_in;
    assign wren[0] = wr_en;
    assign rden[DEPTH - 1] = rd_en;
    assign is_almost_full = almost_full[ALMOST_FULL_FIFO];
    assign is_empty = empty[DEPTH - 1];
    assign d_out = dout[DEPTH - 1];

    // Input Layer
    assign rden[0] = wren[1];

    // generate middle layer fifo connection
    genvar y;
    generate
        for( y = 1; y < DEPTH - 1; y = y + 1 )
        begin
            assign din[y] = dout[y - 1];
            assign wren[y] = ~(empty[y - 1] | full[y]);
            assign rden[y] = wren[y + 1];            
        end
    endgenerate
    // Output layer
    assign din[DEPTH - 1] = dout[DEPTH - 2];
    assign wren[DEPTH - 1] = ~(empty[DEPTH - 2] | full[DEPTH - 1]);

endmodule

// Module to manage trigger header and tailer
// Outut data is valid at the rising edge of the next clock after the "trigged" signal
//
// *** CAUTION : Because of delay in FIFO, there are 11 clocks delay for "almost_full" flags.
// Moreover, the minimum backlog threshold is 7. Therefore, the required backlog size
// must be at least 18.
//
module hydrophone_trigger_backlog
	#(
	parameter PRETRIG_SAMPLING = 10000,	// Number of d_in samples preceded of the trigged points
	parameter POSTTRIG_SAMPLING = 10000	// Number of d_in samples include in a valid data packet after the trigger level is not satisfied
	) (
	// Debug
	output rdy, fifo_rdy,		// Debug signals

	input rst,					// system reset (active high)
	input clk,					// signal clock (64 MHz)
	input [63:0] d_in,			// data input (concatenation of 4 16-bit data) all in format Q14.2
	input input_strobe,			// Strobe signal from each channel
	input trigger_event,		// Trigger condition met
	output [63:0] d_out,		// data output  all in format Q14.2
	output output_strobe,		// Combined strobe signal from all channels plus edge detected
	output trigged			// indicates that the data is part of valid packet
);
	// Constants
	localparam TOTAL_TAIL = (PRETRIG_SAMPLING + POSTTRIG_SAMPLING);
	localparam ALMOST_FULL_THRESHOLD = PRETRIG_SAMPLING - 11;	// 11 overhead clocks

	reg [15:0] t_counter;		// Counter for packet tailing
	reg tail_trigged;			// Trigger delay for tailing
	reg strb_d, strb_dd;		// Delay line of strobe signal to detect rising edge
	reg rst_d, rst_dd, rst_3d, rst_4d, rst_5d, rst_6d, rst_7d, rst_8d;	// Delay line for reset signal to make the internal reset time greater than 5 clk

	wire rst_internal;			// Internal reset signal
	wire fifo_rst_internal;		// Internal reset for fifo module
	wire almost_full;			// FIFO is almost full
	wire strobe_all;			// Combined strobe signals
	wire fifo_rd_en, fifo_wr_en;	// Combined read and write enable of all signal

	
	// Combine all strobe signals
	assign fifo_rst_internal = rst | rst_d | rst_dd | rst_3d | rst_4d | rst_5d;
	assign rst_internal = fifo_rst_internal | rst_6d | rst_7d | rst_8d;
	assign output_strobe = strb_d & ~strb_dd & ~rst_internal;
	assign fifo_wr_en = output_strobe;
	assign fifo_rd_en = almost_full & output_strobe & ~rst_internal ;// Enable read when FIFO almost full too.
	assign trigged = trigger_event | tail_trigged;	// Trigger comes from actual event plus tailing

	// Debug signals
	assign rdy = ~rst_internal;
	assign fifo_rdy = almost_full;

	// Initial block
	initial
	begin
		t_counter <= TOTAL_TAIL;
		tail_trigged <= 0;
		strb_d <= 0;
		strb_dd <= 0;
		rst_8d <= 1;
		rst_7d <= 1;
		rst_6d <= 1;
		rst_5d <= 1;
		rst_4d <= 1;
		rst_3d <= 1;
		rst_dd <= 1;
		rst_d <= 1;
	end
	
	// Delay line for reset
	always @(posedge clk)
	begin
		rst_8d <= rst_7d;
		rst_7d <= rst_6d;
		rst_6d <= rst_5d;
		rst_5d <= rst_4d;
		rst_4d <= rst_3d;
		rst_3d <= rst_dd;
		rst_dd <= rst_d;
		rst_d <= rst;
	end
	
	// Delay line for edge detection
	always @(negedge clk)
	begin
		if( rst_internal )
		begin
			strb_d <= 0;
			strb_dd <= 0;
		end
		else
		begin
			strb_dd <= strb_d;
			strb_d <= input_strobe;
		end
	end

	// Main state machine
	always @(negedge clk)
	begin
		if( rst_internal )
		begin
			// Reset signal asserted. Just initialize state
			t_counter <= TOTAL_TAIL;
			tail_trigged <= 0;
		end
		else
		begin
			if( almost_full )
			begin
				if( trigger_event )
				begin
					// Trigged
					tail_trigged <= 1;
					t_counter <= TOTAL_TAIL;
				end
				else
				begin
					// clk freq. is higher than data rate, so we use the Strobe signal indicates each datum
					if( output_strobe )
					begin
						if( tail_trigged && t_counter != 0 )
							t_counter <= t_counter - 1;
						else
							tail_trigged <= 0;
					end
				end
			end
		end
	end


	// FIFO for backlog
	CascadedFIFO64bit #(.BACKLOG_SIZE(ALMOST_FULL_THRESHOLD)) 
				backlog(.clk(clk), .rst(fifo_rst_internal), .d_in(d_in), .d_out(d_out), 
				.rd_en(fifo_rd_en), .wr_en(fifo_wr_en), .is_almost_full(almost_full), .is_empty() );

endmodule

// Helper module to get absolute value
module absolute( input [15:0] in, output [15:0] out );
	assign out = in[15] ? -in : in;
endmodule

// Main trigger module
// Outut data is valid at the rising edge of the next clock after the "trigged" signal
module hydrophone_simple_trigger (
	// Debug signal
	output [63:0] abs_data,
	output [15:0] abs_trig,

	input rst,					// system reset (active high)
	input clk,					// signal clock (64 MHz)
	input enable,				// enable trigger funtion (aka. start of the capture function)
	input [63:0] d_in,			// data input (concatenation of 4 16-bit data) all in format Q13.6
	input input_strobe,			// Strobe signal from each channel
	input [15:0] trigger_level,	// level of the trigger in 16-bit signed integer in format Q13.2
	output reg trigged			// indicates that the data is part of packet of trigged signal
);
	// Variables
	reg strb_d, strb_dd;		// Delay line of strobe signal to detect rising edge
	reg rst_d, rst_dd, rst_3d, rst_4d, rst_5d, rst_6d, rst_7d, rst_8d;	// Delay line for reset signal to make the internal reset time greater than 5 clk

	wire rst_internal;			// Internal reset signal
	
	wire [63:0] abs_d_in;		// Magnetude (aka. absolute) values of d_in
	wire [15:0] abs_trigger;	// Magnetude of trigger level
	
	// Combine all strobe signals
	assign rst_internal = rst | rst_d | rst_dd | rst_3d | rst_4d | rst_5d | rst_6d | rst_7d | rst_8d;
	assign data_strobe = strb_d & ~strb_dd & ~rst_internal; 

	// Debug
	assign abs_data = abs_d_in;
	assign abs_trig = abs_trigger;
	
	// Absolute implementation
	absolute abs1( .in(d_in[15:0]), .out(abs_d_in[15:0]) );
	absolute abs2( .in(d_in[31:16]), .out(abs_d_in[31:16]) );
	absolute abs3( .in(d_in[47:32]), .out(abs_d_in[47:32]) );
	absolute abs4( .in(d_in[63:48]), .out(abs_d_in[63:48]) );
	absolute abs5( .in(trigger_level), .out(abs_trigger) );
	
	// Initial block
	initial
	begin
		trigged <= 0;
		strb_d <= 0;
		strb_dd <= 0;
		rst_8d <= 1;
		rst_7d <= 1;
		rst_6d <= 1;
		rst_5d <= 1;
		rst_4d <= 1;
		rst_3d <= 1;
		rst_dd <= 1;
		rst_d <= 1;
	end
	
	// Delay line for reset
	always @(posedge clk)
	begin
		rst_8d <= rst_7d;
		rst_7d <= rst_6d;
		rst_6d <= rst_5d;
		rst_5d <= rst_4d;
		rst_4d <= rst_3d;
		rst_3d <= rst_dd;
		rst_dd <= rst_d;
		rst_d <= rst;
	end
	
	// Delay line for edge detection
	always @(negedge clk)
	begin
		if( rst_internal )
		begin
			strb_d <= 0;
			strb_dd <= 0;
		end
		else
		begin
			strb_dd <= strb_d;
			strb_d <= input_strobe;
		end
	end
	
	// Main state machine
	always @(negedge clk)
	begin
		if( rst_internal )
		begin
			// Reset signal asserted. Just initialize state
			trigged <= 0;
		end
		else
		begin
			if( enable && data_strobe && ( ( abs_d_in[15:0] >= abs_trigger ) ||
				( abs_d_in[31:16] >= abs_trigger ) ||
				( abs_d_in[47:32] >= abs_trigger ) ||
				( abs_d_in[63:48] >= abs_trigger ) )
			  )
			begin
			   // Trigged
			   trigged <= 1;
			end
			else
			begin
				trigged <= 0;
			end
		end
	end
endmodule