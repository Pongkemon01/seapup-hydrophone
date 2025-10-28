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

module packetizer #(
	parameter	SAMPLING_PER_PACKET = 1000	// Total sampling in a packet (max 1020)
    ) (
	// Debug output
	// output logic [4:0] debug_main_state,
	// output logic debug_strb_d,
	// output logic [15:0] pkt_size_counter,
	output logic sending,		// Packetizer is in the middle of sending data

	// Control ports
	input logic clk,			// System clock (64 MHz)
	input logic rst,			// System reset (active high)

	// Input ports
	input logic [63:0] din,		// Data input from each channel
	input logic din_strobe,		// Clock to latch the input data (at logic 1 + posedge clk)
	input logic trigged,			// Indicates that the system can detect valid data

	// Output ports
	output logic [15:0] dout,// Output data
	output logic dout_strobe,	// Clock to latch the output data (at each posedge)
	output logic pkt_end,		// 0 = nothing, 1 = current data is the last word

	// Feedback from output buffer
	input logic out_full			// Output buffer is almost full

);

/*
 ADC-stream packet format:
	2 bytes: ID (fixed as 0xDCB0)
	2 bytes sequence: packet counter value. Maximum value is wrapped to 0 when counted.
	4 bytes time stamp: time since the system starts in milli-second. The maximum
        value is wrapped to 0 when counted. (It takes 46 days of continuous running to
        overflow!!!)
	n * 8 bytes: stream of "n" sampled data that satisfied the trigger
*/

	localparam MAX_PKT_SIZE = SAMPLING_PER_PACKET * 4;  // Maximum size of a packet exclude header

	// States
	typedef enum logic[3:0] {
		STATE_IDLE,						// Waiting for trigged and posedge in_strobe
		STATE_SEND_HEADER_SEQ,			// Sending header data (Sequence)
		STATE_SEND_HEADER_TIME_H,		// Sending header data (Timestamp MSB)
		STATE_SEND_HEADER_TIME_L,		// Sending header data (Timestamp LSB)
		STATE_SEND_DATA_CH1,			// Sending the following data (Channel 1)
		STATE_SEND_DATA_CH2,			// Sending the following data (Channel 2)
		STATE_SEND_DATA_CH3,			// Sending the following data (Channel 3)
		STATE_SEND_DATA_CH4,			// Sending the following data (Channel 4)
		STATE_WAIT_STROBE				// Waiting for input strobe signal
	} State_t;

	// Output data selection
	typedef enum logic [2:0] {
		OUT_ID,				// d_out contains packet ID	
		OUT_SEQ,			// d_out contains packet sequence number
		OUT_TIME_L,			// d_out contains low_word of time stamp
		OUT_TIME_H,			// d_out contains high_word of time stamp
		OUT_CH1,			// d_out contains data from channel 1
		OUT_CH2,			// d_out contains data from channel 2
		OUT_CH3,			// d_out contains data from channel 3
		OUT_CH4				// d_out contains data from channel 4
	} Output_Selection_t;

	State_t main_state;

	logic [31:0] timer;
	logic [15:0] seq_cnt;

	Output_Selection_t out_sel;     	// Output data selection
	
	logic [15:0] current_pkt_size;	// size of current packet
	
	// Debug output
	// assign debug_main_state = main_state;
	// assign debug_strb_d = in_strb_d;
	// assign pkt_size_counter = current_pkt_size;

	// Time counter
	logic [15:0] time_cnt;
	always_ff @(posedge clk) begin
		if( rst ) begin
			time_cnt <= 16'b0;
			timer <= 32'b0;
		end
		else begin
			time_cnt <= time_cnt + 1;
			if( time_cnt == 16'd64000 ) begin
				time_cnt <= 16'b0;
				timer <= timer + 32'b1;
			end
		end
	end

	// Data are latched at the middle of strobe signal
	logic [63:0] latched_input;
	always_ff @(posedge clk) begin
		if( rst ) begin
			latched_input <= 64'b0;
		end
		else begin
			if( din_strobe ) begin
				if( trigged ) begin
					latched_input <= din;
				end
				else begin
					latched_input <= 64'b0;
				end
			end
		end
	end

	// Select the output
	always_comb	begin
		case( out_sel )
			OUT_ID:		dout = 16'hDCB0;
			OUT_SEQ:	dout = seq_cnt;
			OUT_TIME_L: dout = timer[15:0];
			OUT_TIME_H: dout = timer[31:16];
			OUT_CH1:	dout = latched_input[15:0];
			OUT_CH2:	dout = latched_input[31:16];
			OUT_CH3:	dout = latched_input[47:32];
			OUT_CH4:	dout = latched_input[63:48];
		endcase
	end

	// Main process
	always_ff @(posedge clk) begin
		if( rst ) begin
			out_sel <= OUT_ID;
			pkt_end <= 1'b0;
			seq_cnt <= 16'b0;
			sending <= 0;
			dout_strobe <= 1'b0;
			main_state <= STATE_IDLE;
		end
		else begin
			case( main_state )
				STATE_IDLE:
				begin
					current_pkt_size <= 16'b0;
					pkt_end <= 1'b0;
					dout_strobe <= 1'b0;
					out_sel <= OUT_ID;		// out header ID
					if( ( trigged && din_strobe == 1 ) && !out_full )
					begin
						current_pkt_size <= 16'd4;
						seq_cnt <= seq_cnt + 1;
						sending <= 1;
						dout_strobe <= 1;
						main_state <= STATE_SEND_HEADER_SEQ;
					end
				end

				// Slave FIFO has clock speed at 64MHz which is much faster than data sending rate.
				// Therefore, we only save 1 backlog for header sending.
				// Each sub-state macro requires 3 clk_64MHz.
				STATE_SEND_HEADER_SEQ:
				begin
					out_sel <= OUT_SEQ;		// out packet sequence, latch ID
					main_state <= STATE_SEND_HEADER_TIME_H;
				end

				STATE_SEND_HEADER_TIME_H:
				begin
					out_sel <= OUT_TIME_H;		// out timestamp (MSB), latch seq
					main_state <= STATE_SEND_HEADER_TIME_L;
				end

				STATE_SEND_HEADER_TIME_L:
				begin
					out_sel <= OUT_TIME_L;		// out timestamp (LSB), latch ts(msb)
					main_state <= STATE_SEND_DATA_CH1;
				end

				STATE_SEND_DATA_CH1:
				begin
					dout_strobe <= 1;		// Redundant for data sending loop
					current_pkt_size <= current_pkt_size + 16'd4;
					out_sel <= OUT_CH1;		// out CH1 sampling, latch ts(lsb), or xxx
					main_state <= STATE_SEND_DATA_CH2;
				end

				STATE_SEND_DATA_CH2:
				begin
					out_sel <= OUT_CH2;		// out CH2 sampling, latch ch1
					main_state <= STATE_SEND_DATA_CH3;
				end

				STATE_SEND_DATA_CH3:
				begin
					out_sel <= OUT_CH3;		// out CH3 sampling, latch ch2
					main_state <= STATE_SEND_DATA_CH4;
				end

				STATE_SEND_DATA_CH4:
				begin
					out_sel <= OUT_CH4;		// out CH4 sampling, latch ch3
					if( ( current_pkt_size > MAX_PKT_SIZE ) || !trigged || out_full ) begin
						// Still have data but packet size reached max.
						// Close current packet and start new packet
					   	pkt_end <= 1;
					end
					if( ( current_pkt_size > MAX_PKT_SIZE ) || !trigged ) begin
						sending <= 0;
						main_state <= STATE_IDLE;
					end
					else begin
						main_state <= STATE_WAIT_STROBE;
					end				
				end

				STATE_WAIT_STROBE:
				begin
					dout_strobe <= 1'b0;
					pkt_end <= 1'b0;
					if( !trigged ) begin
						sending <= 0;
						main_state <= STATE_IDLE;
					end
					else begin
						if( din_strobe == 1 ) begin
							main_state <= STATE_SEND_DATA_CH1;
						end
					end
				end

                default:        // Error!!! We should not be here
                begin
					out_sel <= OUT_ID;
					pkt_end <= 1'b0;
					sending <= 0;
					dout_strobe <= 1'b0;
					main_state <= STATE_IDLE;
				end
			endcase
		end
	end
endmodule
