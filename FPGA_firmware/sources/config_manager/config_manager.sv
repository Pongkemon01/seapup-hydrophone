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

/* Module to store and manage all settings such as trigger level and poten value.
 * It also parse the configuration setting data from host through FX3S slave FIFO.
 * The input data have the format as
 *  1 bytes: ID (fixed as 0xDC)
 *  1 bytes: Configuration fields
 *      The configuraton fields are Bit-fields indicates which configuration to set.
 *      This field also indicates what data would follow. Each bit has meaning as:
 *          bit 7 - 4: Pinger frequency index the list of frequency is shown below
 *          bit 3: 1 = enable trigger level setting
 *          bit 2: 1 = enable amplifier gains setting
 *          bit 1 - bit 0: (Reserved)
 *  2 bytes: New trigger level. (Exists only when the 3rd bit of the prefix is set)
 *  4 bytes: New amplifier gain for each channel. Each channel can have it gain different
 *      from others. (Thie field exists only when 2nd bit of the prefix is set)
 *
 * Pinger frequency table (for IQ demodulation)
 * +---------+-----------+
 * |  index  | frequency |
 * +---------+-----------+
 * | 4'b0000 |   25kHz   |
 * | 4'b0001 |   26kHz   |
 * | 4'b0010 |   27kHz   |
 * | 4'b0011 |   28kHz   |
 * | 4'b0100 |   29kHz   |
 * | 4'b0101 |   30kHz   |
 * | 4'b0110 |   31kHz   |
 * | 4'b0111 |   32kHz   |
 * | 4'b1000 |   33kHz   |
 * | 4'b1001 |   34kHz   |
 * | 4'b1010 |   35kHz   |
 * | 4'b1011 |   36kHz   |
 * | 4'b1100 |   37kHz   |
 * | 4'b1101 |   38kHz   |
 * | 4'b1110 |   39kHz   |
 * | 4'b1111 |   40kHz   |
 * +---------+-----------+
 */
module config_manager #(
	// constants
	parameter config_prefix = 8'hDC,
	parameter rst_delay = 8			// Total clock cycles to delay the start after reset
	) (
	// Degug signals
	// output [8:0]dbg,
	// output [2:0] current_state,

	// Control
	input logic clk,					// Master clock
	input logic rst,						// Master reset (active high)
	output logic update_poten,		// Trigger for potentiometer register updating. (rising edge)
	
	// Incoming configuration data from the slave fifo output buffer
	input  logic [15:0] din,				// Data from slave FIFO
	input  logic data_valid,				// Indicate that there are some available config data to read
	output logic config_d_oe,				// Enable read-out data
	
	// Control registers
	output logic [3:0] pinger_freq,	// Frequency of the pinger
	output logic [15:0] trigger_level,// hydrophone signal level
	output logic [7:0] poten1_value,	// Value of potentiometer 1 (defines gain of channel 1)
	output logic [7:0] poten2_value,	// Value of potentiometer 2 (defines gain of channel 2)
	output logic [7:0] poten3_value,	// Value of potentiometer 3 (defines gain of channel 3)
	output logic [7:0] poten4_value	// Value of potentiometer 4 (defines gain of channel 4)
);
	localparam DEFAULT_POTEN = 8'd230;		// Default value of potentiometers (making gain = 1)
	localparam DEFAULT_TRIG_LEVEL = 16'd16383;	// Default trigger level

	// State value
	typedef enum logic [2:0] {
		STATE_WAIT_PREFIX,				// Waiting for the prefix
		STATE_READ_PREFIX,				// Read the prefix
		STATE_WAIT_TRIGGER,				// Waiting for trigger level
		STATE_READ_TRIGGER,				// Read trigger level from input
		STATE_WAIT_POTEN1_2,			// Waiting for potentiometer 1 and 2 values
		STATE_READ_POTEN1_2,			// Read values of potentiometer 1 and 2 from input
		STATE_WAIT_POTEN3_4,			// Waiting for potentiometer 3 and 4 values
		STATE_READ_POTEN3_4				// Read values of potentiometer 3 and 4 from input
	} state_t;
	
	// Variables
	State_t state;
	integer counter;			// Counter is for the initialization step
	logic is_update_poten;

	// Combination logic
	// assign current_state = state;
	// assign dbg[7:0] = trigger_level[15:8];
	// assign dbg[8] = data_valid;

	// Behavioral part
	always_ff @( posedge clk ) begin
		if( rst ) begin
			state <= STATE_WAIT_PREFIX;
			counter <= 0;
			config_d_oe <= 1'b0;
			update_poten <= 1'b0;
			is_update_poten <= 1'b0;
			pinger_freq <= 4'b1111;
			trigger_level <= DEFAULT_TRIG_LEVEL;
			poten1_value <= DEFAULT_POTEN;
			poten2_value <= DEFAULT_POTEN;
			poten3_value <= DEFAULT_POTEN;
			poten4_value <= DEFAULT_POTEN;
		end
		else begin
			if( counter < rst_delay )begin
				counter <= counter + 1;
			end
			else begin
				case( state )
					STATE_WAIT_PREFIX:
					begin
						update_poten <= 1'b1;
						if( data_valid ) begin
							config_d_oe <= 1;
							state <= STATE_READ_PREFIX;
						end
					end
					
					STATE_READ_PREFIX:
					begin
						is_update_poten <= din[2];
						if( din[15:8] == config_prefix ) begin
							$display("Config : Start config");
							pinger_freq <= din[7:4];	// Get pinger frequency
							if( din[3] ) begin
								if( data_valid ) begin
									state <= STATE_READ_TRIGGER;
								end
								else begin
									state <= STATE_WAIT_TRIGGER;
								end
							end
							else begin
								if( din[2] ) begin
									update_poten <= 1'b0;
									if( data_valid ) begin
										state <= STATE_READ_POTEN1_2;
									end
									else begin
										state <= STATE_WAIT_POTEN1_2;
									end
								end
							end
						end
						else begin
							config_d_oe <= 1'b0;
							state <= STATE_WAIT_PREFIX;
						end
					end

					STATE_WAIT_TRIGGER:
					begin
						if( data_valid ) begin
							state <= STATE_READ_TRIGGER;
						end
					end
								
					STATE_READ_TRIGGER:	// Read trigger level from input
					begin
						trigger_level <= din;
						if( is_update_poten ) begin
							update_poten <= 1'b0;
							if( data_valid ) begin
								state <= STATE_READ_POTEN1_2;
							end
							else begin
								state <= STATE_WAIT_POTEN1_2;
							end
						end
						else begin
							config_d_oe <= 1'b0;
							state <= STATE_WAIT_PREFIX;			// All reserved bits mean doing nothing
						end
					end

					STATE_WAIT_POTEN1_2:	// Waiting for potentiometer 1 and 2 values
					begin
						if( data_valid ) begin
							state <= STATE_READ_POTEN1_2;
						end
					end
					
					STATE_READ_POTEN1_2:	// Read values of potentiometer 1 and 2 from input
					begin
						{ poten1_value, poten2_value } <= din;
						if( data_valid ) begin
							state <= STATE_READ_POTEN3_4;
						end
						else begin
							state <= STATE_WAIT_POTEN3_4;
						end
					end

					STATE_WAIT_POTEN3_4:	// Waiting for potentiometer 3 and 4 values
					begin
						if( data_valid ) begin
							state <= STATE_READ_POTEN3_4;
						end
					end
					
					STATE_READ_POTEN3_4: // Read values of potentiometer 3 and 4 from input
					begin
						{ poten3_value, poten4_value } <= din;
						config_d_oe <= 1'b0;
						state <= STATE_WAIT_PREFIX;
					end				
				endcase
			end
		end
	end
endmodule
