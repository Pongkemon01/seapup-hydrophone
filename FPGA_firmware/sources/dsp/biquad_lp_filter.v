`timescale 1ns / 10ps

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

// Helper module to hide many static configurations
//
// It performs: dout = (A * B) + Cascade_in
//
// Input and output ports have full width. If the actual data have different 
// word length, the external module must extended the data explicitly
module DSP(
	input clk,
	input rst,
	input [29:0] A,
	input [17:0] B,
	input [47:0] Cascade_in,
	output [47:0] dout
);
	// DSP48E1: 48-bit Multi-Functional Arithmetic Block
	//          7 Series
	// Xilinx HDL Language Template, version 2022.1

	DSP48E1 #(
   		// Feature Control Attributes: Data Path Selection
   		.A_INPUT("DIRECT"),               // Selects A input source, "DIRECT" (A port) or "CASCADE" (ACIN port)
   		.B_INPUT("DIRECT"),               // Selects B input source, "DIRECT" (B port) or "CASCADE" (BCIN port)
   		.USE_DPORT("FALSE"),              // Select D port usage (TRUE or FALSE)
   		.USE_MULT("MULTIPLY"),            // Select multiplier usage ("MULTIPLY", "DYNAMIC", or "NONE")
   		.USE_SIMD("ONE48"),               // SIMD selection ("ONE48", "TWO24", "FOUR12")
   		// Pattern Detector Attributes: Pattern Detection Configuration
   		.AUTORESET_PATDET("NO_RESET"),    // "NO_RESET", "RESET_MATCH", "RESET_NOT_MATCH"
   		.MASK(48'h3fffffffffff),          // 48-bit mask value for pattern detect (1=ignore)
   		.PATTERN(48'h000000000000),       // 48-bit pattern match for pattern detect
   		.SEL_MASK("MASK"),                // "C", "MASK", "ROUNDING_MODE1", "ROUNDING_MODE2"
   		.SEL_PATTERN("PATTERN"),          // Select pattern value ("PATTERN" or "C")
   		.USE_PATTERN_DETECT("NO_PATDET"), // Enable pattern detect ("PATDET" or "NO_PATDET")
   		// Register Control Attributes: Pipeline Register Configuration
   		.ACASCREG(0),                     // Number of pipeline stages between A/ACIN and ACOUT (0, 1 or 2)
   		.ADREG(0),                        // Number of pipeline stages for pre-adder (0 or 1)
   		.ALUMODEREG(0),                   // Number of pipeline stages for ALUMODE (0 or 1)
   		.AREG(0),                         // Number of pipeline stages for A (0, 1 or 2)
   		.BCASCREG(0),                     // Number of pipeline stages between B/BCIN and BCOUT (0, 1 or 2)
   		.BREG(0),                         // Number of pipeline stages for B (0, 1 or 2)
   		.CARRYINREG(0),                   // Number of pipeline stages for CARRYIN (0 or 1)
   		.CARRYINSELREG(0),                // Number of pipeline stages for CARRYINSEL (0 or 1)
   		.CREG(0),                         // Number of pipeline stages for C (0 or 1)
   		.DREG(0),                         // Number of pipeline stages for D (0 or 1)
   		.INMODEREG(0),                    // Number of pipeline stages for INMODE (0 or 1)
   		.MREG(0),                         // Number of multiplier pipeline stages (0 or 1)
   		.OPMODEREG(0),                    // Number of pipeline stages for OPMODE (0 or 1)
   		.PREG(0)                          // Number of pipeline stages for P (0 or 1)
	)
	DSP48E1_inst (
   		// Cascade: 30-bit (each) output: Cascade Ports
   		.ACOUT(ACOUT),              // 30-bit output: A port cascade output
   		.BCOUT(),                   // 18-bit output: B port cascade output
   		.CARRYCASCOUT(),     		// 1-bit output: Cascade carry output
   		.MULTSIGNOUT(),       		// 1-bit output: Multiplier sign cascade output
   		.PCOUT(),                   // 48-bit output: Cascade output
   		// Control: 1-bit (each) output: Control Inputs/Status Bits
   		.OVERFLOW(),             	// 1-bit output: Overflow in add/acc output
   		.PATTERNBDETECT(), 			// 1-bit output: Pattern bar detect output
   		.PATTERNDETECT(),   		// 1-bit output: Pattern detect output
   		.UNDERFLOW(),           	// 1-bit output: Underflow in add/acc output
   		// Data: 4-bit (each) output: Data Ports
  	 	.CARRYOUT(),             	// 4-bit output: Carry output
   		.P(dout),                   // 48-bit output: Primary data output
   		// Cascade: 30-bit (each) input: Cascade Ports
   		.ACIN(30'b0),               // 30-bit input: A cascade data input
   		.BCIN(18'b0),               // 18-bit input: B cascade input
   		.CARRYCASCIN(0),       		// 1-bit input: Cascade carry input
   		.MULTSIGNIN(0),    			// 1-bit input: Multiplier sign input
   		.PCIN(Cascade_in),  		// 48-bit input: P cascade input
   		// Data: 30-bit (each) input: Data Ports
   		.A(A),                      // 30-bit input: A data input
   		.B(B),                      // 18-bit input: B data input
   		.C(48'b0),                  // 48-bit input: C data input
   		.CARRYIN(0),               	// 1-bit input: Carry input signal
   		.D(25'b0),                  // 25-bit input: D data input

   		// Control: 4-bit (each) input: Control Inputs/Status Bits
   		.ALUMODE(4'b0),             // 4-bit input: ALU control input (UG479 page 35)
   		.CARRYINSEL(3'b0),         	// 3-bit input: Carry select input (UG479 page 37)
   		.CLK(clk),                  // 1-bit input: Clock input
   		.INMODE(5'b00000),          // 5-bit input: INMODE control input (UG479 pages 31 - 32)
   		.OPMODE(7'b001_0101),       // 7-bit input: Operation mode input (UG479 page 34)
   		// Reset/Clock Enable: 1-bit (each) input: Reset/Clock Enable Inputs
   		.CEA1(0),                   // 1-bit input: Clock enable input for 1st stage AREG
   		.CEA2(0),                   // 1-bit input: Clock enable input for 2nd stage AREG
   		.CEAD(0),                   // 1-bit input: Clock enable input for ADREG
   		.CEALUMODE(0),           	// 1-bit input: Clock enable input for ALUMODE
   		.CEB1(0),                   // 1-bit input: Clock enable input for 1st stage BREG
   		.CEB2(0),                   // 1-bit input: Clock enable input for 2nd stage BREG
   		.CEC(0),                    // 1-bit input: Clock enable input for CREG
   		.CECARRYIN(0),           	// 1-bit input: Clock enable input for CARRYINREG
   		.CECTRL(0),                 // 1-bit input: Clock enable input for OPMODEREG and CARRYINSELREG
   		.CED(0),                    // 1-bit input: Clock enable input for DREG
   		.CEINMODE(0),             	// 1-bit input: Clock enable input for INMODEREG
   		.CEM(0),                    // 1-bit input: Clock enable input for MREG
   		.CEP(0),                    // 1-bit input: Clock enable input for PREG
   		.RSTA(rst),                 // 1-bit input: Reset input for AREG
   		.RSTALLCARRYIN(rst),   // 1-bit input: Reset input for CARRYINREG
   		.RSTALUMODE(rst),         	// 1-bit input: Reset input for ALUMODEREG
   		.RSTB(rst),                 // 1-bit input: Reset input for BREG
   		.RSTC(rst),                 // 1-bit input: Reset input for CREG
   		.RSTCTRL(rst),              // 1-bit input: Reset input for OPMODEREG and CARRYINSELREG
   		.RSTD(rst),                 // 1-bit input: Reset input for DREG and ADREG
   		.RSTINMODE(rst),            // 1-bit input: Reset input for INMODEREG
   		.RSTM(rst),                 // 1-bit input: Reset input for MREG
   		.RSTP(rst)                  // 1-bit input: Reset input for PREG
	);

	// End of DSP48E1_inst instantiation
endmodule

/*
 * Implementation of Biquad IIR filter operates as low-pass filter with the 
 * minimum pass-band frequency. From Matlab, Biquad filter is unstable with
 * the cut-off frequency less than or equal to 0.01 times the sampling rate.
 * Therefore, this implementation has the cut-off frequency just above this
 * unstable band. Hence, it is the minimum posible cut-off frequency.
 *
 * Digital filter has the frequency response related to the sampling frequency.
 * The cut-off frequencies usually speficied relatively to the sampling such as
 * 0.01 x fs.
 */

// This module designed to fit with Seapup filter specifiaton, which set b0 and b2 to 1
// **** All Qm.n designation are in AMD convension, where m + n is the total bits including sign bit ****
// The module has 1 clock delay as it must latch the input first.
//
// All IP modules here are combinational implementation. Therefore, the final result
// runs directly from the computaion stage to the next stage without any pipeline.
// Therefore, further synchronization should be performed in the packetizer module.
module biquad_lp_filter( 
	input clk, 				// 64 MHz system clock
	input rst,				// System reset
	input strobe,			// Strobe signal for din

	// Input and output data
	input signed [15:0] din, // Input from IQ demod. This input connects to trigger's FIFO output through some IQ-demod logic
							// The input data is actually in the form of Q14.2. However, we pretend to divide it by 2^13 to be Q.19
							// This pretending does not cause any extra operation as we just scale it with the same bit length.
	output signed [15:0] dout // Output of the filter in the format of Q.15

	// Coefficients both are 18 bits wide in Q2.16 format.
	input signed [17:0] b2,
	input signed [17:0] minus_a1,	// a1 must be inverted as it is a part of denomonator
	input signed [17:0] minus_a2	// a2 is also a part of denominator; thus, it should be inverted
	);

	// Input must be signed-extended to 16 bits (Q14.2). All delayed lines are defined in this format
	reg signed [15:0] input_latch;
	reg signed [15:0] numerator_delay_line[0:1];

	// Strobe delay line for edge detection
	reg strb_d, strb_dd, is_strobe;
	assign is_strobe = strb_d & (~strb_dd);

	initial
	begin
		input_latch <= 18'b0;
		numerator_delay_line[0] <= 18'b0;
		numerator_delay_line[1] <= 18'b0;
		strb_d <= 0;
		strb_dd <= 0;
	end

	// Strobe edge detection line
	always @(posedge clk)
	begin
		strb_dd <= strb_d;
		strb_d <= strobe;
	end
	
	// Forward part (Numerator). Delay line.
	always @(posedge clk)
	begin
		if( is_strobe )
		begin
			numerator_delay_line[1] <= numerator_delay_line[0];
			numerator_delay_line[0] <= input_latch;
			input_latch <= din;			// Signed extended din from Q14.2 to Q16.2
		end
	end
	
	// Input data is Q14.2 and Coefficients are Q2.16. Therefore, their multiplication generates
	// Q16.18 data. They should be extended to Q30.18 (48 bits)
	wire signed [33:0] raw_mult;
	wire signed [47:0] numerator_res[0:2];
	assign raw_mult = numerator_delay_line[0] * b2;		// Signed multiplication. Result is in Q16.18
	assign numerator_res[0] = { {16(input_latch[15])}, input_latch, 16'b0 }; // b0 = 1; just extend bits
	assign numerator_res[1] = { {14(raw_mult[33])}, raw_mult }				// Extended to Q30.18
	assign numerator_res[2] = { {16(numerator_delay_line[1][15])}, numerator_delay_line[1], 16'b0 }; // b2 = 1

	// Sum the forward part. The actual input was signed extended; thus, we have enough head room for the result.
	wire [47:0] FIR_part;		// It is in Q30.18
	assign FIR_part = numerator_res[0] + numerator_res[1] + numerator_res[2];

	// Backward part (Denominator). This part must be perform by DSP slices or actual multiplication.
	// The DSP slice require 2 inputs with length of 30 bits and 18 bits. 
	wire [47:0] out;					// Final summation before rounding in Q30.18
	wire [29:0] round_out;				// Rounded "out" to Q28.2 
	reg [29:0] denom_delay_line[0:1];	// Q28.2
	
	initial
	begin
		denom_delay_line[0] <= 30'b0;
		denom_delay_line[1] <= 30'b0;
	end
	
	// Create delay line
	assign round_out = { out[47], out[44:16] } + { 29'b0, out[15] };	// Rounding Q30.18 to Q28.2
	always @(posedge clk)
	begin
		if( is_strobe )
		begin
			denom_delay_line[1] <= denom_delay_line[0];
			denom_delay_line[0] <= (round_out === 30'bX) ? 30'b0 : round_out; // Prevent unknown data at start
		end
	end
	
	// IP module for multiply-add
	wire [47:0] denom_a2_out;	// Result from mul_minus_a2 Q30.18

	DSP denom_a1 ( .clk(clk), .rst(rst), 
		.A(denom_delay_line[0]), .B(minus_a1), .Cascade_in(denum_a2_out), .dout(out));
	DSP denom_a2 ( .clk(clk), .rst(rst), 
		.A(denom_delay_line[1]), .B(minus_a2), .Cascade_in(FIR_part), .dout(denom_a2_out));

	// The "true" output (round_out) is Q28.2.
	// Total significant bits in output is 20 bit max.
	// However, we has only 16 bits space available for output.
	// Therefore, we need to scaled the output by 1/16 (Q28.2 => Q24.6)
	// then rounding and cutting to get Q16.0.
	// Scaling and rounding are simply bit selection.

	assign dout = round_out[21:6] + { 15'b0, round_out[5] };
endmodule
