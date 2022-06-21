`timescale 1ns / 1ns

// --------------------------------------------------------------------------------
// Copyright 2019-2022 Akrapong Patchararungruang.
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
module i2c_slave_model (scl, sda);
 
	//
	// parameters
	//
	parameter I2C_ADR = 7'b001_0000;
 
	//
	// input && outpus
	//
	input scl;
	inout sda;
 
	//
	// Variable declaration
	//
	wire debug = 1'b1;
 
	reg [7:0] mem [7:0]; // initiate memory
	reg [7:0] mem_adr;   // memory address
	reg [7:0] mem_do;    // memory data output
 
	reg sta, d_sta;
	reg sto, d_sto;
 
	reg [7:0] sr;        // 8bit shift register
	reg       rw;        // read/write direction
 
	wire      my_adr;    // my address called ??
	wire      i2c_reset; // i2c-statemachine reset
	reg [2:0] bit_cnt;   // 3bit downcounter
	wire      acc_done;  // 8bits transfered
	reg       ld;        // load downcounter
 
	reg       sda_o;     // sda-drive level
	wire      sda_dly;   // delayed version of sda
 
	// statemachine declaration
	parameter idle        = 3'b000;
	parameter slave_ack   = 3'b001;
	parameter get_mem_adr = 3'b010;
	parameter gma_ack     = 3'b011;
	parameter data        = 3'b100;
	parameter data_ack    = 3'b101;
 
	reg [2:0] state; // synopsys enum_state
 
	//
	// module body
	//
 
	initial
	begin
	   sda_o = 1'b1;
	   state = idle;
	end
 
	// generate shift register
	always @(posedge scl)
	  sr <= #1 {sr[6:0],sda};
 
	//detect my_address
	assign my_adr = (sr[7:1] == I2C_ADR);
	// FIXME: This should not be a generic assign, but rather
	// qualified on address transfer phase and probably reset by stop
 
	//generate bit-counter
	always @(posedge scl)
	  if(ld)
	    bit_cnt <= #1 3'b111;
	  else
	    bit_cnt <= #1 bit_cnt - 3'h1;
 
	//generate access done signal
	assign acc_done = !(|bit_cnt);
 
	// generate delayed version of sda
	// this model assumes a hold time for sda after the falling edge of scl.
	// According to the Phillips i2c spec, there s/b a 0 ns hold time for sda
	// with regards to scl. If the data changes coincident with the clock, the
	// acknowledge is missed
	// Fix by Michael Sosnoski
	assign #1 sda_dly = sda;
 
 
	//detect start condition
	always @(negedge sda)
	  if(scl)
	    begin
	        sta   <= #1 1'b1;
		d_sta <= #1 1'b0;
		sto   <= #1 1'b0;
 
	        if(debug)
	          $display("DEBUG i2c_slave p_addr=%x; start condition detected at %t", I2C_ADR, $time);
	    end
	  else
	    sta <= #1 1'b0;
 
	always @(posedge scl)
	  d_sta <= #1 sta;
 
	// detect stop condition
	always @(posedge sda)
	  if(scl)
	    begin
	       sta <= #1 1'b0;
	       sto <= #1 1'b1;
 
	       if(debug)
	         $display("DEBUG i2c_slave p_addr=%x; stop condition detected at %t", I2C_ADR, $time);
	    end
	  else
	    sto <= #1 1'b0;
 
	//generate i2c_reset signal
	assign i2c_reset = sta || sto;
 
	// generate statemachine
	always @(negedge scl or posedge sto)
	  if (sto || (sta && !d_sta) )
	    begin
	        state <= #1 idle; // reset statemachine
 
	        sda_o <= #1 1'b1;
	        ld    <= #1 1'b1;
	    end
	  else
	    begin
	        // initial settings
	        sda_o <= #1 1'b1;
	        ld    <= #1 1'b0;
 
	        case(state) // synopsys full_case parallel_case
	            idle: // idle state
	              if (acc_done && my_adr)
	                begin
	                    state <= #1 slave_ack;
	                    rw <= #1 sr[0];
	                    sda_o <= #1 1'b0; // generate i2c_ack
 
	                    #2;
	                    if(debug && rw)
	                      $display("DEBUG i2c_slave p_addr=%x; command byte received (read) at %t", I2C_ADR, $time);
	                    if(debug && !rw)
	                      $display("DEBUG i2c_slave p_addr=%x; command byte received (write) at %t", I2C_ADR, $time);
 
	                    if(rw)
	                      begin
	                          mem_do <= #1 mem[mem_adr];
 
	                          if(debug)
	                            begin
	                                #2 $display("DEBUG i2c_slave p_addr=%x; data block read %x from address %x (1)", I2C_ADR, mem_do, mem_adr);
	                                #2 $display("DEBUG i2c_slave p_addr=%x; memcheck [0]=%x, [1]=%x, [2]=%x", mem[4'h0], I2C_ADR, mem[4'h1], mem[4'h2]);
	                            end
	                      end
	                end
 
	            slave_ack:
	              begin
	                  if(rw)
	                    begin
	                        state <= #1 data;
	                        sda_o <= #1 mem_do[7];
	                    end
	                  else
	                    state <= #1 get_mem_adr;
 
	                  ld    <= #1 1'b1;
	              end
 
	            get_mem_adr: // wait for memory address
	              if(acc_done)
	                begin
	                    state <= #1 gma_ack;
	                    mem_adr <= #1 sr; // store memory address
	                    sda_o <= #1 !(sr <= 255); // generate i2c_ack, for valid address
 
	                    if(debug)
	                      #1 $display("DEBUG i2c_slave p_addr=%x; address received. adr=%x, ack=%b", I2C_ADR, sr, !(sr <= 255));
	                end
 
	            gma_ack:
	              begin
	                  state <= #1 data;
	                  ld    <= #1 1'b1;
	              end
 
	            data: // receive or drive data
	              begin
	                  if(rw)
	                    sda_o <= #1 mem_do[7];
 
	                  if(acc_done)
	                    begin
	                        state <= #1 data_ack;
	                        mem_adr <= #2 mem_adr + 8'h1;
	                        sda_o <= #1 (rw && (mem_adr <= 255) ); // send ack on write, receive ack on read
 
	                        if(rw)
	                          begin
	                              #3 mem_do <= mem[mem_adr];
 
	                              if(debug)
	                                #5 $display("DEBUG i2c_slave p_addr=%x; data block read %x from address %x (2)", I2C_ADR, mem_do, mem_adr);
	                          end
 
	                        if(!rw)
	                          begin
	                              mem[ mem_adr ] <= #1 sr; // store data in memory
 
	                              if(debug)
	                                #2 $display("DEBUG i2c_slave p_addr=%x; data block write %x to address %x", I2C_ADR, sr, mem_adr);
	                          end
	                    end
	              end
 
	            data_ack:
	              begin
	                  ld <= #1 1'b1;
 
	                  if(rw)
	                    if(sr[0]) // read operation && master send NACK
	                      begin
	                          state <= #1 idle;
	                          sda_o <= #1 1'b1;
	                      end
	                    else
	                      begin
	                          state <= #1 data;
	                          sda_o <= #1 mem_do[7];
	                      end
	                  else
	                    begin
	                        state <= #1 data;
	                        sda_o <= #1 1'b1;
	                    end
	              end
 
	        endcase
	    end
 
	// read data from memory
	always @(posedge scl)
	  if(!acc_done && rw)
	    mem_do <= #1 {mem_do[6:0], 1'b1}; // insert 1'b1 for host ack generation
 
	// generate tri-states
	assign sda = sda_o ? 1'bz : 1'b0;
 
 
	//
	// Timing checks
	//
 
	wire tst_sto = sto;
	wire tst_sta = sta;
 
/*	specify
	  specparam normal_scl_low  = 4700,
	            normal_scl_high = 4000,
	            normal_tsu_sta  = 4700,
	            normal_thd_sta  = 4000,
	            normal_tsu_sto  = 4000,
	            normal_tbuf     = 4700,
 
	            fast_scl_low  = 1300,
	            fast_scl_high =  600,
	            fast_tsu_sta  = 1300,
	            fast_thd_sta  =  600,
	            fast_tsu_sto  =  600,
	            fast_tbuf     = 1300;
 
	  $width(negedge scl, normal_scl_low);  // scl low time
	  $width(posedge scl, normal_scl_high); // scl high time
 
	  $setup(posedge scl, negedge sda &&& scl, normal_tsu_sta); // setup start
	  $setup(negedge sda &&& scl, negedge scl, normal_thd_sta); // hold start
	  $setup(posedge scl, posedge sda &&& scl, normal_tsu_sto); // setup stop
 
	  $setup(posedge tst_sta, posedge tst_sto, normal_tbuf); // stop to start time
	endspecify
 */
endmodule

module poten_interface_tb ;
	// Device pins
	wire SDA;			// I2C Data
	wire SCL;			// I2C Clock
	
	// Control signals
	reg rst;			// Synchronous reset (active high)
	reg clk;	        // System clock.
	wire start_update;	// Activation bit. Rising edge of this bit means starting of sending data to poten

    // FIFO signal
    wire [15:0] dout;
    wire rd_en;
    wire fifo_empty, fifo_full;
    reg  wr_en;
    reg  [15:0] din;
	
	// Potentiometer data
	wire [7:0] p0_val;
	wire [7:0] p1_val;
	wire [7:0] p2_val;
	wire [7:0] p3_val;

	wire phase;		// Phase of I2C byte operation (0 = activate, 1 = wait for complete)
	wire [2:0] poten_state; // Poten update state
	wire [1:0] poten_sub_state; // Sub state

    // Aux variable
    integer cycle_counter;
    integer conf_index;
    reg [15:0] conf_data[0:15];				// Configuration data
    reg conf_rdy;
    wire fifo_rdy;

    localparam conf_start = 46;         // Must be at least 46 to make FIFO working

    assign fifo_rdy = ~fifo_empty & conf_rdy;
	
	pullup p1(SCL); // pullup scl line
	pullup p2(SDA); // pullup sda line

	// --------------------------------------------------------------------------------
	// module under test
    hydrophone_config_manager hc(
	// Degug signals
	// output [8:0]dbg,
	// output [2:0] current_state,

	    // Interface to slave fifo output buffer
	    .d_in(dout),				// Data from slave FIFO
	    .data_valid(fifo_rdy),		// Indicate that there are some available config data to read
	    .config_d_oe(rd_en),			// Enable read-out data
	
	    // Control
	    .clk(clk),					// Master clock
	    .rst(rst),					// Master reset (active high)
	    .update_poten(start_update),// Trigger for potentiometer register updating. (rising edge)
	
	    // Output
	    .trigger_level(),// hydrophone signal level
	    .poten1_value(p0_val),	// Value of potentiometer 1 (defines gain of channel 1)
	    .poten2_value(p1_val),	// Value of potentiometer 2 (defines gain of channel 2)
	    .poten3_value(p2_val),	// Value of potentiometer 3 (defines gain of channel 3)
	    .poten4_value(p3_val)	// Value of potentiometer 4 (defines gain of channel 4)
    );

	i2c_slave_model #( .I2C_ADR(7'b0101_110) ) poten1 ( .scl(SCL), .sda(SDA) );
	i2c_slave_model #( .I2C_ADR(7'b0101_111) ) poten2 ( .scl(SCL), .sda(SDA) );
	poten_interface poten( .phase(phase), .poten_state(poten_state), .poten_sub_state(poten_sub_state),
                           .SDA(SDA), .SCL(SCL), .rst(rst), .clk(clk), .start_update(start_update),
						   .p0_val(p0_val), .p1_val(p1_val), .p2_val(p2_val), .p3_val(p3_val) );
    
    // --------------------------------------------------------------------------------
    // Aux IP
        //************************************************************
    // IP Instances
    //************************************************************
    // FIFO36E1: 36Kb FIFO (First-In-First-Out) Block RAM Memory
    //           7 Series
    // Xilinx HDL Language Template, version 2021.2

    FIFO36E1 #(
        .ALMOST_EMPTY_OFFSET(13'h0080),    // Sets the almost empty threshold
        .ALMOST_FULL_OFFSET(13'd20),       // Sets almost full threshold to 20 available entries
        .DATA_WIDTH(18),                   // Sets data width to 4-72 (18-bit by 2k-depth)
        .DO_REG(1),                        // Enable output register (1-0) Must be 1 if EN_SYN = FALSE
        .EN_ECC_READ("FALSE"),             // Enable ECC decoder, FALSE, TRUE
        .EN_ECC_WRITE("FALSE"),            // Enable ECC encoder, FALSE, TRUE
        .EN_SYN("FALSE"),                  // Specifies FIFO as Asynchronous (FALSE) or Synchronous (TRUE)
        .FIFO_MODE("FIFO36"),              // Sets mode to "FIFO36" or "FIFO36_72"
        .FIRST_WORD_FALL_THROUGH("TRUE"),  // Sets the FIFO FWFT to FALSE, TRUE
        .INIT(72'h000000000000000000),     // Initial values on output port
        .SIM_DEVICE("7SERIES"),            // Must be set to "7SERIES" for simulation behavior
        .SRVAL(72'h000000000000000000)     // Set/Reset value for output port
    )
    fifo_arrival (
        // ECC Signals: 1-bit (each) output: Error Correction Circuitry ports
        .DBITERR(),                    // 1-bit output: Double bit error status
        .ECCPARITY(),                  // 8-bit output: Generated error correction parity
        .SBITERR(),                    // 1-bit output: Single bit error status
        // Read Data: 64-bit (each) output: Read output data
        .DO(dout),                    // 64-bit output: Data output
        .DOP(),                        // 8-bit output: Parity data output
        // Status: 1-bit (each) output: Flags and other FIFO status outputs
        .ALMOSTEMPTY(),                // 1-bit output: Almost empty flag
        .ALMOSTFULL(fifo_full),          // 1-bit output: Almost full flag
        .EMPTY(fifo_empty),              // 1-bit output: Empty flag
        .FULL(),                       // 1-bit output: Full flag
        .RDCOUNT(),                    // 13-bit output: Read count
        .RDERR(),                      // 1-bit output: Read error
        .WRCOUNT(),                    // 13-bit output: Write count
        .WRERR(),                      // 1-bit output: Write error
        // ECC Signals: 1-bit (each) input: Error Correction Circuitry ports
        .INJECTDBITERR(1'b0),              // 1-bit input: Inject a double bit error input
        .INJECTSBITERR(1'b0),
        // Read Control Signals: 1-bit (each) input: Read clock, enable and reset input signals
        .RDCLK(clk),                   // 1-bit input: Read clock
        .RDEN(rd_en), // 1-bit input: Read enable
        .REGCE(1'b1),                  // 1-bit input: Clock enable
        .RST(rst),       // 1-bit input: Reset
        .RSTREG(1'b0),                 // 1-bit input: Output register set/reset
        // Write Control Signals: 1-bit (each) input: Write clock and enable input signals
        .WRCLK(clk),                   // 1-bit input: Rising edge write clock.
        .WREN(wr_en),      // 1-bit input: Write enable
        // Write Data: 64-bit (each) input: Write input data
        .DI(din),                   // 64-bit input: Data input
        .DIP(2'b0)                     // 2-bit input: Parity input
    );
    // End of FIFO36E1_inst instantiation

	
    // --------------------------------------------------------------------------------
    // Testbench
	initial
	begin
		conf_data[0] <= 16'hDC0C;  // Enable Trigger and Poten settings
		conf_data[1] <= 16'h0100;  // Trigger level (Signed integer)
		conf_data[2] <= 16'h2A4C;  // Poten 1 and 2
		conf_data[3] <= 16'h3B5D;  // Poten 3 and 4
        conf_data[4] <= 16'hAA55;  // EOF
		clk <= 0;
        cycle_counter <= 0;
        conf_index <= 0;
        conf_rdy <= 0;
        wr_en <= 0;
		rst = 1;
	end
	
	// Clk gen
	always 
	begin
		#1 clk = ~clk; 
	end

    // Cycle counter
    always @(posedge clk)
    begin
        cycle_counter <= cycle_counter + 1;
    end

    // Test sequence
    always @(posedge clk)
    begin
        case(cycle_counter)
            1:
            begin
                din <= 0;
            end

            16:
            begin
                rst <= 0;   // Release reset
            end

            conf_start:
            begin
                wr_en <= 1;
                din <= conf_data[conf_index];
                conf_index <= conf_index + 1;
            end

            (conf_start + 1):
            begin
                din <= conf_data[conf_index];
                conf_index <= conf_index + 1;
            end
            
            (conf_start + 2):
            begin
                din <= conf_data[conf_index];
                conf_index <= conf_index + 1;
            end
            
            (conf_start + 3):
            begin
                din <= conf_data[conf_index];
                conf_index <= conf_index + 1;
            end

            (conf_start + 4):
            begin
                wr_en <= 0;
                din <= conf_data[conf_index];
            end

            (conf_start + 6):
            begin
                conf_rdy <= 1;
            end
            
            50000:
            begin
                $display("\n\nstatus: %t Testbench done", $time);
	            $finish;
            end
        endcase
    end
endmodule