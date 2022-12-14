// -------------------------------------------------------------
// 
// File Name: hdl_prj\hdlsrc\zeabus_hydrophone_fixed_point\Channel4_Filter.v
// Created: 2020-04-16 11:11:01
// 
// Generated by MATLAB 9.7 and HDL Coder 3.15
// 
// -------------------------------------------------------------


// -------------------------------------------------------------
// 
// Module: Channel4_Filter
// Source Path: zeabus_hydrophone_fixed_point/Hydrophone processing/Channel4 Filter
// Hierarchy Level: 1
// 
// -------------------------------------------------------------

`timescale 10 ns / 10 ns

module Channel4_Filter
          (clk,
           reset,
           enb_1_320_1,
           enb_1_320_0,
           enb,
           I_in,
           Q_in,
           I_out,
           Q_out);


  input   clk;
  input   reset;
  input   enb_1_320_1;
  input   enb_1_320_0;
  input   enb;
  input   signed [17:0] I_in;  // sfix18_En17
  input   signed [17:0] Q_in;  // sfix18_En17
  output  signed [15:0] I_out;  // sfix16_En15
  output  signed [15:0] Q_out;  // sfix16_En15


  wire signed [17:0] I_in_1;  // sfix18_En17
  reg signed [17:0] I_in_2;  // sfix18_En17
  wire signed [17:0] scaleconst1;  // sfix18_En24
  reg signed [17:0] scaleconst1_1;  // sfix18_En24
  wire signed [35:0] scale1;  // sfix36_En41
  reg signed [35:0] scale1_1;  // sfix36_En41
  wire signed [17:0] numtypeconvert1;  // sfix18_En17
  wire signed [35:0] gain2_cast;  // sfix36_En34
  wire signed [17:0] b1mul1;  // sfix18_En17
  reg signed [17:0] numdelay_process_section1_tap0_bypass_reg;  // sfix18
  wire signed [17:0] numtypeconvert1_1;  // sfix18_En17
  wire signed [17:0] numtypeconvert1_tap0;  // sfix18_En17
  wire signed [35:0] gain_cast;  // sfix36_En34
  wire signed [17:0] b2mul1;  // sfix18_En17
  reg signed [17:0] b2mul1_1;  // sfix18_En17
  wire signed [17:0] b1sum1;  // sfix18_En17
  reg signed [17:0] numtypeconvert1_tap1;  // sfix18_En17
  wire signed [17:0] numtypeconvert1_tap1_1;  // sfix18_En17
  wire signed [35:0] gain1_cast;  // sfix36_En34
  wire signed [17:0] b3mul1;  // sfix18_En17
  reg signed [17:0] b3mul1_1;  // sfix18_En17
  wire signed [17:0] b2sum1;  // sfix18_En17
  wire signed [17:0] coeff_a2_section1;  // sfix18_En16
  reg signed [17:0] coeff_a2_section1_1;  // sfix18_En16
  wire signed [17:0] coeff_a3_section1;  // sfix18_En16
  reg signed [17:0] coeff_a3_section1_1;  // sfix18_En16
  wire signed [17:0] a1sum1;  // sfix18_En17
  reg signed [17:0] a1sum1_tap1;  // sfix18_En17
  wire signed [17:0] a1sum1_tap1_1;  // sfix18_En17
  reg signed [17:0] dendelay_section1_1;  // sfix18_En17
  wire signed [35:0] multiplier1_mul_temp;  // sfix36_En33
  wire signed [17:0] a3mul1;  // sfix18_En17
  reg signed [17:0] a3mul1_1;  // sfix18_En17
  wire signed [17:0] a1sum1_1;  // sfix18_En17
  reg signed [17:0] dendelay_process_section1_tap0_bypass_reg;  // sfix18
  wire signed [17:0] a1sum1_tap0;  // sfix18_En17
  reg signed [17:0] dendelay_section1_0;  // sfix18_En17
  wire signed [35:0] multiplier_mul_temp;  // sfix36_En33
  wire signed [17:0] a2mul1;  // sfix18_En17
  reg signed [17:0] a2mul1_1;  // sfix18_En17
  wire signed [18:0] Sub1_sub_cast;  // sfix19_En17
  wire signed [18:0] Sub1_sub_cast_1;  // sfix19_En17
  wire signed [18:0] a2sum1_temp;  // sfix19_En17
  wire signed [17:0] a2sum1;  // sfix18_En17
  wire signed [18:0] Sub_sub_cast;  // sfix19_En17
  wire signed [18:0] Sub_sub_cast_1;  // sfix19_En17
  wire signed [18:0] a1sum1_temp;  // sfix19_En17
  wire signed [15:0] output_typeconvert;  // sfix16_En15
  reg signed [15:0] t4_bypass_reg;  // sfix16
  wire signed [17:0] Q_in_1;  // sfix18_En17
  reg signed [17:0] Q_in_2;  // sfix18_En17
  wire signed [17:0] scaleconst1_2;  // sfix18_En24
  reg signed [17:0] scaleconst1_3;  // sfix18_En24
  wire signed [35:0] scale1_2;  // sfix36_En41
  reg signed [35:0] scale1_3;  // sfix36_En41
  wire signed [17:0] numtypeconvert1_2;  // sfix18_En17
  wire signed [35:0] gain5_cast;  // sfix36_En34
  wire signed [17:0] b1mul1_1;  // sfix18_En17
  reg signed [17:0] numdelay_process_section1_tap01_bypass_reg;  // sfix18
  wire signed [17:0] numtypeconvert1_3;  // sfix18_En17
  wire signed [17:0] numtypeconvert1_tap0_1;  // sfix18_En17
  wire signed [35:0] gain3_cast;  // sfix36_En34
  wire signed [17:0] b2mul1_2;  // sfix18_En17
  reg signed [17:0] b2mul1_3;  // sfix18_En17
  wire signed [17:0] b1sum1_1;  // sfix18_En17
  reg signed [17:0] numtypeconvert1_tap1_2;  // sfix18_En17
  wire signed [17:0] numtypeconvert1_tap1_3;  // sfix18_En17
  wire signed [35:0] gain4_cast;  // sfix36_En34
  wire signed [17:0] b3mul1_2;  // sfix18_En17
  reg signed [17:0] b3mul1_3;  // sfix18_En17
  wire signed [17:0] b2sum1_1;  // sfix18_En17
  wire signed [17:0] coeff_a2_section1_2;  // sfix18_En16
  reg signed [17:0] coeff_a2_section1_3;  // sfix18_En16
  wire signed [17:0] coeff_a3_section1_2;  // sfix18_En16
  reg signed [17:0] coeff_a3_section1_3;  // sfix18_En16
  wire signed [17:0] a1sum1_2;  // sfix18_En17
  reg signed [17:0] a1sum1_tap1_2;  // sfix18_En17
  wire signed [17:0] a1sum1_tap1_3;  // sfix18_En17
  reg signed [17:0] dendelay_section1_1_1;  // sfix18_En17
  wire signed [35:0] multiplier4_mul_temp;  // sfix36_En33
  wire signed [17:0] a3mul1_2;  // sfix18_En17
  reg signed [17:0] a3mul1_3;  // sfix18_En17
  wire signed [17:0] a1sum1_3;  // sfix18_En17
  reg signed [17:0] dendelay_process_section1_tap01_bypass_reg;  // sfix18
  wire signed [17:0] a1sum1_tap0_1;  // sfix18_En17
  reg signed [17:0] dendelay_section1_0_1;  // sfix18_En17
  wire signed [35:0] multiplier3_mul_temp;  // sfix36_En33
  wire signed [17:0] a2mul1_2;  // sfix18_En17
  reg signed [17:0] a2mul1_3;  // sfix18_En17
  wire signed [18:0] Sub3_sub_cast;  // sfix19_En17
  wire signed [18:0] Sub3_sub_cast_1;  // sfix19_En17
  wire signed [18:0] a2sum1_temp_1;  // sfix19_En17
  wire signed [17:0] a2sum1_1;  // sfix18_En17
  wire signed [18:0] Sub2_sub_cast;  // sfix19_En17
  wire signed [18:0] Sub2_sub_cast_1;  // sfix19_En17
  wire signed [18:0] a1sum1_temp_1;  // sfix19_En17
  wire signed [15:0] output_typeconvert_1;  // sfix16_En15
  reg signed [15:0] t9_bypass_reg;  // sfix16


  assign I_in_1 = I_in;

  always @(posedge clk)
    begin : HwModeRegister4_process
      if (reset == 1'b1) begin
        I_in_2 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          I_in_2 <= I_in_1;
        end
      end
    end



  assign scaleconst1 = 18'sb010100011110101110;



  always @(posedge clk)
    begin : HwModeRegister5_process
      if (reset == 1'b1) begin
        scaleconst1_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          scaleconst1_1 <= scaleconst1;
        end
      end
    end



  assign scale1 = I_in_2 * scaleconst1_1;



  always @(posedge clk)
    begin : PipelineRegister2_process
      if (reset == 1'b1) begin
        scale1_1 <= 36'sh000000000;
      end
      else begin
        if (enb) begin
          scale1_1 <= scale1;
        end
      end
    end



  assign numtypeconvert1 = {{6{scale1_1[35]}}, scale1_1[35:24]};



  // coeff_b1_section1
  assign gain2_cast = {{3{numtypeconvert1[17]}}, {numtypeconvert1, 15'b000000000000000}};
  assign b1mul1 = gain2_cast[34:17];



  always @(posedge clk)
    begin : numdelay_process_section1_tap0_bypass_process
      if (reset == 1'b1) begin
        numdelay_process_section1_tap0_bypass_reg <= 18'sb000000000000000000;
      end
      else begin
        if (enb_1_320_1) begin
          numdelay_process_section1_tap0_bypass_reg <= numtypeconvert1;
        end
      end
    end

  assign numtypeconvert1_1 = (enb_1_320_1 == 1'b1 ? numtypeconvert1 :
              numdelay_process_section1_tap0_bypass_reg);



  assign numtypeconvert1_tap0 = numtypeconvert1_1;

  // coeff_b2_section1
  assign gain_cast = {{2{numtypeconvert1_tap0[17]}}, {numtypeconvert1_tap0, 16'b0000000000000000}};
  assign b2mul1 = gain_cast[34:17];



  always @(posedge clk)
    begin : delayMatch_process
      if (reset == 1'b1) begin
        b2mul1_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          b2mul1_1 <= b2mul1;
        end
      end
    end



  assign b1sum1 = b1mul1 + b2mul1_1;



  always @(posedge clk)
    begin : numdelay_process_section1_tap1_process
      if (reset == 1'b1) begin
        numtypeconvert1_tap1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb_1_320_0) begin
          numtypeconvert1_tap1 <= numtypeconvert1_1;
        end
      end
    end



  assign numtypeconvert1_tap1_1 = numtypeconvert1_tap1;

  // coeff_b3_section1
  assign gain1_cast = {{3{numtypeconvert1_tap1_1[17]}}, {numtypeconvert1_tap1_1, 15'b000000000000000}};
  assign b3mul1 = gain1_cast[34:17];



  always @(posedge clk)
    begin : delayMatch1_process
      if (reset == 1'b1) begin
        b3mul1_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          b3mul1_1 <= b3mul1;
        end
      end
    end



  assign b2sum1 = b1sum1 + b3mul1_1;



  assign coeff_a2_section1 = 18'sb100001011100001010;



  always @(posedge clk)
    begin : HwModeRegister1_process
      if (reset == 1'b1) begin
        coeff_a2_section1_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          coeff_a2_section1_1 <= coeff_a2_section1;
        end
      end
    end



  assign coeff_a3_section1 = 18'sb001110111000010100;



  always @(posedge clk)
    begin : HwModeRegister3_process
      if (reset == 1'b1) begin
        coeff_a3_section1_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          coeff_a3_section1_1 <= coeff_a3_section1;
        end
      end
    end



  always @(posedge clk)
    begin : dendelay_process_section1_tap1_process
      if (reset == 1'b1) begin
        a1sum1_tap1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb_1_320_0) begin
          a1sum1_tap1 <= a1sum1;
        end
      end
    end



  assign a1sum1_tap1_1 = a1sum1_tap1;

  always @(posedge clk)
    begin : HwModeRegister2_process
      if (reset == 1'b1) begin
        dendelay_section1_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          dendelay_section1_1 <= a1sum1_tap1_1;
        end
      end
    end



  assign multiplier1_mul_temp = dendelay_section1_1 * coeff_a3_section1_1;
  assign a3mul1 = multiplier1_mul_temp[33:16];



  always @(posedge clk)
    begin : PipelineRegister1_process
      if (reset == 1'b1) begin
        a3mul1_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          a3mul1_1 <= a3mul1;
        end
      end
    end



  always @(posedge clk)
    begin : dendelay_process_section1_tap0_bypass_process
      if (reset == 1'b1) begin
        dendelay_process_section1_tap0_bypass_reg <= 18'sb000000000000000000;
      end
      else begin
        if (enb_1_320_1) begin
          dendelay_process_section1_tap0_bypass_reg <= a1sum1_1;
        end
      end
    end

  assign a1sum1 = (enb_1_320_1 == 1'b1 ? a1sum1_1 :
              dendelay_process_section1_tap0_bypass_reg);



  assign a1sum1_tap0 = a1sum1;

  always @(posedge clk)
    begin : HwModeRegister_process
      if (reset == 1'b1) begin
        dendelay_section1_0 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          dendelay_section1_0 <= a1sum1_tap0;
        end
      end
    end



  assign multiplier_mul_temp = dendelay_section1_0 * coeff_a2_section1_1;
  assign a2mul1 = multiplier_mul_temp[33:16];



  always @(posedge clk)
    begin : PipelineRegister_process
      if (reset == 1'b1) begin
        a2mul1_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          a2mul1_1 <= a2mul1;
        end
      end
    end



  assign Sub1_sub_cast = {b2sum1[17], b2sum1};
  assign Sub1_sub_cast_1 = {a2mul1_1[17], a2mul1_1};
  assign a2sum1_temp = Sub1_sub_cast - Sub1_sub_cast_1;



  assign a2sum1 = a2sum1_temp[17:0];



  assign Sub_sub_cast = {a2sum1[17], a2sum1};
  assign Sub_sub_cast_1 = {a3mul1_1[17], a3mul1_1};
  assign a1sum1_temp = Sub_sub_cast - Sub_sub_cast_1;



  assign a1sum1_1 = a1sum1_temp[17:0];



  assign output_typeconvert = a1sum1_1[17:2];



  always @(posedge clk)
    begin : t4_bypass_process
      if (reset == 1'b1) begin
        t4_bypass_reg <= 16'sb0000000000000000;
      end
      else begin
        if (enb_1_320_1) begin
          t4_bypass_reg <= output_typeconvert;
        end
      end
    end

  assign I_out = (enb_1_320_1 == 1'b1 ? output_typeconvert :
              t4_bypass_reg);



  assign Q_in_1 = Q_in;

  always @(posedge clk)
    begin : HwModeRegister10_process
      if (reset == 1'b1) begin
        Q_in_2 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          Q_in_2 <= Q_in_1;
        end
      end
    end



  assign scaleconst1_2 = 18'sb010100011110101110;



  always @(posedge clk)
    begin : HwModeRegister11_process
      if (reset == 1'b1) begin
        scaleconst1_3 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          scaleconst1_3 <= scaleconst1_2;
        end
      end
    end



  assign scale1_2 = Q_in_2 * scaleconst1_3;



  always @(posedge clk)
    begin : PipelineRegister5_process
      if (reset == 1'b1) begin
        scale1_3 <= 36'sh000000000;
      end
      else begin
        if (enb) begin
          scale1_3 <= scale1_2;
        end
      end
    end



  assign numtypeconvert1_2 = {{6{scale1_3[35]}}, scale1_3[35:24]};



  // coeff_b1_section1
  assign gain5_cast = {{3{numtypeconvert1_2[17]}}, {numtypeconvert1_2, 15'b000000000000000}};
  assign b1mul1_1 = gain5_cast[34:17];



  always @(posedge clk)
    begin : numdelay_process_section1_tap01_bypass_process
      if (reset == 1'b1) begin
        numdelay_process_section1_tap01_bypass_reg <= 18'sb000000000000000000;
      end
      else begin
        if (enb_1_320_1) begin
          numdelay_process_section1_tap01_bypass_reg <= numtypeconvert1_2;
        end
      end
    end

  assign numtypeconvert1_3 = (enb_1_320_1 == 1'b1 ? numtypeconvert1_2 :
              numdelay_process_section1_tap01_bypass_reg);



  assign numtypeconvert1_tap0_1 = numtypeconvert1_3;

  // coeff_b2_section1
  assign gain3_cast = {{2{numtypeconvert1_tap0_1[17]}}, {numtypeconvert1_tap0_1, 16'b0000000000000000}};
  assign b2mul1_2 = gain3_cast[34:17];



  always @(posedge clk)
    begin : delayMatch2_process
      if (reset == 1'b1) begin
        b2mul1_3 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          b2mul1_3 <= b2mul1_2;
        end
      end
    end



  assign b1sum1_1 = b1mul1_1 + b2mul1_3;



  always @(posedge clk)
    begin : numdelay_process_section1_tap11_process
      if (reset == 1'b1) begin
        numtypeconvert1_tap1_2 <= 18'sb000000000000000000;
      end
      else begin
        if (enb_1_320_0) begin
          numtypeconvert1_tap1_2 <= numtypeconvert1_3;
        end
      end
    end



  assign numtypeconvert1_tap1_3 = numtypeconvert1_tap1_2;

  // coeff_b3_section1
  assign gain4_cast = {{3{numtypeconvert1_tap1_3[17]}}, {numtypeconvert1_tap1_3, 15'b000000000000000}};
  assign b3mul1_2 = gain4_cast[34:17];



  always @(posedge clk)
    begin : delayMatch3_process
      if (reset == 1'b1) begin
        b3mul1_3 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          b3mul1_3 <= b3mul1_2;
        end
      end
    end



  assign b2sum1_1 = b1sum1_1 + b3mul1_3;



  assign coeff_a2_section1_2 = 18'sb100001011100001010;



  always @(posedge clk)
    begin : HwModeRegister7_process
      if (reset == 1'b1) begin
        coeff_a2_section1_3 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          coeff_a2_section1_3 <= coeff_a2_section1_2;
        end
      end
    end



  assign coeff_a3_section1_2 = 18'sb001110111000010100;



  always @(posedge clk)
    begin : HwModeRegister9_process
      if (reset == 1'b1) begin
        coeff_a3_section1_3 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          coeff_a3_section1_3 <= coeff_a3_section1_2;
        end
      end
    end



  always @(posedge clk)
    begin : dendelay_process_section1_tap11_process
      if (reset == 1'b1) begin
        a1sum1_tap1_2 <= 18'sb000000000000000000;
      end
      else begin
        if (enb_1_320_0) begin
          a1sum1_tap1_2 <= a1sum1_2;
        end
      end
    end



  assign a1sum1_tap1_3 = a1sum1_tap1_2;

  always @(posedge clk)
    begin : HwModeRegister8_process
      if (reset == 1'b1) begin
        dendelay_section1_1_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          dendelay_section1_1_1 <= a1sum1_tap1_3;
        end
      end
    end



  assign multiplier4_mul_temp = dendelay_section1_1_1 * coeff_a3_section1_3;
  assign a3mul1_2 = multiplier4_mul_temp[33:16];



  always @(posedge clk)
    begin : PipelineRegister4_process
      if (reset == 1'b1) begin
        a3mul1_3 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          a3mul1_3 <= a3mul1_2;
        end
      end
    end



  always @(posedge clk)
    begin : dendelay_process_section1_tap01_bypass_process
      if (reset == 1'b1) begin
        dendelay_process_section1_tap01_bypass_reg <= 18'sb000000000000000000;
      end
      else begin
        if (enb_1_320_1) begin
          dendelay_process_section1_tap01_bypass_reg <= a1sum1_3;
        end
      end
    end

  assign a1sum1_2 = (enb_1_320_1 == 1'b1 ? a1sum1_3 :
              dendelay_process_section1_tap01_bypass_reg);



  assign a1sum1_tap0_1 = a1sum1_2;

  always @(posedge clk)
    begin : HwModeRegister6_process
      if (reset == 1'b1) begin
        dendelay_section1_0_1 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          dendelay_section1_0_1 <= a1sum1_tap0_1;
        end
      end
    end



  assign multiplier3_mul_temp = dendelay_section1_0_1 * coeff_a2_section1_3;
  assign a2mul1_2 = multiplier3_mul_temp[33:16];



  always @(posedge clk)
    begin : PipelineRegister3_process
      if (reset == 1'b1) begin
        a2mul1_3 <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          a2mul1_3 <= a2mul1_2;
        end
      end
    end



  assign Sub3_sub_cast = {b2sum1_1[17], b2sum1_1};
  assign Sub3_sub_cast_1 = {a2mul1_3[17], a2mul1_3};
  assign a2sum1_temp_1 = Sub3_sub_cast - Sub3_sub_cast_1;



  assign a2sum1_1 = a2sum1_temp_1[17:0];



  assign Sub2_sub_cast = {a2sum1_1[17], a2sum1_1};
  assign Sub2_sub_cast_1 = {a3mul1_3[17], a3mul1_3};
  assign a1sum1_temp_1 = Sub2_sub_cast - Sub2_sub_cast_1;



  assign a1sum1_3 = a1sum1_temp_1[17:0];



  assign output_typeconvert_1 = a1sum1_3[17:2];



  always @(posedge clk)
    begin : t9_bypass_process
      if (reset == 1'b1) begin
        t9_bypass_reg <= 16'sb0000000000000000;
      end
      else begin
        if (enb_1_320_1) begin
          t9_bypass_reg <= output_typeconvert_1;
        end
      end
    end

  assign Q_out = (enb_1_320_1 == 1'b1 ? output_typeconvert_1 :
              t9_bypass_reg);



endmodule  // Channel4_Filter

