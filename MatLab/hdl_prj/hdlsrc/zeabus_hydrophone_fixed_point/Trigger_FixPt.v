// -------------------------------------------------------------
// 
// File Name: hdl_prj\hdlsrc\zeabus_hydrophone_fixed_point\Trigger_FixPt.v
// Created: 2020-04-16 11:11:01
// 
// Generated by MATLAB 9.7 and HDL Coder 3.15
// 
// -------------------------------------------------------------


// -------------------------------------------------------------
// 
// Module: Trigger_FixPt
// Source Path: zeabus_hydrophone_fixed_point/Hydrophone processing/Trigger/Trigger_FixPt
// Hierarchy Level: 2
// 
// -------------------------------------------------------------

`timescale 10 ns / 10 ns

module Trigger_FixPt
          (clk,
           reset,
           enb_1_64_0,
           i1,
           i2,
           i3,
           i4,
           threshold,
           o1,
           o2,
           o3,
           o4,
           trigged);


  input   clk;
  input   reset;
  input   enb_1_64_0;
  input   signed [17:0] i1;  // sfix18_En17
  input   signed [17:0] i2;  // sfix18_En17
  input   signed [17:0] i3;  // sfix18_En17
  input   signed [17:0] i4;  // sfix18_En17
  input   [15:0] threshold;  // ufix16_En16
  output  signed [17:0] o1;  // sfix18_En17
  output  signed [17:0] o2;  // sfix18_En17
  output  signed [17:0] o3;  // sfix18_En17
  output  signed [17:0] o4;  // sfix18_En17
  output  [7:0] trigged;  // uint8


  reg [15:0] threshold_1;  // ufix16_En16
  reg signed [17:0] o1_1;  // sfix18_En17
  reg signed [17:0] o2_1;  // sfix18_En17
  reg signed [17:0] o3_1;  // sfix18_En17
  reg signed [17:0] o4_1;  // sfix18_En17
  reg [7:0] trigged_1;  // uint8
  reg [6:0] after;  // ufix7
  reg [6:0] before_rsvd;  // ufix7
  reg signed [17:0] fifo [0:255];  // sfix18 [256]
  reg [6:0] after_next;  // ufix7
  reg [6:0] before_next;  // ufix7
  reg signed [17:0] fifo_next [0:255];  // sfix18_En17 [256]
  reg signed [17:0] d1;  // sfix18_En17
  reg signed [17:0] d2;  // sfix18_En17
  reg signed [17:0] d3;  // sfix18_En17
  reg signed [17:0] d4;  // sfix18_En17
  reg signed [17:0] y;  // sfix18_En17
  reg signed [17:0] y_0;  // sfix18_En17
  reg signed [17:0] y_1;  // sfix18_En17
  reg signed [17:0] y_2;  // sfix18_En17
  reg  guard1;
  reg [6:0] before_temp;  // ufix7
  reg signed [17:0] fifo_temp [0:255];  // sfix18_En17 [256]
  reg signed [17:0] fifo_temp_0 [0:251];  // sfix18_En17 [252]
  reg signed [17:0] fifo_temp_1 [0:251];  // sfix18_En17 [252]
  reg signed [17:0] fifo_temp_2 [0:251];  // sfix18_En17 [252]
  reg signed [31:0] t_0;  // int32
  reg signed [31:0] t_1;  // int32
  reg signed [31:0] t_2;  // int32
  reg signed [31:0] t_3;  // int32
  reg signed [31:0] t_4;  // int32
  reg signed [31:0] t_5;  // int32
  reg signed [31:0] t_6;  // int32
  reg signed [31:0] t_7;  // int32
  reg signed [18:0] cast;  // sfix19_En17
  reg signed [18:0] cast_0;  // sfix19_En17
  reg signed [17:0] cast_1;  // sfix18_En17
  reg signed [18:0] cast_2;  // sfix19_En17
  reg signed [18:0] cast_3;  // sfix19_En17
  reg signed [17:0] cast_4;  // sfix18_En17
  reg signed [18:0] cast_5;  // sfix19_En17
  reg signed [18:0] cast_6;  // sfix19_En17
  reg signed [17:0] cast_7;  // sfix18_En17
  reg signed [18:0] cast_8;  // sfix19_En17
  reg signed [18:0] cast_9;  // sfix19_En17
  reg signed [17:0] cast_10;  // sfix18_En17
  reg signed [63:0] add_cast [0:3];  // sfix64 [4]
  reg signed [63:0] add_cast_0 [0:3];  // sfix64 [4]
  reg signed [63:0] add_cast_1 [0:3];  // sfix64 [4]
  reg signed [63:0] add_cast_2 [0:3];  // sfix64 [4]
  reg signed [63:0] add_cast_3 [0:3];  // sfix64 [4]
  reg signed [63:0] add_cast_4 [0:3];  // sfix64 [4]
  reg signed [63:0] add_cast_5 [0:3];  // sfix64 [4]
  reg signed [63:0] add_cast_6 [0:3];  // sfix64 [4]
  reg signed [63:0] add_cast_7 [0:3];  // sfix64 [4]
  reg signed [31:0] t_0_0;  // int32


  always @(posedge clk)
    begin : delayMatch_process
      if (reset == 1'b1) begin
        threshold_1 <= 16'b0000000000000000;
      end
      else begin
        if (enb_1_64_0) begin
          threshold_1 <= threshold;
        end
      end
    end



  always @(posedge clk)
    begin : Trigger_FixPt_1_process
      if (reset == 1'b1) begin
        after <= 7'b0000000;
        before_rsvd <= 7'b0000001;
        fifo[0] <= 18'sb000000000000000000;
        fifo[1] <= 18'sb000000000000000000;
        fifo[2] <= 18'sb000000000000000000;
        fifo[3] <= 18'sb000000000000000000;
        fifo[4] <= 18'sb000000000000000000;
        fifo[5] <= 18'sb000000000000000000;
        fifo[6] <= 18'sb000000000000000000;
        fifo[7] <= 18'sb000000000000000000;
        fifo[8] <= 18'sb000000000000000000;
        fifo[9] <= 18'sb000000000000000000;
        fifo[10] <= 18'sb000000000000000000;
        fifo[11] <= 18'sb000000000000000000;
        fifo[12] <= 18'sb000000000000000000;
        fifo[13] <= 18'sb000000000000000000;
        fifo[14] <= 18'sb000000000000000000;
        fifo[15] <= 18'sb000000000000000000;
        fifo[16] <= 18'sb000000000000000000;
        fifo[17] <= 18'sb000000000000000000;
        fifo[18] <= 18'sb000000000000000000;
        fifo[19] <= 18'sb000000000000000000;
        fifo[20] <= 18'sb000000000000000000;
        fifo[21] <= 18'sb000000000000000000;
        fifo[22] <= 18'sb000000000000000000;
        fifo[23] <= 18'sb000000000000000000;
        fifo[24] <= 18'sb000000000000000000;
        fifo[25] <= 18'sb000000000000000000;
        fifo[26] <= 18'sb000000000000000000;
        fifo[27] <= 18'sb000000000000000000;
        fifo[28] <= 18'sb000000000000000000;
        fifo[29] <= 18'sb000000000000000000;
        fifo[30] <= 18'sb000000000000000000;
        fifo[31] <= 18'sb000000000000000000;
        fifo[32] <= 18'sb000000000000000000;
        fifo[33] <= 18'sb000000000000000000;
        fifo[34] <= 18'sb000000000000000000;
        fifo[35] <= 18'sb000000000000000000;
        fifo[36] <= 18'sb000000000000000000;
        fifo[37] <= 18'sb000000000000000000;
        fifo[38] <= 18'sb000000000000000000;
        fifo[39] <= 18'sb000000000000000000;
        fifo[40] <= 18'sb000000000000000000;
        fifo[41] <= 18'sb000000000000000000;
        fifo[42] <= 18'sb000000000000000000;
        fifo[43] <= 18'sb000000000000000000;
        fifo[44] <= 18'sb000000000000000000;
        fifo[45] <= 18'sb000000000000000000;
        fifo[46] <= 18'sb000000000000000000;
        fifo[47] <= 18'sb000000000000000000;
        fifo[48] <= 18'sb000000000000000000;
        fifo[49] <= 18'sb000000000000000000;
        fifo[50] <= 18'sb000000000000000000;
        fifo[51] <= 18'sb000000000000000000;
        fifo[52] <= 18'sb000000000000000000;
        fifo[53] <= 18'sb000000000000000000;
        fifo[54] <= 18'sb000000000000000000;
        fifo[55] <= 18'sb000000000000000000;
        fifo[56] <= 18'sb000000000000000000;
        fifo[57] <= 18'sb000000000000000000;
        fifo[58] <= 18'sb000000000000000000;
        fifo[59] <= 18'sb000000000000000000;
        fifo[60] <= 18'sb000000000000000000;
        fifo[61] <= 18'sb000000000000000000;
        fifo[62] <= 18'sb000000000000000000;
        fifo[63] <= 18'sb000000000000000000;
        fifo[64] <= 18'sb000000000000000000;
        fifo[65] <= 18'sb000000000000000000;
        fifo[66] <= 18'sb000000000000000000;
        fifo[67] <= 18'sb000000000000000000;
        fifo[68] <= 18'sb000000000000000000;
        fifo[69] <= 18'sb000000000000000000;
        fifo[70] <= 18'sb000000000000000000;
        fifo[71] <= 18'sb000000000000000000;
        fifo[72] <= 18'sb000000000000000000;
        fifo[73] <= 18'sb000000000000000000;
        fifo[74] <= 18'sb000000000000000000;
        fifo[75] <= 18'sb000000000000000000;
        fifo[76] <= 18'sb000000000000000000;
        fifo[77] <= 18'sb000000000000000000;
        fifo[78] <= 18'sb000000000000000000;
        fifo[79] <= 18'sb000000000000000000;
        fifo[80] <= 18'sb000000000000000000;
        fifo[81] <= 18'sb000000000000000000;
        fifo[82] <= 18'sb000000000000000000;
        fifo[83] <= 18'sb000000000000000000;
        fifo[84] <= 18'sb000000000000000000;
        fifo[85] <= 18'sb000000000000000000;
        fifo[86] <= 18'sb000000000000000000;
        fifo[87] <= 18'sb000000000000000000;
        fifo[88] <= 18'sb000000000000000000;
        fifo[89] <= 18'sb000000000000000000;
        fifo[90] <= 18'sb000000000000000000;
        fifo[91] <= 18'sb000000000000000000;
        fifo[92] <= 18'sb000000000000000000;
        fifo[93] <= 18'sb000000000000000000;
        fifo[94] <= 18'sb000000000000000000;
        fifo[95] <= 18'sb000000000000000000;
        fifo[96] <= 18'sb000000000000000000;
        fifo[97] <= 18'sb000000000000000000;
        fifo[98] <= 18'sb000000000000000000;
        fifo[99] <= 18'sb000000000000000000;
        fifo[100] <= 18'sb000000000000000000;
        fifo[101] <= 18'sb000000000000000000;
        fifo[102] <= 18'sb000000000000000000;
        fifo[103] <= 18'sb000000000000000000;
        fifo[104] <= 18'sb000000000000000000;
        fifo[105] <= 18'sb000000000000000000;
        fifo[106] <= 18'sb000000000000000000;
        fifo[107] <= 18'sb000000000000000000;
        fifo[108] <= 18'sb000000000000000000;
        fifo[109] <= 18'sb000000000000000000;
        fifo[110] <= 18'sb000000000000000000;
        fifo[111] <= 18'sb000000000000000000;
        fifo[112] <= 18'sb000000000000000000;
        fifo[113] <= 18'sb000000000000000000;
        fifo[114] <= 18'sb000000000000000000;
        fifo[115] <= 18'sb000000000000000000;
        fifo[116] <= 18'sb000000000000000000;
        fifo[117] <= 18'sb000000000000000000;
        fifo[118] <= 18'sb000000000000000000;
        fifo[119] <= 18'sb000000000000000000;
        fifo[120] <= 18'sb000000000000000000;
        fifo[121] <= 18'sb000000000000000000;
        fifo[122] <= 18'sb000000000000000000;
        fifo[123] <= 18'sb000000000000000000;
        fifo[124] <= 18'sb000000000000000000;
        fifo[125] <= 18'sb000000000000000000;
        fifo[126] <= 18'sb000000000000000000;
        fifo[127] <= 18'sb000000000000000000;
        fifo[128] <= 18'sb000000000000000000;
        fifo[129] <= 18'sb000000000000000000;
        fifo[130] <= 18'sb000000000000000000;
        fifo[131] <= 18'sb000000000000000000;
        fifo[132] <= 18'sb000000000000000000;
        fifo[133] <= 18'sb000000000000000000;
        fifo[134] <= 18'sb000000000000000000;
        fifo[135] <= 18'sb000000000000000000;
        fifo[136] <= 18'sb000000000000000000;
        fifo[137] <= 18'sb000000000000000000;
        fifo[138] <= 18'sb000000000000000000;
        fifo[139] <= 18'sb000000000000000000;
        fifo[140] <= 18'sb000000000000000000;
        fifo[141] <= 18'sb000000000000000000;
        fifo[142] <= 18'sb000000000000000000;
        fifo[143] <= 18'sb000000000000000000;
        fifo[144] <= 18'sb000000000000000000;
        fifo[145] <= 18'sb000000000000000000;
        fifo[146] <= 18'sb000000000000000000;
        fifo[147] <= 18'sb000000000000000000;
        fifo[148] <= 18'sb000000000000000000;
        fifo[149] <= 18'sb000000000000000000;
        fifo[150] <= 18'sb000000000000000000;
        fifo[151] <= 18'sb000000000000000000;
        fifo[152] <= 18'sb000000000000000000;
        fifo[153] <= 18'sb000000000000000000;
        fifo[154] <= 18'sb000000000000000000;
        fifo[155] <= 18'sb000000000000000000;
        fifo[156] <= 18'sb000000000000000000;
        fifo[157] <= 18'sb000000000000000000;
        fifo[158] <= 18'sb000000000000000000;
        fifo[159] <= 18'sb000000000000000000;
        fifo[160] <= 18'sb000000000000000000;
        fifo[161] <= 18'sb000000000000000000;
        fifo[162] <= 18'sb000000000000000000;
        fifo[163] <= 18'sb000000000000000000;
        fifo[164] <= 18'sb000000000000000000;
        fifo[165] <= 18'sb000000000000000000;
        fifo[166] <= 18'sb000000000000000000;
        fifo[167] <= 18'sb000000000000000000;
        fifo[168] <= 18'sb000000000000000000;
        fifo[169] <= 18'sb000000000000000000;
        fifo[170] <= 18'sb000000000000000000;
        fifo[171] <= 18'sb000000000000000000;
        fifo[172] <= 18'sb000000000000000000;
        fifo[173] <= 18'sb000000000000000000;
        fifo[174] <= 18'sb000000000000000000;
        fifo[175] <= 18'sb000000000000000000;
        fifo[176] <= 18'sb000000000000000000;
        fifo[177] <= 18'sb000000000000000000;
        fifo[178] <= 18'sb000000000000000000;
        fifo[179] <= 18'sb000000000000000000;
        fifo[180] <= 18'sb000000000000000000;
        fifo[181] <= 18'sb000000000000000000;
        fifo[182] <= 18'sb000000000000000000;
        fifo[183] <= 18'sb000000000000000000;
        fifo[184] <= 18'sb000000000000000000;
        fifo[185] <= 18'sb000000000000000000;
        fifo[186] <= 18'sb000000000000000000;
        fifo[187] <= 18'sb000000000000000000;
        fifo[188] <= 18'sb000000000000000000;
        fifo[189] <= 18'sb000000000000000000;
        fifo[190] <= 18'sb000000000000000000;
        fifo[191] <= 18'sb000000000000000000;
        fifo[192] <= 18'sb000000000000000000;
        fifo[193] <= 18'sb000000000000000000;
        fifo[194] <= 18'sb000000000000000000;
        fifo[195] <= 18'sb000000000000000000;
        fifo[196] <= 18'sb000000000000000000;
        fifo[197] <= 18'sb000000000000000000;
        fifo[198] <= 18'sb000000000000000000;
        fifo[199] <= 18'sb000000000000000000;
        fifo[200] <= 18'sb000000000000000000;
        fifo[201] <= 18'sb000000000000000000;
        fifo[202] <= 18'sb000000000000000000;
        fifo[203] <= 18'sb000000000000000000;
        fifo[204] <= 18'sb000000000000000000;
        fifo[205] <= 18'sb000000000000000000;
        fifo[206] <= 18'sb000000000000000000;
        fifo[207] <= 18'sb000000000000000000;
        fifo[208] <= 18'sb000000000000000000;
        fifo[209] <= 18'sb000000000000000000;
        fifo[210] <= 18'sb000000000000000000;
        fifo[211] <= 18'sb000000000000000000;
        fifo[212] <= 18'sb000000000000000000;
        fifo[213] <= 18'sb000000000000000000;
        fifo[214] <= 18'sb000000000000000000;
        fifo[215] <= 18'sb000000000000000000;
        fifo[216] <= 18'sb000000000000000000;
        fifo[217] <= 18'sb000000000000000000;
        fifo[218] <= 18'sb000000000000000000;
        fifo[219] <= 18'sb000000000000000000;
        fifo[220] <= 18'sb000000000000000000;
        fifo[221] <= 18'sb000000000000000000;
        fifo[222] <= 18'sb000000000000000000;
        fifo[223] <= 18'sb000000000000000000;
        fifo[224] <= 18'sb000000000000000000;
        fifo[225] <= 18'sb000000000000000000;
        fifo[226] <= 18'sb000000000000000000;
        fifo[227] <= 18'sb000000000000000000;
        fifo[228] <= 18'sb000000000000000000;
        fifo[229] <= 18'sb000000000000000000;
        fifo[230] <= 18'sb000000000000000000;
        fifo[231] <= 18'sb000000000000000000;
        fifo[232] <= 18'sb000000000000000000;
        fifo[233] <= 18'sb000000000000000000;
        fifo[234] <= 18'sb000000000000000000;
        fifo[235] <= 18'sb000000000000000000;
        fifo[236] <= 18'sb000000000000000000;
        fifo[237] <= 18'sb000000000000000000;
        fifo[238] <= 18'sb000000000000000000;
        fifo[239] <= 18'sb000000000000000000;
        fifo[240] <= 18'sb000000000000000000;
        fifo[241] <= 18'sb000000000000000000;
        fifo[242] <= 18'sb000000000000000000;
        fifo[243] <= 18'sb000000000000000000;
        fifo[244] <= 18'sb000000000000000000;
        fifo[245] <= 18'sb000000000000000000;
        fifo[246] <= 18'sb000000000000000000;
        fifo[247] <= 18'sb000000000000000000;
        fifo[248] <= 18'sb000000000000000000;
        fifo[249] <= 18'sb000000000000000000;
        fifo[250] <= 18'sb000000000000000000;
        fifo[251] <= 18'sb000000000000000000;
        fifo[252] <= 18'sb000000000000000000;
        fifo[253] <= 18'sb000000000000000000;
        fifo[254] <= 18'sb000000000000000000;
        fifo[255] <= 18'sb000000000000000000;
      end
      else begin
        if (enb_1_64_0) begin
          after <= after_next;
          before_rsvd <= before_next;

          for(t_0_0 = 32'sd0; t_0_0 <= 32'sd255; t_0_0 = t_0_0 + 32'sd1) begin
            fifo[t_0_0] <= fifo_next[t_0_0];
          end

        end
      end
    end

  always @* begin
    before_temp = before_rsvd;

    for(t_0 = 32'sd0; t_0 <= 32'sd255; t_0 = t_0 + 32'sd1) begin
      fifo_temp[t_0] = fifo[t_0];
    end

    after_next = after;
    //auto-generated
    // Enqueue channel 1
    fifo_temp[32'sd4 * ($signed({1'b0, before_rsvd}) - 32'sd1)] = i1;
    d1 = fifo_temp[0];
    // Enqueue channel 2
    fifo_temp[32'sd1 + (32'sd4 * ($signed({1'b0, before_rsvd}) - 32'sd1))] = i2;
    d2 = fifo_temp[1];
    // Enqueue channel 3
    fifo_temp[32'sd2 + (32'sd4 * ($signed({1'b0, before_rsvd}) - 32'sd1))] = i3;
    d3 = fifo_temp[2];
    // Enqueue channel 4
    fifo_temp[32'sd3 + (32'sd4 * ($signed({1'b0, before_rsvd}) - 32'sd1))] = i4;
    d4 = fifo_temp[3];
    if (i1 < 18'sb000000000000000000) begin
      cast = {i1[17], i1};
      cast_0 =  - (cast);
      y = cast_0[17:0];
    end
    else begin
      y = i1;
    end
    guard1 = 1'b0;
    cast_1 = {1'b0, {threshold_1, 1'b0}};
    if (y >= cast_1) begin
      guard1 = 1'b1;
    end
    else begin
      if (i2 < 18'sb000000000000000000) begin
        cast_2 = {i2[17], i2};
        cast_3 =  - (cast_2);
        y_0 = cast_3[17:0];
      end
      else begin
        y_0 = i2;
      end
      cast_4 = {1'b0, {threshold_1, 1'b0}};
      if (y_0 >= cast_4) begin
        guard1 = 1'b1;
      end
      else begin
        if (i3 < 18'sb000000000000000000) begin
          cast_5 = {i3[17], i3};
          cast_6 =  - (cast_5);
          y_1 = cast_6[17:0];
        end
        else begin
          y_1 = i3;
        end
        cast_7 = {1'b0, {threshold_1, 1'b0}};
        if (y_1 >= cast_7) begin
          guard1 = 1'b1;
        end
        else begin
          if (i4 < 18'sb000000000000000000) begin
            cast_8 = {i4[17], i4};
            cast_9 =  - (cast_8);
            y_2 = cast_9[17:0];
          end
          else begin
            y_2 = i4;
          end
          cast_10 = {1'b0, {threshold_1, 1'b0}};
          if (y_2 >= cast_10) begin
            guard1 = 1'b1;
          end
          else if (after > 7'b0000000) begin
            o1_1 = d1;
            o2_1 = d2;
            o3_1 = d3;
            o4_1 = d4;
            trigged_1 = 8'b00000001;
            after_next = after - 7'b0000001;
            if (before_rsvd >= 7'b0000010) begin

              for(t_7 = 32'sd0; t_7 <= 32'sd62; t_7 = t_7 + 32'sd1) begin
                for(t_6 = 32'sd0; t_6 <= 32'sd3; t_6 = t_6 + 32'sd1) begin
                  add_cast_5[t_6] = {{30{t_7[31]}}, {t_7, 2'b00}};
                  fifo_temp_2[t_6 + add_cast_5[t_6]] = fifo_temp[t_6 + (32'sd4 * (32'sd1 + t_7))];
                  add_cast_6[t_6] = {{30{t_7[31]}}, {t_7, 2'b00}};
                  add_cast_7[t_6] = {{30{t_7[31]}}, {t_7, 2'b00}};
                  fifo_temp[t_6 + add_cast_6[t_6]] = fifo_temp_2[t_6 + add_cast_7[t_6]];
                end
              end

            end
          end
          else begin
            o1_1 = 18'sb000000000000000000;
            o2_1 = 18'sb000000000000000000;
            o3_1 = 18'sb000000000000000000;
            o4_1 = 18'sb000000000000000000;
            trigged_1 = 8'b00000000;
            if (before_rsvd < 7'b1000000) begin
              before_temp = before_rsvd + 7'b0000001;
            end
            else begin

              for(t_5 = 32'sd0; t_5 <= 32'sd62; t_5 = t_5 + 32'sd1) begin
                for(t_4 = 32'sd0; t_4 <= 32'sd3; t_4 = t_4 + 32'sd1) begin
                  add_cast_2[t_4] = {{30{t_5[31]}}, {t_5, 2'b00}};
                  fifo_temp_1[t_4 + add_cast_2[t_4]] = fifo_temp[t_4 + (32'sd4 * (32'sd1 + t_5))];
                  add_cast_3[t_4] = {{30{t_5[31]}}, {t_5, 2'b00}};
                  add_cast_4[t_4] = {{30{t_5[31]}}, {t_5, 2'b00}};
                  fifo_temp[t_4 + add_cast_3[t_4]] = fifo_temp_1[t_4 + add_cast_4[t_4]];
                end
              end

            end
          end
        end
      end
    end
    if (guard1) begin
      o1_1 = d1;
      o2_1 = d2;
      o3_1 = d3;
      o4_1 = d4;
      trigged_1 = 8'b00000001;
      after_next = 7'b1000000;
      if (before_temp >= 7'b0000010) begin

        for(t_3 = 32'sd0; t_3 <= 32'sd62; t_3 = t_3 + 32'sd1) begin
          for(t_2 = 32'sd0; t_2 <= 32'sd3; t_2 = t_2 + 32'sd1) begin
            add_cast[t_2] = {{30{t_3[31]}}, {t_3, 2'b00}};
            fifo_temp_0[t_2 + add_cast[t_2]] = fifo_temp[t_2 + (32'sd4 * (32'sd1 + t_3))];
            add_cast_0[t_2] = {{30{t_3[31]}}, {t_3, 2'b00}};
            add_cast_1[t_2] = {{30{t_3[31]}}, {t_3, 2'b00}};
            fifo_temp[t_2 + add_cast_0[t_2]] = fifo_temp_0[t_2 + add_cast_1[t_2]];
          end
        end

      end
    end
    before_next = before_temp;

    for(t_1 = 32'sd0; t_1 <= 32'sd255; t_1 = t_1 + 32'sd1) begin
      fifo_next[t_1] = fifo_temp[t_1];
    end

  end



  assign o1 = o1_1;

  assign o2 = o2_1;

  assign o3 = o3_1;

  assign o4 = o4_1;

  assign trigged = trigged_1;

endmodule  // Trigger_FixPt

