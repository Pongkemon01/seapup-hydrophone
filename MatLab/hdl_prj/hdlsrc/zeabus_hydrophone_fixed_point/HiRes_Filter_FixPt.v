// -------------------------------------------------------------
// 
// File Name: hdl_prj\hdlsrc\zeabus_hydrophone_fixed_point\HiRes_Filter_FixPt.v
// Created: 2020-04-16 11:11:01
// 
// Generated by MATLAB 9.7 and HDL Coder 3.15
// 
// -------------------------------------------------------------


// -------------------------------------------------------------
// 
// Module: HiRes_Filter_FixPt
// Source Path: zeabus_hydrophone_fixed_point/Hydrophone processing/ADC1 Interface/HiRes Filter/HiRes Filter_FixPt
// Hierarchy Level: 3
// 
// -------------------------------------------------------------

`timescale 10 ns / 10 ns

module HiRes_Filter_FixPt
          (clk,
           reset,
           enb,
           u,
           y);


  input   clk;
  input   reset;
  input   enb;
  input   signed [17:0] u;  // sfix18_En17
  output  signed [17:0] y;  // sfix18_En17


  reg signed [17:0] y_1;  // sfix18_En17
  reg signed [23:0] acc;  // sfix24
  reg [6:0] counter;  // ufix7
  reg signed [17:0] avg;  // sfix18
  reg signed [23:0] acc_next;  // sfix24_En17
  reg [6:0] counter_next;  // ufix7
  reg signed [17:0] avg_next;  // sfix18_En17
  reg signed [29:0] a;  // sfix30_En23
  reg signed [29:0] c;  // sfix30_En23
  reg signed [23:0] acc_temp;  // sfix24_En17
  reg [6:0] counter_temp;  // ufix7
  reg signed [17:0] avg_temp;  // sfix18_En17
  reg signed [23:0] add_cast;  // sfix24_En17


  always @(posedge clk)
    begin : HiRes_Filter_FixPt_1_process
      if (reset == 1'b1) begin
        acc <= 24'sb000000000000000000000000;
        counter <= 7'b0000000;
        avg <= 18'sb000000000000000000;
      end
      else begin
        if (enb) begin
          acc <= acc_next;
          counter <= counter_next;
          avg <= avg_next;
        end
      end
    end

  always @(acc, avg, counter, u) begin
    avg_temp = avg;
    //auto-generated
    //set_param('Block', 'SampleTime', [-2 64]);
    add_cast = {{6{u[17]}}, u};
    acc_temp = acc + add_cast;
    // accumulate data
    counter_temp = counter + 7'b0000001;
    if (counter_temp == 7'b1000000) begin
      counter_temp = 7'b0000000;
      a = {acc_temp, 6'b000000};
      c = a >>> 8'd6;
      avg_temp = c[23:6];
      acc_temp = 24'sb000000000000000000000000;
    end
    y_1 = avg_temp;
    acc_next = acc_temp;
    counter_next = counter_temp;
    avg_next = avg_temp;
  end



  assign y = y_1;

endmodule  // HiRes_Filter_FixPt

