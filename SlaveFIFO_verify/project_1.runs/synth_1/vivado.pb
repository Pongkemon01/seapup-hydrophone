
x
Command: %s
53*	vivadotcl2G
3synth_design -top fifo_verify -part xc7a15tcsg324-12default:defaultZ4-113h px? 
:
Starting synth_design
149*	vivadotclZ4-321h px? 
?
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2default:default2
xc7a15t2default:defaultZ17-347h px? 
?
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2default:default2
xc7a15t2default:defaultZ17-349h px? 
V
Loading part %s157*device2#
xc7a15tcsg324-12default:defaultZ21-403h px? 
?
%s*synth2?
xStarting RTL Elaboration : Time (s): cpu = 00:00:05 ; elapsed = 00:00:05 . Memory (MB): peak = 713.004 ; gain = 176.984
2default:defaulth px? 
?
synthesizing module '%s'%s4497*oasys2
fifo_verify2default:default2
 2default:default2t
^C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/slavefifo_verify.v2default:default2
392default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2"
fx3s_interface2default:default2
 2default:default2r
\C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/fx3s_interface.v2default:default2
362default:default8@Z8-6157h px? 
e
%s
*synth2M
9	Parameter FX3S_DMA_Size bound to: 4095 - type: integer 
2default:defaulth p
x
? 
R
%s
*synth2:
&	Parameter addr_write bound to: 1'b1 
2default:defaulth p
x
? 
Q
%s
*synth29
%	Parameter addr_read bound to: 1'b0 
2default:defaulth p
x
? 
U
%s
*synth2=
)	Parameter state_idle bound to: 4'b0000 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter state_start_read bound to: 4'b0001 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter state_read_pre1 bound to: 4'b0010 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter state_read_pre2 bound to: 4'b0011 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter state_read_pre3 bound to: 4'b0100 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter state_read_loop bound to: 4'b0101 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter state_start_write bound to: 4'b1000 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter state_write_wait1 bound to: 4'b1001 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter state_write_wait2 bound to: 4'b1010 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter state_write_wait3 bound to: 4'b1011 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter state_write_wait4 bound to: 4'b0110 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter state_zlp bound to: 4'b1100 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter state_zlp_wait1 bound to: 4'b1101 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter state_zlp_wait2 bound to: 4'b1110 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter state_zlp_wait3 bound to: 4'b1111 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2+
fifo_departure_1024x64b2default:default2
 2default:default2?
?C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.runs/synth_1/.Xil/Vivado-4964-DESKTOP-MDIICNE/realtime/fifo_departure_1024x64b_stub.v2default:default2
62default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2+
fifo_departure_1024x64b2default:default2
 2default:default2
12default:default2
12default:default2?
?C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.runs/synth_1/.Xil/Vivado-4964-DESKTOP-MDIICNE/realtime/fifo_departure_1024x64b_stub.v2default:default2
62default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2'
fifo_arrival_64x16b2default:default2
 2default:default2?
?C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.runs/synth_1/.Xil/Vivado-4964-DESKTOP-MDIICNE/realtime/fifo_arrival_64x16b_stub.v2default:default2
62default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2'
fifo_arrival_64x16b2default:default2
 2default:default2
22default:default2
12default:default2?
?C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.runs/synth_1/.Xil/Vivado-4964-DESKTOP-MDIICNE/realtime/fifo_arrival_64x16b_stub.v2default:default2
62default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2"
fx3s_interface2default:default2
 2default:default2
32default:default2
12default:default2r
\C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/fx3s_interface.v2default:default2
362default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
BUFG2default:default2
 2default:default2K
5C:/Xilinx/Vivado/2019.1/scripts/rt/data/unisim_comp.v2default:default2
10752default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
BUFG2default:default2
 2default:default2
42default:default2
12default:default2K
5C:/Xilinx/Vivado/2019.1/scripts/rt/data/unisim_comp.v2default:default2
10752default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
	clk_wiz_02default:default2
 2default:default2?
nc:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.v2default:default2
712default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2%
clk_wiz_0_clk_wiz2default:default2
 2default:default2?
vc:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_clk_wiz.v2default:default2
692default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2
IBUF2default:default2
 2default:default2K
5C:/Xilinx/Vivado/2019.1/scripts/rt/data/unisim_comp.v2default:default2
328562default:default8@Z8-6157h px? 
g
%s
*synth2O
;	Parameter CAPACITANCE bound to: DONT_CARE - type: string 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter IBUF_DELAY_VALUE bound to: 0 - type: string 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter IBUF_LOW_PWR bound to: TRUE - type: string 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter IFD_DELAY_VALUE bound to: AUTO - type: string 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter IOSTANDARD bound to: DEFAULT - type: string 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
IBUF2default:default2
 2default:default2
52default:default2
12default:default2K
5C:/Xilinx/Vivado/2019.1/scripts/rt/data/unisim_comp.v2default:default2
328562default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
	PLLE2_ADV2default:default2
 2default:default2K
5C:/Xilinx/Vivado/2019.1/scripts/rt/data/unisim_comp.v2default:default2
611992default:default8@Z8-6157h px? 
`
%s
*synth2H
4	Parameter BANDWIDTH bound to: HIGH - type: string 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter CLKFBOUT_MULT bound to: 32 - type: integer 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter CLKFBOUT_PHASE bound to: 0.000000 - type: float 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter CLKIN1_PERIOD bound to: 38.462000 - type: float 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter CLKIN2_PERIOD bound to: 0.000000 - type: float 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter CLKOUT0_DIVIDE bound to: 13 - type: integer 
2default:defaulth p
x
? 
l
%s
*synth2T
@	Parameter CLKOUT0_DUTY_CYCLE bound to: 0.500000 - type: float 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter CLKOUT0_PHASE bound to: 0.000000 - type: float 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter CLKOUT1_DIVIDE bound to: 13 - type: integer 
2default:defaulth p
x
? 
l
%s
*synth2T
@	Parameter CLKOUT1_DUTY_CYCLE bound to: 0.500000 - type: float 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter CLKOUT1_PHASE bound to: 90.000000 - type: float 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter CLKOUT2_DIVIDE bound to: 1 - type: integer 
2default:defaulth p
x
? 
l
%s
*synth2T
@	Parameter CLKOUT2_DUTY_CYCLE bound to: 0.500000 - type: float 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter CLKOUT2_PHASE bound to: 0.000000 - type: float 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter CLKOUT3_DIVIDE bound to: 1 - type: integer 
2default:defaulth p
x
? 
l
%s
*synth2T
@	Parameter CLKOUT3_DUTY_CYCLE bound to: 0.500000 - type: float 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter CLKOUT3_PHASE bound to: 0.000000 - type: float 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter CLKOUT4_DIVIDE bound to: 1 - type: integer 
2default:defaulth p
x
? 
l
%s
*synth2T
@	Parameter CLKOUT4_DUTY_CYCLE bound to: 0.500000 - type: float 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter CLKOUT4_PHASE bound to: 0.000000 - type: float 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter CLKOUT5_DIVIDE bound to: 1 - type: integer 
2default:defaulth p
x
? 
l
%s
*synth2T
@	Parameter CLKOUT5_DUTY_CYCLE bound to: 0.500000 - type: float 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter CLKOUT5_PHASE bound to: 0.000000 - type: float 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter COMPENSATION bound to: ZHOLD - type: string 
2default:defaulth p
x
? 
b
%s
*synth2J
6	Parameter DIVCLK_DIVIDE bound to: 1 - type: integer 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter IS_CLKINSEL_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter IS_PWRDWN_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
W
%s
*synth2?
+	Parameter IS_RST_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
e
%s
*synth2M
9	Parameter REF_JITTER1 bound to: 0.010000 - type: float 
2default:defaulth p
x
? 
e
%s
*synth2M
9	Parameter REF_JITTER2 bound to: 0.010000 - type: float 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter STARTUP_WAIT bound to: FALSE - type: string 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	PLLE2_ADV2default:default2
 2default:default2
62default:default2
12default:default2K
5C:/Xilinx/Vivado/2019.1/scripts/rt/data/unisim_comp.v2default:default2
611992default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2%
clk_wiz_0_clk_wiz2default:default2
 2default:default2
72default:default2
12default:default2?
vc:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_clk_wiz.v2default:default2
692default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	clk_wiz_02default:default2
 2default:default2
82default:default2
12default:default2?
nc:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.v2default:default2
712default:default8@Z8-6155h px? 
?
0Net %s in module/entity %s does not have driver.3422*oasys2
LED_GREEN_12default:default2
fifo_verify2default:default2t
^C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/slavefifo_verify.v2default:default2
582default:default8@Z8-3848h px? 
?
0Net %s in module/entity %s does not have driver.3422*oasys2
	LED_RED_22default:default2
fifo_verify2default:default2t
^C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/slavefifo_verify.v2default:default2
592default:default8@Z8-3848h px? 
?
0Net %s in module/entity %s does not have driver.3422*oasys2 
LED_YELLOW_22default:default2
fifo_verify2default:default2t
^C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/slavefifo_verify.v2default:default2
602default:default8@Z8-3848h px? 
?
0Net %s in module/entity %s does not have driver.3422*oasys2
rx_oe2default:default2
fifo_verify2default:default2t
^C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/slavefifo_verify.v2default:default2
1392default:default8@Z8-3848h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
fifo_verify2default:default2
 2default:default2
92default:default2
12default:default2t
^C:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/slavefifo_verify.v2default:default2
392default:default8@Z8-6155h px? 
?
!design %s has unconnected port %s3331*oasys2
fifo_verify2default:default2
LED_GREEN_12default:defaultZ8-3331h px? 
?
!design %s has unconnected port %s3331*oasys2
fifo_verify2default:default2
	LED_RED_22default:defaultZ8-3331h px? 
?
!design %s has unconnected port %s3331*oasys2
fifo_verify2default:default2 
LED_YELLOW_22default:defaultZ8-3331h px? 
?
%s*synth2?
xFinished RTL Elaboration : Time (s): cpu = 00:00:06 ; elapsed = 00:00:06 . Memory (MB): peak = 779.777 ; gain = 243.758
2default:defaulth px? 
D
%s
*synth2,

Report Check Netlist: 
2default:defaulth p
x
? 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
? 
u
%s
*synth2]
I|      |Item              |Errors |Warnings |Status |Description       |
2default:defaulth p
x
? 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
? 
u
%s
*synth2]
I|1     |multi_driven_nets |      0|        0|Passed |Multi driven nets |
2default:defaulth p
x
? 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:06 ; elapsed = 00:00:07 . Memory (MB): peak = 779.777 ; gain = 243.758
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:06 ; elapsed = 00:00:07 . Memory (MB): peak = 779.777 ; gain = 243.758
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
e
-Analyzing %s Unisim elements for replacement
17*netlist2
12default:defaultZ29-17h px? 
j
2Unisim Transformation completed in %s CPU seconds
28*netlist2
02default:defaultZ29-28h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
>

Processing XDC Constraints
244*projectZ1-262h px? 
=
Initializing timing engine
348*projectZ1-569h px? 
?
$Parsing XDC File [%s] for cell '%s'
848*designutils2?
?c:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/fifo_departure_1024x64b/fifo_departure_1024x64b/fifo_departure_1024x64b_in_context.xdc2default:default2/
slave_fifo/fifo_departure	2default:default8Z20-848h px? 
?
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2?
?c:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/fifo_departure_1024x64b/fifo_departure_1024x64b/fifo_departure_1024x64b_in_context.xdc2default:default2/
slave_fifo/fifo_departure	2default:default8Z20-847h px? 
?
$Parsing XDC File [%s] for cell '%s'
848*designutils2?
?c:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/fifo_arrival_64x16b/fifo_arrival_64x16b/fifo_arrival_64x16b_in_context.xdc2default:default2-
slave_fifo/fifo_arrival	2default:default8Z20-848h px? 
?
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2?
?c:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/fifo_arrival_64x16b/fifo_arrival_64x16b/fifo_arrival_64x16b_in_context.xdc2default:default2-
slave_fifo/fifo_arrival	2default:default8Z20-847h px? 
?
$Parsing XDC File [%s] for cell '%s'
848*designutils2?
vc:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_board.xdc2default:default2"
clk_gen/inst	2default:default8Z20-848h px? 
?
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2?
vc:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_board.xdc2default:default2"
clk_gen/inst	2default:default8Z20-847h px? 
?
$Parsing XDC File [%s] for cell '%s'
848*designutils2?
pc:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xdc2default:default2"
clk_gen/inst	2default:default8Z20-848h px? 
?
-Finished Parsing XDC File [%s] for cell '%s'
847*designutils2?
pc:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xdc2default:default2"
clk_gen/inst	2default:default8Z20-847h px? 
?
?Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2?
pc:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xdc2default:default21
.Xil/fifo_verify_propImpl.xdc2default:defaultZ1-236h px? 
8
Deriving generated clocks
2*timingZ38-2h px? 
?
Parsing XDC File [%s]
179*designutils2b
LC:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/FIFO_verify.xdc2default:default8Z20-179h px? 
?
Finished Parsing XDC File [%s]
178*designutils2b
LC:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/FIFO_verify.xdc2default:default8Z20-178h px? 
?
?Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2`
LC:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/FIFO_verify.xdc2default:default21
.Xil/fifo_verify_propImpl.xdc2default:defaultZ1-236h px? 
?
Parsing XDC File [%s]
179*designutils2x
bC:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.runs/synth_1/dont_touch.xdc2default:default8Z20-179h px? 
?
Finished Parsing XDC File [%s]
178*designutils2x
bC:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.runs/synth_1/dont_touch.xdc2default:default8Z20-178h px? 
?
?Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2v
bC:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.runs/synth_1/dont_touch.xdc2default:default21
.Xil/fifo_verify_propImpl.xdc2default:defaultZ1-236h px? 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
897.3322default:default2
0.0002default:defaultZ17-268h px? 
~
!Unisim Transformation Summary:
%s111*project29
%No Unisim elements were transformed.
2default:defaultZ1-111h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common24
 Constraint Validation Runtime : 2default:default2
00:00:002default:default2 
00:00:00.0122default:default2
897.3322default:default2
0.0002default:defaultZ17-268h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
~Finished Constraint Validation : Time (s): cpu = 00:00:13 ; elapsed = 00:00:14 . Memory (MB): peak = 897.332 ; gain = 361.313
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
V
%s
*synth2>
*Start Loading Part and Timing Information
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
J
%s
*synth22
Loading part: xc7a15tcsg324-1
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Loading Part and Timing Information : Time (s): cpu = 00:00:13 ; elapsed = 00:00:14 . Memory (MB): peak = 897.332 ; gain = 361.313
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Z
%s
*synth2B
.Start Applying 'set_property' XDC Constraints
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:13 ; elapsed = 00:00:14 . Memory (MB): peak = 897.332 ; gain = 361.313
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2$
master_state_reg2default:default2"
fx3s_interface2default:defaultZ8-802h px? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2t
`                   State |                     New Encoding |                Previous Encoding 
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2s
_              state_idle |                             0000 |                             0000
2default:defaulth p
x
? 
?
%s
*synth2s
_       state_start_write |                             0001 |                             1000
2default:defaulth p
x
? 
?
%s
*synth2s
_       state_write_wait1 |                             0010 |                             1001
2default:defaulth p
x
? 
?
%s
*synth2s
_       state_write_wait2 |                             0011 |                             1010
2default:defaulth p
x
? 
?
%s
*synth2s
_       state_write_wait3 |                             0100 |                             1011
2default:defaulth p
x
? 
?
%s
*synth2s
_        state_start_read |                             0101 |                             0001
2default:defaulth p
x
? 
?
%s
*synth2s
_         state_read_pre1 |                             0110 |                             0010
2default:defaulth p
x
? 
?
%s
*synth2s
_         state_read_pre2 |                             0111 |                             0011
2default:defaulth p
x
? 
?
%s
*synth2s
_         state_read_pre3 |                             1000 |                             0100
2default:defaulth p
x
? 
?
%s
*synth2s
_         state_read_loop |                             1001 |                             0101
2default:defaulth p
x
? 
?
%s
*synth2s
_               state_zlp |                             1010 |                             1100
2default:defaulth p
x
? 
?
%s
*synth2s
_         state_zlp_wait1 |                             1011 |                             1101
2default:defaulth p
x
? 
?
%s
*synth2s
_         state_zlp_wait2 |                             1100 |                             1110
2default:defaulth p
x
? 
?
%s
*synth2s
_         state_zlp_wait3 |                             1101 |                             1111
2default:defaulth p
x
? 
?
%s
*synth2x
d---------------------------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2$
master_state_reg2default:default2

sequential2default:default2"
fx3s_interface2default:defaultZ8-3354h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:14 ; elapsed = 00:00:15 . Memory (MB): peak = 897.332 ; gain = 361.313
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     18 Bit       Adders := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     16 Bit       Adders := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      9 Bit       Adders := 1     
2default:defaulth p
x
? 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               18 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               16 Bit    Registers := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                9 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                1 Bit    Registers := 15    
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     16 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	  14 Input     16 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	  14 Input      4 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      4 Bit        Muxes := 5     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      1 Bit        Muxes := 8     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	  14 Input      1 Bit        Muxes := 21    
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Finished RTL Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Y
%s
*synth2A
-Start RTL Hierarchical Component Statistics 
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Hierarchical RTL Component report 
2default:defaulth p
x
? 
@
%s
*synth2(
Module fifo_verify 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     18 Bit       Adders := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      9 Bit       Adders := 1     
2default:defaulth p
x
? 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               18 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                9 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                1 Bit    Registers := 5     
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     64 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      1 Bit        Muxes := 1     
2default:defaulth p
x
? 
C
%s
*synth2+
Module fx3s_interface 
2default:defaulth p
x
? 
K
%s
*synth23
Detailed RTL Component Info : 
2default:defaulth p
x
? 
:
%s
*synth2"
+---Adders : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     16 Bit       Adders := 1     
2default:defaulth p
x
? 
=
%s
*synth2%
+---Registers : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               16 Bit    Registers := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                1 Bit    Registers := 10    
2default:defaulth p
x
? 
9
%s
*synth2!
+---Muxes : 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input     16 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	  14 Input     16 Bit        Muxes := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	  14 Input      4 Bit        Muxes := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      4 Bit        Muxes := 5     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	   2 Input      1 Bit        Muxes := 7     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	  14 Input      1 Bit        Muxes := 21    
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
[
%s
*synth2C
/Finished RTL Hierarchical Component Statistics
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2i
UPart Resources:
DSPs: 45 (col length:60)
BRAMs: 50 (col length: RAMB18 60 RAMB36 30)
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Part Resource Summary
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
W
%s
*synth2?
+Start Cross Boundary and Area Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
]
%s
*synth2E
1Warning: Parallel synthesis criteria is not met 
2default:defaulth p
x
? 
?
+design %s has port %s driven by constant %s3447*oasys2
fifo_verify2default:default2
A[1]2default:default2
02default:defaultZ8-3917h px? 
?
!design %s has unconnected port %s3331*oasys2
fifo_verify2default:default2
LED_GREEN_12default:defaultZ8-3331h px? 
?
!design %s has unconnected port %s3331*oasys2
fifo_verify2default:default2
	LED_RED_22default:defaultZ8-3331h px? 
?
!design %s has unconnected port %s3331*oasys2
fifo_verify2default:default2 
LED_YELLOW_22default:defaultZ8-3331h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2%
clk_1M_cnt_reg[7]2default:default2
FD2default:default2%
clk_1M_cnt_reg[6]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2%
clk_1M_cnt_reg[8]2default:default2
FD2default:default2%
clk_1M_cnt_reg[6]2default:defaultZ8-3886h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2'
\clk_1M_cnt_reg[6] 2default:defaultZ8-3333h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2!
clkcnt_reg[8]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2!
clkcnt_reg[9]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2"
clkcnt_reg[10]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2"
clkcnt_reg[11]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2"
clkcnt_reg[12]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2"
clkcnt_reg[13]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2"
clkcnt_reg[14]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2"
clkcnt_reg[15]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2"
clkcnt_reg[16]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2"
clkcnt_reg[17]2default:default2
FD2default:default2!
clkcnt_reg[5]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2!
clkcnt_reg[5]2default:default2
FD2default:default2!
clkcnt_reg[6]2default:defaultZ8-3886h px? 
?
"merging instance '%s' (%s) to '%s'3436*oasys2!
clkcnt_reg[6]2default:default2
FD2default:default2!
clkcnt_reg[7]2default:defaultZ8-3886h px? 
?
6propagating constant %s across sequential element (%s)3333*oasys2
02default:default2#
\clkcnt_reg[7] 2default:defaultZ8-3333h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:17 ; elapsed = 00:00:18 . Memory (MB): peak = 897.332 ; gain = 361.313
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
R
%s
*synth2:
&Start Applying XDC Timing Constraints
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
2Moved timing constraint from pin '%s' to pin '%s'
4028*oasys2'
clk_gen/inst/clk_in2default:default2 
sysclk_buf/O2default:defaultZ8-5578h px? 
?
SMoved %s constraints on hierarchical pins to their respective driving/loading pins
4235*oasys2
12default:defaultZ8-5819h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Applying XDC Timing Constraints : Time (s): cpu = 00:00:25 ; elapsed = 00:00:27 . Memory (MB): peak = 897.332 ; gain = 361.313
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
F
%s
*synth2.
Start Timing Optimization
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
|Finished Timing Optimization : Time (s): cpu = 00:00:26 ; elapsed = 00:00:28 . Memory (MB): peak = 898.703 ; gain = 362.684
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-
Start Technology Mapping
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
{Finished Technology Mapping : Time (s): cpu = 00:00:26 ; elapsed = 00:00:28 . Memory (MB): peak = 900.848 ; gain = 364.828
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s
*synth2'
Start IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
Q
%s
*synth29
%Start Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
T
%s
*synth2<
(Finished Flattening Before IO Insertion
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
H
%s
*synth20
Start Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Finished Final Netlist Cleanup
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
uFinished IO Insertion : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 907.633 ; gain = 371.613
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
D
%s
*synth2,

Report Check Netlist: 
2default:defaulth p
x
? 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
? 
u
%s
*synth2]
I|      |Item              |Errors |Warnings |Status |Description       |
2default:defaulth p
x
? 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
? 
u
%s
*synth2]
I|1     |multi_driven_nets |      0|        0|Passed |Multi driven nets |
2default:defaulth p
x
? 
u
%s
*synth2]
I+------+------------------+-------+---------+-------+------------------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
O
%s
*synth27
#Start Renaming Generated Instances
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Instances : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 907.633 ; gain = 371.613
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
E
%s
*synth2-

Report RTL Partitions: 
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
+| |RTL Partition |Replication |Instances |
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
W
%s
*synth2?
++-+--------------+------------+----------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
L
%s
*synth24
 Start Rebuilding User Hierarchy
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 907.633 ; gain = 371.613
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Renaming Generated Ports
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Ports : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 907.633 ; gain = 371.613
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
M
%s
*synth25
!Start Handling Custom Attributes
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 907.633 ; gain = 371.613
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
J
%s
*synth22
Start Renaming Generated Nets
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Renaming Generated Nets : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 907.633 ; gain = 371.613
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
K
%s
*synth23
Start Writing Synthesis Report
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
A
%s
*synth2)

Report BlackBoxes: 
2default:defaulth p
x
? 
Y
%s
*synth2A
-+------+------------------------+----------+
2default:defaulth p
x
? 
Y
%s
*synth2A
-|      |BlackBox name           |Instances |
2default:defaulth p
x
? 
Y
%s
*synth2A
-+------+------------------------+----------+
2default:defaulth p
x
? 
Y
%s
*synth2A
-|1     |fifo_departure_1024x64b |         1|
2default:defaulth p
x
? 
Y
%s
*synth2A
-|2     |fifo_arrival_64x16b     |         1|
2default:defaulth p
x
? 
Y
%s
*synth2A
-+------+------------------------+----------+
2default:defaulth p
x
? 
A
%s*synth2)

Report Cell Usage: 
2default:defaulth px? 
U
%s*synth2=
)+------+------------------------+------+
2default:defaulth px? 
U
%s*synth2=
)|      |Cell                    |Count |
2default:defaulth px? 
U
%s*synth2=
)+------+------------------------+------+
2default:defaulth px? 
U
%s*synth2=
)|1     |fifo_arrival_64x16b     |     1|
2default:defaulth px? 
U
%s*synth2=
)|2     |fifo_departure_1024x64b |     1|
2default:defaulth px? 
U
%s*synth2=
)|3     |BUFG                    |     6|
2default:defaulth px? 
U
%s*synth2=
)|4     |CARRY4                  |    14|
2default:defaulth px? 
U
%s*synth2=
)|5     |LUT1                    |    21|
2default:defaulth px? 
U
%s*synth2=
)|6     |LUT2                    |    44|
2default:defaulth px? 
U
%s*synth2=
)|7     |LUT3                    |     3|
2default:defaulth px? 
U
%s*synth2=
)|8     |LUT4                    |    18|
2default:defaulth px? 
U
%s*synth2=
)|9     |LUT5                    |    11|
2default:defaulth px? 
U
%s*synth2=
)|10    |LUT6                    |    58|
2default:defaulth px? 
U
%s*synth2=
)|11    |PLLE2_ADV               |     1|
2default:defaulth px? 
U
%s*synth2=
)|12    |FDRE                    |    88|
2default:defaulth px? 
U
%s*synth2=
)|13    |FDSE                    |     6|
2default:defaulth px? 
U
%s*synth2=
)|14    |IBUF                    |     5|
2default:defaulth px? 
U
%s*synth2=
)|15    |IOBUF                   |    16|
2default:defaulth px? 
U
%s*synth2=
)|16    |OBUF                    |    15|
2default:defaulth px? 
U
%s*synth2=
)|17    |OBUFT                   |     3|
2default:defaulth px? 
U
%s*synth2=
)+------+------------------------+------+
2default:defaulth px? 
E
%s
*synth2-

Report Instance Areas: 
2default:defaulth p
x
? 
]
%s
*synth2E
1+------+-------------+------------------+------+
2default:defaulth p
x
? 
]
%s
*synth2E
1|      |Instance     |Module            |Cells |
2default:defaulth p
x
? 
]
%s
*synth2E
1+------+-------------+------------------+------+
2default:defaulth p
x
? 
]
%s
*synth2E
1|1     |top          |                  |   349|
2default:defaulth p
x
? 
]
%s
*synth2E
1|2     |  clk_gen    |clk_wiz_0         |     5|
2default:defaulth p
x
? 
]
%s
*synth2E
1|3     |    inst     |clk_wiz_0_clk_wiz |     5|
2default:defaulth p
x
? 
]
%s
*synth2E
1|4     |  slave_fifo |fx3s_interface    |   251|
2default:defaulth p
x
? 
]
%s
*synth2E
1+------+-------------+------------------+------+
2default:defaulth p
x
? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Writing Synthesis Report : Time (s): cpu = 00:00:29 ; elapsed = 00:00:30 . Memory (MB): peak = 907.633 ; gain = 371.613
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
r
%s
*synth2Z
FSynthesis finished with 0 errors, 0 critical warnings and 4 warnings.
2default:defaulth p
x
? 
?
%s
*synth2?
~Synthesis Optimization Runtime : Time (s): cpu = 00:00:20 ; elapsed = 00:00:28 . Memory (MB): peak = 907.633 ; gain = 254.059
2default:defaulth p
x
? 
?
%s
*synth2?
Synthesis Optimization Complete : Time (s): cpu = 00:00:29 ; elapsed = 00:00:31 . Memory (MB): peak = 907.633 ; gain = 371.613
2default:defaulth p
x
? 
B
 Translating synthesized netlist
350*projectZ1-571h px? 
f
-Analyzing %s Unisim elements for replacement
17*netlist2
312default:defaultZ29-17h px? 
j
2Unisim Transformation completed in %s CPU seconds
28*netlist2
02default:defaultZ29-28h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
?
NRemoving redundant IBUF since it is not being driven by a top-level port. %s 
32*opt2-
clk_gen/inst/clkin1_ibufg2default:defaultZ31-32h px? 
u
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
920.4102default:default2
0.0002default:defaultZ17-268h px? 
?
!Unisim Transformation Summary:
%s111*project2m
Y  A total of 16 instances were transformed.
  IOBUF => IOBUF (IBUF, OBUFT): 16 instances
2default:defaultZ1-111h px? 
U
Releasing license: %s
83*common2
	Synthesis2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
552default:default2
122default:default2
02default:default2
02default:defaultZ4-41h px? 
^
%s completed successfully
29*	vivadotcl2 
synth_design2default:defaultZ4-42h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
synth_design: 2default:default2
00:00:342default:default2
00:00:372default:default2
920.4102default:default2
629.4572default:defaultZ17-268h px? 
?
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
920.4102default:default2
0.0002default:defaultZ17-268h px? 
K
"No constraints selected for write.1103*constraintsZ18-5210h px? 
?
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2w
cC:/Users/akrap/Google/AUV/zeabus_hydrophone/SlaveFIFO_verify/project_1.runs/synth_1/fifo_verify.dcp2default:defaultZ17-1381h px? 
?
%s4*runtcl2?
lExecuting : report_utilization -file fifo_verify_utilization_synth.rpt -pb fifo_verify_utilization_synth.pb
2default:defaulth px? 
?
Exiting %s at %s...
206*common2
Vivado2default:default2,
Thu Feb 27 20:13:44 20202default:defaultZ17-206h px? 


End Record