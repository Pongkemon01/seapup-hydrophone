
~
Command: %s
53*	vivadotcl2M
9synth_design -top zeabus_hydrophone -part xc7a15tcsg324-12default:defaultZ4-113h px? 
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
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
42default:defaultZ8-7079h px? 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px? 
_
#Helper process launched with PID %s4824*oasys2
80562default:defaultZ8-7075h px? 
?
.identifier '%s' is used before its declaration4750*oasys2!
trigger_event2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
1442default:default8@Z8-6901h px? 
?
%s*synth2?
?Starting RTL Elaboration : Time (s): cpu = 00:00:03 ; elapsed = 00:00:05 . Memory (MB): peak = 2389.785 ; gain = 0.000 ; free physical = 6529 ; free virtual = 12995
2default:defaulth px? 
?
synthesizing module '%s'%s4497*oasys2%
zeabus_hydrophone2default:default2
 2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
392default:default8@Z8-6157h px? 
b
%s
*synth2J
6	Parameter trigger_head bound to: 64 - type: integer 
2default:defaulth p
x
? 
b
%s
*synth2J
6	Parameter trigger_tail bound to: 64 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2

reset_subs2default:default2
 2default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/reset_subs/reset_subs.v2default:default2
392default:default8@Z8-6157h px? 
e
%s
*synth2M
9	Parameter reset_hold_time bound to: 16 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

reset_subs2default:default2
 2default:default2
12default:default2
12default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/reset_subs/reset_subs.v2default:default2
392default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2"
fx3s_interface2default:default2
 2default:default2n
X/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/fx3s_interface/fx3s_interface.v2default:default2
1472default:default8@Z8-6157h px? 
e
%s
*synth2M
9	Parameter FX3S_DMA_Size bound to: 4096 - type: integer 
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
]
%s
*synth2E
1	Parameter state_stop_write_1 bound to: 4'b0001 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter state_stop_write_2 bound to: 4'b0010 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter state_stop_write_3 bound to: 4'b0011 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter state_stop_write_4 bound to: 4'b0100 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter state_start_read bound to: 4'b0101 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter state_read_pre_1 bound to: 4'b0110 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter state_read_pre_2 bound to: 4'b0111 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter state_read_word bound to: 4'b1000 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter state_read_post_1 bound to: 4'b1001 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter state_read_post_2 bound to: 4'b1010 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter state_zlp bound to: 4'b1011 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2%
CascadedFIFO32bit2default:default2
 2default:default2n
X/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/fx3s_interface/fx3s_interface.v2default:default2
362default:default8@Z8-6157h px? 
Z
%s
*synth2B
.	Parameter DEPTH bound to: 4 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2
FIFO36E12default:default2
 2default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
143742default:default8@Z8-6157h px? 
h
%s
*synth2P
<	Parameter ALMOST_EMPTY_OFFSET bound to: 13'b0000010000000 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter ALMOST_FULL_OFFSET bound to: 13'b0000000010100 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter DATA_WIDTH bound to: 36 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter DO_REG bound to: 1 - type: integer 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter EN_ECC_READ bound to: FALSE - type: string 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter EN_ECC_WRITE bound to: FALSE - type: string 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter EN_SYN bound to: FALSE - type: string 
2default:defaulth p
x
? 
b
%s
*synth2J
6	Parameter FIFO_MODE bound to: FIFO36 - type: string 
2default:defaulth p
x
? 
n
%s
*synth2V
B	Parameter FIRST_WORD_FALL_THROUGH bound to: TRUE - type: string 
2default:defaulth p
x
? 
?
%s
*synth2|
h	Parameter INIT bound to: 72'b000000000000000000000000000000000000000000000000000000000000000000000000 
2default:defaulth p
x
? 
Y
%s
*synth2A
-	Parameter IS_RDCLK_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter IS_RDEN_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter IS_RSTREG_INVERTED bound to: 1'b0 
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
Y
%s
*synth2A
-	Parameter IS_WRCLK_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter IS_WREN_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter SIM_DEVICE bound to: 7SERIES - type: string 
2default:defaulth p
x
? 
?
%s
*synth2}
i	Parameter SRVAL bound to: 72'b000000000000000000000000000000000000000000000000000000000000000000000000 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
FIFO36E12default:default2
 2default:default2
22default:default2
12default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
143742default:default8@Z8-6155h px? 
?
Pwidth (%s) of port connection '%s' does not match port width (%s) of module '%s'689*oasys2
172default:default2
DO2default:default2
642default:default2
FIFO36E12default:default2n
X/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/fx3s_interface/fx3s_interface.v2default:default2
882default:default8@Z8-689h px? 
?
Pwidth (%s) of port connection '%s' does not match port width (%s) of module '%s'689*oasys2
172default:default2
DO2default:default2
642default:default2
FIFO36E12default:default2n
X/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/fx3s_interface/fx3s_interface.v2default:default2
882default:default8@Z8-689h px? 
?
Pwidth (%s) of port connection '%s' does not match port width (%s) of module '%s'689*oasys2
172default:default2
DO2default:default2
642default:default2
FIFO36E12default:default2n
X/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/fx3s_interface/fx3s_interface.v2default:default2
882default:default8@Z8-689h px? 
?
Pwidth (%s) of port connection '%s' does not match port width (%s) of module '%s'689*oasys2
172default:default2
DO2default:default2
642default:default2
FIFO36E12default:default2n
X/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/fx3s_interface/fx3s_interface.v2default:default2
882default:default8@Z8-689h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2%
CascadedFIFO32bit2default:default2
 2default:default2
32default:default2
12default:default2n
X/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/fx3s_interface/fx3s_interface.v2default:default2
362default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2,
FIFO36E1__parameterized02default:default2
 2default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
143742default:default8@Z8-6157h px? 
h
%s
*synth2P
<	Parameter ALMOST_EMPTY_OFFSET bound to: 13'b0000010000000 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter ALMOST_FULL_OFFSET bound to: 13'b0000000010100 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter DATA_WIDTH bound to: 18 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter DO_REG bound to: 1 - type: integer 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter EN_ECC_READ bound to: FALSE - type: string 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter EN_ECC_WRITE bound to: FALSE - type: string 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter EN_SYN bound to: FALSE - type: string 
2default:defaulth p
x
? 
b
%s
*synth2J
6	Parameter FIFO_MODE bound to: FIFO36 - type: string 
2default:defaulth p
x
? 
n
%s
*synth2V
B	Parameter FIRST_WORD_FALL_THROUGH bound to: TRUE - type: string 
2default:defaulth p
x
? 
?
%s
*synth2|
h	Parameter INIT bound to: 72'b000000000000000000000000000000000000000000000000000000000000000000000000 
2default:defaulth p
x
? 
Y
%s
*synth2A
-	Parameter IS_RDCLK_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter IS_RDEN_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter IS_RSTREG_INVERTED bound to: 1'b0 
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
Y
%s
*synth2A
-	Parameter IS_WRCLK_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter IS_WREN_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter SIM_DEVICE bound to: 7SERIES - type: string 
2default:defaulth p
x
? 
?
%s
*synth2}
i	Parameter SRVAL bound to: 72'b000000000000000000000000000000000000000000000000000000000000000000000000 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2,
FIFO36E1__parameterized02default:default2
 2default:default2
32default:default2
12default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
143742default:default8@Z8-6155h px? 
?
Pwidth (%s) of port connection '%s' does not match port width (%s) of module '%s'689*oasys2
162default:default2
DO2default:default2
642default:default2,
FIFO36E1__parameterized02default:default2n
X/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/fx3s_interface/fx3s_interface.v2default:default2
5932default:default8@Z8-689h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2"
fx3s_interface2default:default2
 2default:default2
42default:default2
12default:default2n
X/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/fx3s_interface/fx3s_interface.v2default:default2
1472default:default8@Z8-6155h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
state2default:default2"
fx3s_interface2default:default2

slave_fifo2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
1692default:default8@Z8-7071h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
TxEmpty2default:default2"
fx3s_interface2default:default2

slave_fifo2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
1692default:default8@Z8-7071h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
outgoing2default:default2"
fx3s_interface2default:default2

slave_fifo2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
1692default:default8@Z8-7071h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
pkt_end_out2default:default2"
fx3s_interface2default:default2

slave_fifo2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
1692default:default8@Z8-7071h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2

is_sending2default:default2"
fx3s_interface2default:default2

slave_fifo2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
1692default:default8@Z8-7071h px? 
?
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2

slave_fifo2default:default2"
fx3s_interface2default:default2
262default:default2
212default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
1692default:default8@Z8-7023h px? 
?
synthesizing module '%s'%s4497*oasys2-
hydrophone_config_manager2default:default2
 2default:default2m
W/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/cofig_manager/config_manager.v2default:default2
732default:default8@Z8-6157h px? 
\
%s
*synth2D
0	Parameter config_prefix bound to: 8'b11011100 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter rst_delay bound to: 8 - type: integer 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter DEFAULT_POTEN bound to: 8'b11100110 
2default:defaulth p
x
? 
j
%s
*synth2R
>	Parameter DEFAULT_TRIG_LEVEL bound to: 16'b0011111111111111 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter state_wait_prefix bound to: 3'b000 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter state_read_prefix bound to: 3'b001 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter state_wait_trigger bound to: 3'b010 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter state_read_trigger bound to: 3'b011 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter state_wait_poten1_2 bound to: 3'b100 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter state_read_poten1_2 bound to: 3'b101 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter state_wait_poten3_4 bound to: 3'b110 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter state_read_poten3_4 bound to: 3'b111 
2default:defaulth p
x
? 
?
display: %s251*oasys2)
Config : Start config2default:default2m
W/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/cofig_manager/config_manager.v2default:default2
1672default:default8@Z8-251h px? 
?
-case statement is not full and has no default155*oasys2m
W/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/cofig_manager/config_manager.v2default:default2
1572default:default8@Z8-155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2-
hydrophone_config_manager2default:default2
 2default:default2
52default:default2
12default:default2m
W/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/cofig_manager/config_manager.v2default:default2
732default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2#
poten_interface2default:default2
 2default:default2p
Z/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/poten_interface.v2default:default2
402default:default8@Z8-6157h px? 
\
%s
*synth2D
0	Parameter i2c_address_1 bound to: 8'b01011100 
2default:defaulth p
x
? 
\
%s
*synth2D
0	Parameter i2c_address_2 bound to: 8'b01011110 
2default:defaulth p
x
? 
i
%s
*synth2Q
=	Parameter i2c_clk_prescaler bound to: 16'b0000000000111111 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter CMD_GEN_START bound to: 4'b1000 
2default:defaulth p
x
? 
W
%s
*synth2?
+	Parameter CMD_GEN_STOP bound to: 4'b0100 
2default:defaulth p
x
? 
S
%s
*synth2;
'	Parameter CMD_READ bound to: 4'b0010 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter CMD_WRITE bound to: 4'b0001 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter SUBSTATE_SEND_SLAVE_ADDR bound to: 2'b00 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter SUBSTATE_SEND_POTEN_ADDR bound to: 2'b01 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter SUBSTATE_SEND_POTEN_VAL bound to: 2'b10 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter STATE_IDLE bound to: 3'b000 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter STATE_SET_POTEN1 bound to: 3'b001 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter STATE_SET_POTEN2 bound to: 3'b010 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter STATE_SET_POTEN3 bound to: 3'b011 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter STATE_SET_POTEN4 bound to: 3'b100 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2(
i2c_master_byte_ctrl2default:default2
 2default:default2u
_/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/i2c_master_byte_ctrl.v2default:default2
732default:default8@Z8-6157h px? 
S
%s
*synth2;
'	Parameter ST_IDLE bound to: 5'b00000 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter ST_START bound to: 5'b00001 
2default:defaulth p
x
? 
S
%s
*synth2;
'	Parameter ST_READ bound to: 5'b00010 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter ST_WRITE bound to: 5'b00100 
2default:defaulth p
x
? 
R
%s
*synth2:
&	Parameter ST_ACK bound to: 5'b01000 
2default:defaulth p
x
? 
S
%s
*synth2;
'	Parameter ST_STOP bound to: 5'b10000 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2'
i2c_master_bit_ctrl2default:default2
 2default:default2t
^/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/i2c_master_bit_ctrl.v2default:default2
1422default:default8@Z8-6157h px? 
^
%s
*synth2F
2	Parameter idle bound to: 18'b000000000000000000 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter start_a bound to: 18'b000000000000000001 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter start_b bound to: 18'b000000000000000010 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter start_c bound to: 18'b000000000000000100 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter start_d bound to: 18'b000000000000001000 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter start_e bound to: 18'b000000000000010000 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter stop_a bound to: 18'b000000000000100000 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter stop_b bound to: 18'b000000000001000000 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter stop_c bound to: 18'b000000000010000000 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter stop_d bound to: 18'b000000000100000000 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter rd_a bound to: 18'b000000001000000000 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter rd_b bound to: 18'b000000010000000000 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter rd_c bound to: 18'b000000100000000000 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter rd_d bound to: 18'b000001000000000000 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter wr_a bound to: 18'b000010000000000000 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter wr_b bound to: 18'b000100000000000000 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter wr_c bound to: 18'b001000000000000000 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter wr_d bound to: 18'b010000000000000000 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2'
i2c_master_bit_ctrl2default:default2
 2default:default2
62default:default2
12default:default2t
^/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/i2c_master_bit_ctrl.v2default:default2
1422default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2(
i2c_master_byte_ctrl2default:default2
 2default:default2
72default:default2
12default:default2u
_/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/i2c_master_byte_ctrl.v2default:default2
732default:default8@Z8-6155h px? 
?
-case statement is not full and has no default155*oasys2p
Z/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/poten_interface.v2default:default2
2722default:default8@Z8-155h px? 
?
-case statement is not full and has no default155*oasys2p
Z/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/poten_interface.v2default:default2
2772default:default8@Z8-155h px? 
?
-case statement is not full and has no default155*oasys2p
Z/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/poten_interface.v2default:default2
2822default:default8@Z8-155h px? 
?
-case statement is not full and has no default155*oasys2p
Z/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/poten_interface.v2default:default2
2872default:default8@Z8-155h px? 
?
-case statement is not full and has no default155*oasys2p
Z/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/poten_interface.v2default:default2
2632default:default8@Z8-155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2#
poten_interface2default:default2
 2default:default2
82default:default2
12default:default2p
Z/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/poten_interface/poten_interface.v2default:default2
402default:default8@Z8-6155h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
phase2default:default2#
poten_interface2default:default2
	poten_i2c2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
2212default:default8@Z8-7071h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2
poten_state2default:default2#
poten_interface2default:default2
	poten_i2c2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
2212default:default8@Z8-7071h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2#
poten_sub_state2default:default2#
poten_interface2default:default2
	poten_i2c2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
2212default:default8@Z8-7071h px? 
?
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2
	poten_i2c2default:default2#
poten_interface2default:default2
122default:default2
92default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
2212default:default8@Z8-7023h px? 
?
synthesizing module '%s'%s4497*oasys2

packetizer2default:default2
 2default:default2e
O/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/packetizer/packetize.v2default:default2
362default:default8@Z8-6157h px? 
k
%s
*synth2S
?	Parameter SAMPLING_PER_PACKET bound to: 1000 - type: integer 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter MAX_PKT_SIZE bound to: 4000 - type: integer 
2default:defaulth p
x
? 
V
%s
*synth2>
*	Parameter STATE_IDLE bound to: 5'b00000 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter STATE_SEND_HEADER_SEQ bound to: 5'b00001 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter STATE_SEND_HEADER_TIME_H bound to: 5'b00010 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter STATE_SEND_HEADER_TIME_L bound to: 5'b00011 
2default:defaulth p
x
? 
_
%s
*synth2G
3	Parameter STATE_SEND_DATA_CH1 bound to: 5'b00100 
2default:defaulth p
x
? 
_
%s
*synth2G
3	Parameter STATE_SEND_DATA_CH2 bound to: 5'b00101 
2default:defaulth p
x
? 
_
%s
*synth2G
3	Parameter STATE_SEND_DATA_CH3 bound to: 5'b00110 
2default:defaulth p
x
? 
_
%s
*synth2G
3	Parameter STATE_SEND_DATA_CH4 bound to: 5'b00111 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter STATE_LATCH_LAST_WORD bound to: 5'b01000 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter STATE_WAIT_STROBE bound to: 5'b01001 
2default:defaulth p
x
? 
P
%s
*synth28
$	Parameter OUT_ID bound to: 3'b000 
2default:defaulth p
x
? 
Q
%s
*synth29
%	Parameter OUT_SEQ bound to: 3'b001 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter OUT_TIME_L bound to: 3'b010 
2default:defaulth p
x
? 
T
%s
*synth2<
(	Parameter OUT_TIME_H bound to: 3'b011 
2default:defaulth p
x
? 
Q
%s
*synth29
%	Parameter OUT_CH1 bound to: 3'b100 
2default:defaulth p
x
? 
Q
%s
*synth29
%	Parameter OUT_CH2 bound to: 3'b101 
2default:defaulth p
x
? 
Q
%s
*synth29
%	Parameter OUT_CH3 bound to: 3'b110 
2default:defaulth p
x
? 
Q
%s
*synth29
%	Parameter OUT_CH4 bound to: 3'b111 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

packetizer2default:default2
 2default:default2
92default:default2
12default:default2e
O/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/packetizer/packetize.v2default:default2
362default:default8@Z8-6155h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2$
debug_main_state2default:default2

packetizer2default:default2#
packetizer_inst2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
2382default:default8@Z8-7071h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2 
debug_strb_d2default:default2

packetizer2default:default2#
packetizer_inst2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
2382default:default8@Z8-7071h px? 
?
9port '%s' of module '%s' is unconnected for instance '%s'4818*oasys2$
pkt_size_counter2default:default2

packetizer2default:default2#
packetizer_inst2default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
2382default:default8@Z8-7071h px? 
?
Kinstance '%s' of module '%s' has %s connections declared, but only %s given4757*oasys2#
packetizer_inst2default:default2

packetizer2default:default2
122default:default2
92default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
2382default:default8@Z8-7023h px? 
?
synthesizing module '%s'%s4497*oasys2.
hydrophone_trigger_backlog2default:default2
 2default:default2v
`/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/hydrophone_trigger/hydrophone_trigger.v2default:default2
1572default:default8@Z8-6157h px? 
f
%s
*synth2N
:	Parameter PRETRIG_SAMPLING bound to: 64 - type: integer 
2default:defaulth p
x
? 
g
%s
*synth2O
;	Parameter POSTTRIG_SAMPLING bound to: 64 - type: integer 
2default:defaulth p
x
? 
e
%s
*synth2M
9	Parameter PRETRIG_MINUS_1 bound to: 63 - type: integer 
2default:defaulth p
x
? 
a
%s
*synth2I
5	Parameter TOTAL_TAIL bound to: 128 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2%
CascadedFIFO64bit2default:default2
 2default:default2v
`/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/hydrophone_trigger/hydrophone_trigger.v2default:default2
372default:default8@Z8-6157h px? 
[
%s
*synth2C
/	Parameter DEPTH bound to: 20 - type: integer 
2default:defaulth p
x
? 
e
%s
*synth2M
9	Parameter BACKLOG_SIZE bound to: 10000 - type: integer 
2default:defaulth p
x
? 
e
%s
*synth2M
9	Parameter ALMOST_FULL_TH bound to: 240 - type: integer 
2default:defaulth p
x
? 
e
%s
*synth2M
9	Parameter ALMOST_FULL_FIFO bound to: 0 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2,
FIFO36E1__parameterized12default:default2
 2default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
143742default:default8@Z8-6157h px? 
h
%s
*synth2P
<	Parameter ALMOST_EMPTY_OFFSET bound to: 13'b0000010000000 
2default:defaulth p
x
? 
i
%s
*synth2Q
=	Parameter ALMOST_FULL_OFFSET bound to: 240 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter DATA_WIDTH bound to: 72 - type: integer 
2default:defaulth p
x
? 
[
%s
*synth2C
/	Parameter DO_REG bound to: 1 - type: integer 
2default:defaulth p
x
? 
c
%s
*synth2K
7	Parameter EN_ECC_READ bound to: FALSE - type: string 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter EN_ECC_WRITE bound to: FALSE - type: string 
2default:defaulth p
x
? 
^
%s
*synth2F
2	Parameter EN_SYN bound to: FALSE - type: string 
2default:defaulth p
x
? 
e
%s
*synth2M
9	Parameter FIFO_MODE bound to: FIFO36_72 - type: string 
2default:defaulth p
x
? 
n
%s
*synth2V
B	Parameter FIRST_WORD_FALL_THROUGH bound to: TRUE - type: string 
2default:defaulth p
x
? 
?
%s
*synth2|
h	Parameter INIT bound to: 72'b000000000000000000000000000000000000000000000000000000000000000000000000 
2default:defaulth p
x
? 
Y
%s
*synth2A
-	Parameter IS_RDCLK_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter IS_RDEN_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
Z
%s
*synth2B
.	Parameter IS_RSTREG_INVERTED bound to: 1'b0 
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
Y
%s
*synth2A
-	Parameter IS_WRCLK_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
X
%s
*synth2@
,	Parameter IS_WREN_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
d
%s
*synth2L
8	Parameter SIM_DEVICE bound to: 7SERIES - type: string 
2default:defaulth p
x
? 
?
%s
*synth2}
i	Parameter SRVAL bound to: 72'b000000000000000000000000000000000000000000000000000000000000000000000000 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2,
FIFO36E1__parameterized12default:default2
 2default:default2
92default:default2
12default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
143742default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2%
CascadedFIFO64bit2default:default2
 2default:default2
102default:default2
12default:default2v
`/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/hydrophone_trigger/hydrophone_trigger.v2default:default2
372default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2.
hydrophone_trigger_backlog2default:default2
 2default:default2
112default:default2
12default:default2v
`/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/hydrophone_trigger/hydrophone_trigger.v2default:default2
1572default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2-
hydrophone_simple_trigger2default:default2
 2default:default2v
`/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/hydrophone_trigger/hydrophone_trigger.v2default:default2
3032default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2
absolute2default:default2
 2default:default2v
`/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/hydrophone_trigger/hydrophone_trigger.v2default:default2
2972default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
absolute2default:default2
 2default:default2
122default:default2
12default:default2v
`/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/hydrophone_trigger/hydrophone_trigger.v2default:default2
2972default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2-
hydrophone_simple_trigger2default:default2
 2default:default2
132default:default2
12default:default2v
`/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/hydrophone_trigger/hydrophone_trigger.v2default:default2
3032default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2$
strobe_decimator2default:default2
 2default:default2e
O/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/dsp/strobe_decimator.v2default:default2
372default:default8@Z8-6157h px? 
b
%s
*synth2J
6	Parameter MAX_COUNTING bound to: 10 - type: integer 
2default:defaulth p
x
? 
f
%s
*synth2N
:	Parameter MAX_COUNT_MINUS_1 bound to: 9 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2$
strobe_decimator2default:default2
 2default:default2
142default:default2
12default:default2e
O/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/dsp/strobe_decimator.v2default:default2
372default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2!
adc_interface2default:default2
 2default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
2552default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2
IDDR2default:default2
 2default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
349382default:default8@Z8-6157h px? 
r
%s
*synth2Z
F	Parameter DDR_CLK_EDGE bound to: SAME_EDGE_PIPELINED - type: string 
2default:defaulth p
x
? 
O
%s
*synth27
#	Parameter INIT_Q1 bound to: 1'b0 
2default:defaulth p
x
? 
O
%s
*synth27
#	Parameter INIT_Q2 bound to: 1'b0 
2default:defaulth p
x
? 
U
%s
*synth2=
)	Parameter IS_C_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
U
%s
*synth2=
)	Parameter IS_D_INVERTED bound to: 1'b0 
2default:defaulth p
x
? 
]
%s
*synth2E
1	Parameter SRTYPE bound to: SYNC - type: string 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
IDDR2default:default2
 2default:default2
152default:default2
12default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
349382default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2

adc_filter2default:default2
 2default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
2102default:default8@Z8-6157h px? 
]
%s
*synth2E
1	Parameter is_dummy bound to: 0 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter dummy_max bound to: 680 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys2!
median_filter2default:default2
 2default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
462default:default8@Z8-6157h px? 
?
synthesizing module '%s'%s4497*oasys2
sort2default:default2
 2default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
342default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
sort2default:default2
 2default:default2
162default:default2
12default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
342default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2!
median_filter2default:default2
 2default:default2
172default:default2
12default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
462default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2!
avg64_binning2default:default2
 2default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
1292default:default8@Z8-6157h px? 
]
%s
*synth2E
1	Parameter is_dummy bound to: 0 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter dummy_max bound to: 680 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2!
avg64_binning2default:default2
 2default:default2
182default:default2
12default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
1292default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

adc_filter2default:default2
 2default:default2
192default:default2
12default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
2102default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2.
adc_filter__parameterized02default:default2
 2default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
2102default:default8@Z8-6157h px? 
]
%s
*synth2E
1	Parameter is_dummy bound to: 0 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter dummy_max bound to: 100 - type: integer 
2default:defaulth p
x
? 
?
synthesizing module '%s'%s4497*oasys21
avg64_binning__parameterized02default:default2
 2default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
1292default:default8@Z8-6157h px? 
]
%s
*synth2E
1	Parameter is_dummy bound to: 0 - type: integer 
2default:defaulth p
x
? 
`
%s
*synth2H
4	Parameter dummy_max bound to: 100 - type: integer 
2default:defaulth p
x
? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys21
avg64_binning__parameterized02default:default2
 2default:default2
192default:default2
12default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
1292default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2.
adc_filter__parameterized02default:default2
 2default:default2
192default:default2
12default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
2102default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2!
adc_interface2default:default2
 2default:default2
202default:default2
12default:default2f
P/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/adc_filter/adc_filter.v2default:default2
2552default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2
BUFG2default:default2
 2default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
10832default:default8@Z8-6157h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
BUFG2default:default2
 2default:default2
212default:default2
12default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
10832default:default8@Z8-6155h px? 
?
synthesizing module '%s'%s4497*oasys2

PLLE2_BASE2default:default2
 2default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
616052default:default8@Z8-6157h px? 
e
%s
*synth2M
9	Parameter BANDWIDTH bound to: OPTIMIZED - type: string 
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
i
%s
*synth2Q
=	Parameter CLKFBOUT_PHASE bound to: 0.000000 - type: double 
2default:defaulth p
x
? 
i
%s
*synth2Q
=	Parameter CLKIN1_PERIOD bound to: 38.462000 - type: double 
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
m
%s
*synth2U
A	Parameter CLKOUT0_DUTY_CYCLE bound to: 0.500000 - type: double 
2default:defaulth p
x
? 
k
%s
*synth2S
?	Parameter CLKOUT0_PHASE bound to: -140.000000 - type: double 
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
m
%s
*synth2U
A	Parameter CLKOUT1_DUTY_CYCLE bound to: 0.500000 - type: double 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter CLKOUT1_PHASE bound to: 0.000000 - type: double 
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
m
%s
*synth2U
A	Parameter CLKOUT2_DUTY_CYCLE bound to: 0.500000 - type: double 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter CLKOUT2_PHASE bound to: 0.000000 - type: double 
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
m
%s
*synth2U
A	Parameter CLKOUT3_DUTY_CYCLE bound to: 0.500000 - type: double 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter CLKOUT3_PHASE bound to: 0.000000 - type: double 
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
m
%s
*synth2U
A	Parameter CLKOUT4_DUTY_CYCLE bound to: 0.500000 - type: double 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter CLKOUT4_PHASE bound to: 0.000000 - type: double 
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
m
%s
*synth2U
A	Parameter CLKOUT5_DUTY_CYCLE bound to: 0.500000 - type: double 
2default:defaulth p
x
? 
h
%s
*synth2P
<	Parameter CLKOUT5_PHASE bound to: 0.000000 - type: double 
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
f
%s
*synth2N
:	Parameter REF_JITTER1 bound to: 0.000000 - type: double 
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

PLLE2_BASE2default:default2
 2default:default2
222default:default2
12default:default2X
B/home/stp/asset/xilinx/Vivado/2020.2/scripts/rt/data/unisim_comp.v2default:default2
616052default:default8@Z8-6155h px? 
?
'done synthesizing module '%s'%s (%s#%s)4495*oasys2%
zeabus_hydrophone2default:default2
 2default:default2
232default:default2
12default:default2b
L/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/sources/zeabus_hydrophone.v2default:default2
392default:default8@Z8-6155h px? 
?
%s*synth2?
?Finished RTL Elaboration : Time (s): cpu = 00:00:04 ; elapsed = 00:00:07 . Memory (MB): peak = 2389.785 ; gain = 0.000 ; free physical = 7311 ; free virtual = 13784
2default:defaulth px? 
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
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:05 ; elapsed = 00:00:07 . Memory (MB): peak = 2389.785 ; gain = 0.000 ; free physical = 7310 ; free virtual = 13785
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
?Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:05 ; elapsed = 00:00:07 . Memory (MB): peak = 2389.785 ; gain = 0.000 ; free physical = 7310 ; free virtual = 13785
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.032default:default2
00:00:00.052default:default2
2389.7852default:default2
0.0002default:default2
73012default:default2
137762default:defaultZ17-722h px? 
f
-Analyzing %s Unisim elements for replacement
17*netlist2
562default:defaultZ29-17h px? 
?
?The %s for %s (%s) is being adjusted from %s to %s. The %s value needs to be in increments of 1/56 the FVCO and/or increments depending on CLKOUT_DIVIDE. Refer to the Clocking Resources User Guide of the device family for details.370*netlist2!
CLKOUT0_PHASE2default:default2
	PLLE2_ADV2default:default2#
PLLE2_BASE_inst2default:default2
-140.02default:default2
-141.92default:default2!
CLKOUT0_PHASE2default:defaultZ29-346h px? 
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
?
Parsing XDC File [%s]
179*designutils2\
F/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/seapup_hydrophone.xdc2default:default8Z20-179h px? 
?
Deriving generated clocks
2*timing2\
F/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/seapup_hydrophone.xdc2default:default2
5292default:default8@Z38-2h px?
?
Finished Parsing XDC File [%s]
178*designutils2\
F/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/seapup_hydrophone.xdc2default:default8Z20-178h px? 
?
?Implementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2Z
F/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/seapup_hydrophone.xdc2default:default27
#.Xil/zeabus_hydrophone_propImpl.xdc2default:defaultZ1-236h px? 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
2480.5042default:default2
0.0002default:default2
72052default:default2
136942default:defaultZ17-722h px? 
?
!Unisim Transformation Summary:
%s111*project2f
R  A total of 1 instances were transformed.
  PLLE2_BASE => PLLE2_ADV: 1 instance 
2default:defaultZ1-111h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common24
 Constraint Validation Runtime : 2default:default2
00:00:00.012default:default2
00:00:00.012default:default2
2480.5042default:default2
0.0002default:default2
72052default:default2
136942default:defaultZ17-722h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Constraint Validation : Time (s): cpu = 00:00:10 ; elapsed = 00:00:14 . Memory (MB): peak = 2480.504 ; gain = 90.719 ; free physical = 7289 ; free virtual = 13779
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
?Finished Loading Part and Timing Information : Time (s): cpu = 00:00:10 ; elapsed = 00:00:14 . Memory (MB): peak = 2480.504 ; gain = 90.719 ; free physical = 7289 ; free virtual = 13779
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
?Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:10 ; elapsed = 00:00:14 . Memory (MB): peak = 2480.504 ; gain = 90.719 ; free physical = 7289 ; free virtual = 13779
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2
	state_reg2default:default2-
hydrophone_config_manager2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2
c_state_reg2default:default2'
i2c_master_bit_ctrl2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2
c_state_reg2default:default2(
i2c_master_byte_ctrl2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2!
sub_state_reg2default:default2#
poten_interface2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2"
main_state_reg2default:default2#
poten_interface2default:defaultZ8-802h px? 
?
3inferred FSM for state register '%s' in module '%s'802*oasys2"
main_state_reg2default:default2

packetizer2default:defaultZ8-802h px? 
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
_       state_wait_prefix |                               00 |                              000
2default:defaulth p
x
? 
?
%s
*synth2s
_      state_read_trigger |                               01 |                              011
2default:defaulth p
x
? 
?
%s
*synth2s
_     state_read_poten1_2 |                               10 |                              101
2default:defaulth p
x
? 
?
%s
*synth2s
_     state_read_poten3_4 |                               11 |                              111
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
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
	state_reg2default:default2

sequential2default:default2-
hydrophone_config_manager2default:defaultZ8-3354h px? 
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
_                    idle |               000000000000000001 |               000000000000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                 start_a |               000000000000000010 |               000000000000000001
2default:defaulth p
x
? 
?
%s
*synth2s
_                 start_b |               000000000000000100 |               000000000000000010
2default:defaulth p
x
? 
?
%s
*synth2s
_                 start_c |               000000000000001000 |               000000000000000100
2default:defaulth p
x
? 
?
%s
*synth2s
_                 start_d |               000000000000010000 |               000000000000001000
2default:defaulth p
x
? 
?
%s
*synth2s
_                 start_e |               000000000000100000 |               000000000000010000
2default:defaulth p
x
? 
?
%s
*synth2s
_                  stop_a |               000000000001000000 |               000000000000100000
2default:defaulth p
x
? 
?
%s
*synth2s
_                  stop_b |               000000000010000000 |               000000000001000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                  stop_c |               000000000100000000 |               000000000010000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                  stop_d |               000000001000000000 |               000000000100000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                    wr_a |               000000010000000000 |               000010000000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                    wr_b |               000000100000000000 |               000100000000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                    wr_c |               000001000000000000 |               001000000000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                    wr_d |               000010000000000000 |               010000000000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                    rd_a |               000100000000000000 |               000000001000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                    rd_b |               001000000000000000 |               000000010000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                    rd_c |               010000000000000000 |               000000100000000000
2default:defaulth p
x
? 
?
%s
*synth2s
_                    rd_d |               100000000000000000 |               000001000000000000
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
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
c_state_reg2default:default2
one-hot2default:default2'
i2c_master_bit_ctrl2default:defaultZ8-3354h px? 
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
_                 ST_IDLE |                              000 |                            00000
2default:defaulth p
x
? 
?
%s
*synth2s
_                ST_START |                              001 |                            00001
2default:defaulth p
x
? 
?
%s
*synth2s
_                 ST_READ |                              010 |                            00010
2default:defaulth p
x
? 
?
%s
*synth2s
_                ST_WRITE |                              011 |                            00100
2default:defaulth p
x
? 
?
%s
*synth2s
_                  ST_ACK |                              100 |                            01000
2default:defaulth p
x
? 
?
%s
*synth2s
_                 ST_STOP |                              101 |                            10000
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
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2
c_state_reg2default:default2

sequential2default:default2(
i2c_master_byte_ctrl2default:defaultZ8-3354h px? 
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
_SUBSTATE_SEND_SLAVE_ADDR |                               00 |                               00
2default:defaulth p
x
? 
?
%s
*synth2s
_SUBSTATE_SEND_POTEN_ADDR |                               01 |                               01
2default:defaulth p
x
? 
?
%s
*synth2s
_ SUBSTATE_SEND_POTEN_VAL |                               10 |                               10
2default:defaulth p
x
? 
?
%s
*synth2s
_                  iSTATE |                               11 |                               11
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
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2!
sub_state_reg2default:default2

sequential2default:default2#
poten_interface2default:defaultZ8-3354h px? 
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
_              STATE_IDLE |                              000 |                              000
2default:defaulth p
x
? 
?
%s
*synth2s
_        STATE_SET_POTEN1 |                              001 |                              001
2default:defaulth p
x
? 
?
%s
*synth2s
_        STATE_SET_POTEN2 |                              010 |                              010
2default:defaulth p
x
? 
?
%s
*synth2s
_        STATE_SET_POTEN3 |                              011 |                              011
2default:defaulth p
x
? 
?
%s
*synth2s
_        STATE_SET_POTEN4 |                              100 |                              100
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
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2"
main_state_reg2default:default2

sequential2default:default2#
poten_interface2default:defaultZ8-3354h px? 
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
_              STATE_IDLE |                       0000000001 |                            00000
2default:defaulth p
x
? 
?
%s
*synth2s
_   STATE_SEND_HEADER_SEQ |                       0000000010 |                            00001
2default:defaulth p
x
? 
?
%s
*synth2s
_STATE_SEND_HEADER_TIME_H |                       0000000100 |                            00010
2default:defaulth p
x
? 
?
%s
*synth2s
_STATE_SEND_HEADER_TIME_L |                       0000001000 |                            00011
2default:defaulth p
x
? 
?
%s
*synth2s
_     STATE_SEND_DATA_CH1 |                       0000010000 |                            00100
2default:defaulth p
x
? 
?
%s
*synth2s
_     STATE_SEND_DATA_CH2 |                       0000100000 |                            00101
2default:defaulth p
x
? 
?
%s
*synth2s
_     STATE_SEND_DATA_CH3 |                       0001000000 |                            00110
2default:defaulth p
x
? 
?
%s
*synth2s
_     STATE_SEND_DATA_CH4 |                       0010000000 |                            00111
2default:defaulth p
x
? 
?
%s
*synth2s
_   STATE_LATCH_LAST_WORD |                       0100000000 |                            01000
2default:defaulth p
x
? 
?
%s
*synth2s
_       STATE_WAIT_STROBE |                       1000000000 |                            01001
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
Gencoded FSM with state register '%s' using encoding '%s' in module '%s'3353*oasys2"
main_state_reg2default:default2
one-hot2default:default2

packetizer2default:defaultZ8-3354h px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:12 ; elapsed = 00:00:15 . Memory (MB): peak = 2480.504 ; gain = 90.719 ; free physical = 7284 ; free virtual = 13777
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
X
%s
*synth2@
,	   2 Input   20 Bit       Adders := 8     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   16 Bit       Adders := 10    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   14 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    8 Bit       Adders := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    6 Bit       Adders := 4     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    3 Bit       Adders := 1     
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
.	               64 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               32 Bit    Registers := 1     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               20 Bit    Registers := 8     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               16 Bit    Registers := 7     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	               14 Bit    Registers := 41    
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                8 Bit    Registers := 7     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                6 Bit    Registers := 4     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                4 Bit    Registers := 4     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                3 Bit    Registers := 4     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                2 Bit    Registers := 2     
2default:defaulth p
x
? 
Z
%s
*synth2B
.	                1 Bit    Registers := 87    
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
X
%s
*synth2@
,	   2 Input   20 Bit        Muxes := 4     
2default:defaulth p
x
? 
X
%s
*synth2@
,	  24 Input   18 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   16 Bit        Muxes := 9     
2default:defaulth p
x
? 
X
%s
*synth2@
,	  10 Input   16 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   14 Bit        Muxes := 82    
2default:defaulth p
x
? 
X
%s
*synth2@
,	  10 Input   10 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input   10 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    8 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    8 Bit        Muxes := 4     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   5 Input    8 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	  10 Input    5 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    4 Bit        Muxes := 9     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   6 Input    4 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    4 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   5 Input    4 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    3 Bit        Muxes := 6     
2default:defaulth p
x
? 
X
%s
*synth2@
,	  15 Input    3 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   5 Input    3 Bit        Muxes := 2     
2default:defaulth p
x
? 
X
%s
*synth2@
,	  10 Input    3 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    2 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    2 Bit        Muxes := 7     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   3 Input    2 Bit        Muxes := 1     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   2 Input    1 Bit        Muxes := 59    
2default:defaulth p
x
? 
X
%s
*synth2@
,	  13 Input    1 Bit        Muxes := 22    
2default:defaulth p
x
? 
X
%s
*synth2@
,	   5 Input    1 Bit        Muxes := 6     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   4 Input    1 Bit        Muxes := 15    
2default:defaulth p
x
? 
X
%s
*synth2@
,	  18 Input    1 Bit        Muxes := 5     
2default:defaulth p
x
? 
X
%s
*synth2@
,	   6 Input    1 Bit        Muxes := 6     
2default:defaulth p
x
? 
X
%s
*synth2@
,	  10 Input    1 Bit        Muxes := 8     
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
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:20 ; elapsed = 00:00:24 . Memory (MB): peak = 2480.504 ; gain = 90.719 ; free physical = 7260 ; free virtual = 13780
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
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Applying XDC Timing Constraints : Time (s): cpu = 00:00:24 ; elapsed = 00:00:29 . Memory (MB): peak = 2480.504 ; gain = 90.719 ; free physical = 7124 ; free virtual = 13657
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
?Finished Timing Optimization : Time (s): cpu = 00:00:26 ; elapsed = 00:00:30 . Memory (MB): peak = 2480.504 ; gain = 90.719 ; free physical = 7096 ; free virtual = 13630
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
?Finished Technology Mapping : Time (s): cpu = 00:00:27 ; elapsed = 00:00:32 . Memory (MB): peak = 2480.504 ; gain = 90.719 ; free physical = 7095 ; free virtual = 13629
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
?Finished IO Insertion : Time (s): cpu = 00:00:30 ; elapsed = 00:00:35 . Memory (MB): peak = 2483.473 ; gain = 93.688 ; free physical = 7108 ; free virtual = 13640
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
?Finished Renaming Generated Instances : Time (s): cpu = 00:00:30 ; elapsed = 00:00:35 . Memory (MB): peak = 2483.473 ; gain = 93.688 ; free physical = 7108 ; free virtual = 13640
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
?Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:30 ; elapsed = 00:00:35 . Memory (MB): peak = 2483.473 ; gain = 93.688 ; free physical = 7107 ; free virtual = 13640
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
?Finished Renaming Generated Ports : Time (s): cpu = 00:00:30 ; elapsed = 00:00:35 . Memory (MB): peak = 2483.473 ; gain = 93.688 ; free physical = 7107 ; free virtual = 13640
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
?Finished Handling Custom Attributes : Time (s): cpu = 00:00:30 ; elapsed = 00:00:35 . Memory (MB): peak = 2483.473 ; gain = 93.688 ; free physical = 7107 ; free virtual = 13640
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
?Finished Renaming Generated Nets : Time (s): cpu = 00:00:30 ; elapsed = 00:00:35 . Memory (MB): peak = 2483.473 ; gain = 93.688 ; free physical = 7107 ; free virtual = 13640
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
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
| |BlackBox name |Instances |
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
J
%s
*synth22
+-+--------------+----------+
2default:defaulth p
x
? 
A
%s*synth2)

Report Cell Usage: 
2default:defaulth px? 
H
%s*synth20
+------+-----------+------+
2default:defaulth px? 
H
%s*synth20
|      |Cell       |Count |
2default:defaulth px? 
H
%s*synth20
+------+-----------+------+
2default:defaulth px? 
H
%s*synth20
|1     |BUFG       |     2|
2default:defaulth px? 
H
%s*synth20
|2     |CARRY4     |   188|
2default:defaulth px? 
H
%s*synth20
|3     |FIFO36E1   |    25|
2default:defaulth px? 
H
%s*synth20
|6     |IDDR       |    30|
2default:defaulth px? 
H
%s*synth20
|7     |LUT1       |   155|
2default:defaulth px? 
H
%s*synth20
|8     |LUT2       |   158|
2default:defaulth px? 
H
%s*synth20
|9     |LUT3       |   477|
2default:defaulth px? 
H
%s*synth20
|10    |LUT4       |   331|
2default:defaulth px? 
H
%s*synth20
|11    |LUT5       |   564|
2default:defaulth px? 
H
%s*synth20
|12    |LUT6       |   439|
2default:defaulth px? 
H
%s*synth20
|13    |MUXF7      |    16|
2default:defaulth px? 
H
%s*synth20
|14    |PLLE2_BASE |     1|
2default:defaulth px? 
H
%s*synth20
|15    |FDRE       |  1134|
2default:defaulth px? 
H
%s*synth20
|16    |FDSE       |    62|
2default:defaulth px? 
H
%s*synth20
|17    |IBUF       |    35|
2default:defaulth px? 
H
%s*synth20
|18    |IOBUF      |    18|
2default:defaulth px? 
H
%s*synth20
|19    |OBUF       |    24|
2default:defaulth px? 
H
%s*synth20
+------+-----------+------+
2default:defaulth px? 
~
%s
*synth2f
R---------------------------------------------------------------------------------
2default:defaulth p
x
? 
?
%s*synth2?
?Finished Writing Synthesis Report : Time (s): cpu = 00:00:30 ; elapsed = 00:00:35 . Memory (MB): peak = 2483.473 ; gain = 93.688 ; free physical = 7107 ; free virtual = 13640
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
FSynthesis finished with 0 errors, 0 critical warnings and 0 warnings.
2default:defaulth p
x
? 
?
%s
*synth2?
?Synthesis Optimization Runtime : Time (s): cpu = 00:00:28 ; elapsed = 00:00:32 . Memory (MB): peak = 2483.473 ; gain = 2.969 ; free physical = 7161 ; free virtual = 13694
2default:defaulth p
x
? 
?
%s
*synth2?
?Synthesis Optimization Complete : Time (s): cpu = 00:00:30 ; elapsed = 00:00:35 . Memory (MB): peak = 2483.480 ; gain = 93.688 ; free physical = 7161 ; free virtual = 13694
2default:defaulth p
x
? 
B
 Translating synthesized netlist
350*projectZ1-571h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:00.042default:default2
00:00:00.042default:default2
2483.4802default:default2
0.0002default:default2
71532default:default2
136892default:defaultZ17-722h px? 
g
-Analyzing %s Unisim elements for replacement
17*netlist2
2782default:defaultZ29-17h px? 
?
?The %s for %s (%s) is being adjusted from %s to %s. The %s value needs to be in increments of 1/56 the FVCO and/or increments depending on CLKOUT_DIVIDE. Refer to the Clocking Resources User Guide of the device family for details.370*netlist2!
CLKOUT0_PHASE2default:default2
	PLLE2_ADV2default:default2#
PLLE2_BASE_inst2default:default2
-140.02default:default2
-141.92default:default2!
CLKOUT0_PHASE2default:defaultZ29-346h px? 
j
2Unisim Transformation completed in %s CPU seconds
28*netlist2
52default:defaultZ29-28h px? 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px? 
u
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
2483.4802default:default2
0.0002default:default2
71832default:default2
137192default:defaultZ17-722h px? 
?
!Unisim Transformation Summary:
%s111*project2?
?  A total of 19 instances were transformed.
  IOBUF => IOBUF (IBUF, OBUFT): 18 instances
  PLLE2_BASE => PLLE2_ADV: 1 instance 
2default:defaultZ1-111h px? 
U
Releasing license: %s
83*common2
	Synthesis2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
912default:default2
202default:default2
22default:default2
02default:defaultZ4-41h px? 
^
%s completed successfully
29*	vivadotcl2 
synth_design2default:defaultZ4-42h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2"
synth_design: 2default:default2
00:00:412default:default2
00:00:562default:default2
2483.4802default:default2
93.7732default:default2
73302default:default2
138662default:defaultZ17-722h px? 
u
%s6*runtcl2Y
ESynthesis results are not added to the cache due to CRITICAL_WARNING
2default:defaulth px? 
?
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2v
b/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/hydrophone_dsp.runs/synth_1/zeabus_hydrophone.dcp2default:defaultZ17-1381h px? 
?
%s4*runtcl2?
xExecuting : report_utilization -file zeabus_hydrophone_utilization_synth.rpt -pb zeabus_hydrophone_utilization_synth.pb
2default:defaulth px? 
?
Exiting %s at %s...
206*common2
Vivado2default:default2,
Sat Jul  9 11:41:02 20222default:defaultZ17-206h px? 


End Record