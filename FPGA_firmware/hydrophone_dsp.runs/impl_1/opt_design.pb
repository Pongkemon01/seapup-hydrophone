
O
Command: %s
53*	vivadotcl2

opt_design2default:defaultZ4-113h px? 
?
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2"
Implementation2default:default2
xc7a15t2default:defaultZ17-347h px? 
?
0Got license for feature '%s' and/or device '%s'
310*common2"
Implementation2default:default2
xc7a15t2default:defaultZ17-349h px? 
n
,Running DRC as a precondition to command %s
22*	vivadotcl2

opt_design2default:defaultZ4-22h px? 
R

Starting %s Task
103*constraints2
DRC2default:defaultZ18-103h px? 
P
Running DRC with %s threads
24*drc2
42default:defaultZ23-27h px? 
U
DRC finished with %s
272*project2
0 Errors2default:defaultZ1-461h px? 
d
BPlease refer to the DRC report (report_drc) for more information.
274*projectZ1-462h px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:02 ; elapsed = 00:00:01 . Memory (MB): peak = 2713.641 ; gain = 32.016 ; free physical = 7003 ; free virtual = 135522default:defaulth px? 
g

Starting %s Task
103*constraints2,
Cache Timing Information2default:defaultZ18-103h px? 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px? 
P
;Ending Cache Timing Information Task | Checksum: 12207a954
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.21 ; elapsed = 00:00:00.09 . Memory (MB): peak = 2735.453 ; gain = 21.812 ; free physical = 7003 ; free virtual = 135532default:defaulth px? 
a

Starting %s Task
103*constraints2&
Logic Optimization2default:defaultZ18-103h px? 
i

Phase %s%s
101*constraints2
1 2default:default2
Retarget2default:defaultZ18-101h px? 
u
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
12default:default2
12default:defaultZ31-138h px? 
K
Retargeted %s cell(s).
49*opt2
02default:defaultZ31-49h px? 
<
'Phase 1 Retarget | Checksum: 119aae033
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.29 ; elapsed = 00:00:00.18 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6822 ; free virtual = 133712default:defaulth px? 
?
.Phase %s created %s cells and removed %s cells267*opt2
Retarget2default:default2
02default:default2
22default:defaultZ31-389h px? 
u

Phase %s%s
101*constraints2
2 2default:default2(
Constant propagation2default:defaultZ18-101h px? 
u
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02default:default2
02default:defaultZ31-138h px? 
H
3Phase 2 Constant propagation | Checksum: 13bb19144
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.32 ; elapsed = 00:00:00.21 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6822 ; free virtual = 133712default:defaulth px? 
?
.Phase %s created %s cells and removed %s cells267*opt2(
Constant propagation2default:default2
02default:default2
02default:defaultZ31-389h px? 
f

Phase %s%s
101*constraints2
3 2default:default2
Sweep2default:defaultZ18-101h px? 
9
$Phase 3 Sweep | Checksum: 1908ad76b
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.38 ; elapsed = 00:00:00.27 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6822 ; free virtual = 133722default:defaulth px? 
?
.Phase %s created %s cells and removed %s cells267*opt2
Sweep2default:default2
12default:default2
02default:defaultZ31-389h px? 
r

Phase %s%s
101*constraints2
4 2default:default2%
BUFG optimization2default:defaultZ18-101h px? 
?
4Inserted BUFG %s to drive %s load(s) on clock net %s141*opt2)
CLKB_2_OBUF_BUFG_inst2default:default2
42default:default2$
CLKB_2_OBUF_BUFG2default:defaultZ31-194h px? 
W
!Inserted %s BUFG(s) on clock nets140*opt2
12default:defaultZ31-193h px? 
E
0Phase 4 BUFG optimization | Checksum: 12d8bd0c6
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.43 ; elapsed = 00:00:00.33 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6821 ; free virtual = 133712default:defaulth px? 
?
EPhase %s created %s cells of which %s are BUFGs and removed %s cells.395*opt2%
BUFG optimization2default:default2
12default:default2
12default:default2
02default:defaultZ31-662h px? 
|

Phase %s%s
101*constraints2
5 2default:default2/
Shift Register Optimization2default:defaultZ18-101h px? 
?
dSRL Remap converted %s SRLs to %s registers and converted %s registers of register chains to %s SRLs546*opt2
02default:default2
02default:default2
02default:default2
02default:defaultZ31-1064h px? 
O
:Phase 5 Shift Register Optimization | Checksum: 12d8bd0c6
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.44 ; elapsed = 00:00:00.33 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6821 ; free virtual = 133712default:defaulth px? 
?
.Phase %s created %s cells and removed %s cells267*opt2/
Shift Register Optimization2default:default2
02default:default2
02default:defaultZ31-389h px? 
x

Phase %s%s
101*constraints2
6 2default:default2+
Post Processing Netlist2default:defaultZ18-101h px? 
K
6Phase 6 Post Processing Netlist | Checksum: 12d8bd0c6
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.45 ; elapsed = 00:00:00.34 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6821 ; free virtual = 133712default:defaulth px? 
?
.Phase %s created %s cells and removed %s cells267*opt2+
Post Processing Netlist2default:default2
02default:default2
02default:defaultZ31-389h px? 
/
Opt_design Change Summary
*commonh px? 
/
=========================
*commonh px? 


*commonh px? 


*commonh px? 
?
z-------------------------------------------------------------------------------------------------------------------------
*commonh px? 
?
?|  Phase                        |  #Cells created  |  #Cells Removed  |  #Constrained objects preventing optimizations  |
-------------------------------------------------------------------------------------------------------------------------
*commonh px? 
?
?|  Retarget                     |               0  |               2  |                                              0  |
|  Constant propagation         |               0  |               0  |                                              0  |
|  Sweep                        |               1  |               0  |                                              0  |
|  BUFG optimization            |               1  |               0  |                                              0  |
|  Shift Register Optimization  |               0  |               0  |                                              0  |
|  Post Processing Netlist      |               0  |               0  |                                              0  |
-------------------------------------------------------------------------------------------------------------------------
*commonh px? 


*commonh px? 


*commonh px? 
a

Starting %s Task
103*constraints2&
Connectivity Check2default:defaultZ18-103h px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.01 ; elapsed = 00:00:00.01 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6821 ; free virtual = 133712default:defaulth px? 
J
5Ending Logic Optimization Task | Checksum: 1552526f9
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.54 ; elapsed = 00:00:00.43 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6821 ; free virtual = 133712default:defaulth px? 
a

Starting %s Task
103*constraints2&
Power Optimization2default:defaultZ18-103h px? 
s
7Will skip clock gating for clocks with period < %s ns.
114*pwropt2
2.002default:defaultZ34-132h px? 
J
5Ending Power Optimization Task | Checksum: 1552526f9
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00.01 ; elapsed = 00:00:00.01 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6821 ; free virtual = 133702default:defaulth px? 
\

Starting %s Task
103*constraints2!
Final Cleanup2default:defaultZ18-103h px? 
E
0Ending Final Cleanup Task | Checksum: 1552526f9
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00 ; elapsed = 00:00:00 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6821 ; free virtual = 133702default:defaulth px? 
b

Starting %s Task
103*constraints2'
Netlist Obfuscation2default:defaultZ18-103h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2.
Netlist sorting complete. 2default:default2
00:00:002default:default2
00:00:002default:default2
2888.4532default:default2
0.0002default:default2
68212default:default2
133702default:defaultZ17-722h px? 
K
6Ending Netlist Obfuscation Task | Checksum: 1552526f9
*commonh px? 
?

%s
*constraints2?
?Time (s): cpu = 00:00:00 ; elapsed = 00:00:00 . Memory (MB): peak = 2888.453 ; gain = 0.000 ; free physical = 6821 ; free virtual = 133702default:defaulth px? 
Z
Releasing license: %s
83*common2"
Implementation2default:defaultZ17-83h px? 
?
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
282default:default2
02default:default2
02default:default2
02default:defaultZ4-41h px? 
\
%s completed successfully
29*	vivadotcl2

opt_design2default:defaultZ4-42h px? 
H
&Writing timing data to binary archive.266*timingZ38-480h px? 
D
Writing placer database...
1603*designutilsZ20-1893h px? 
=
Writing XDEF routing.
211*designutilsZ20-211h px? 
J
#Writing XDEF routing logical nets.
209*designutilsZ20-209h px? 
J
#Writing XDEF routing special nets.
210*designutilsZ20-210h px? 
?
r%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s ; free physical = %s ; free virtual = %s
480*common2)
Write XDEF Complete: 2default:default2
00:00:00.072default:default2
00:00:00.022default:default2
2896.4572default:default2
0.0002default:default2
68072default:default2
133572default:defaultZ17-722h px? 
?
 The %s '%s' has been generated.
621*common2

checkpoint2default:default2y
e/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/hydrophone_dsp.runs/impl_1/zeabus_hydrophone_opt.dcp2default:defaultZ17-1381h px? 
?
%s4*runtcl2?
?Executing : report_drc -file zeabus_hydrophone_drc_opted.rpt -pb zeabus_hydrophone_drc_opted.pb -rpx zeabus_hydrophone_drc_opted.rpx
2default:defaulth px? 
?
Command: %s
53*	vivadotcl2?
xreport_drc -file zeabus_hydrophone_drc_opted.rpt -pb zeabus_hydrophone_drc_opted.pb -rpx zeabus_hydrophone_drc_opted.rpx2default:defaultZ4-113h px? 
>
Refreshing IP repositories
234*coregenZ19-234h px? 
G
"No user IP repositories specified
1154*coregenZ19-1704h px? 
?
"Loaded Vivado IP repository '%s'.
1332*coregen2@
,/home/stp/asset/xilinx/Vivado/2020.2/data/ip2default:defaultZ19-2313h px? 
E
%Done setting XDC timing constraints.
35*timingZ38-35h px? 
P
Running DRC with %s threads
24*drc2
42default:defaultZ23-27h px? 
?
#The results of DRC are in file %s.
168*coretcl2?
k/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/hydrophone_dsp.runs/impl_1/zeabus_hydrophone_drc_opted.rptk/home/stp/seapup/zeabus_hydrophone/FPGA_firmware/hydrophone_dsp.runs/impl_1/zeabus_hydrophone_drc_opted.rpt2default:default8Z2-168h px? 
\
%s completed successfully
29*	vivadotcl2

report_drc2default:defaultZ4-42h px? 


End Record