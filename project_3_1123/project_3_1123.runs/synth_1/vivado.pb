
­
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
create_project: 2

00:00:022

00:00:112	
530.0042	
200.680Z17-268h px 
Ķ
Command: %s
1870*	planAhead2
read_checkpoint -auto_incremental -incremental C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/utils_1/imports/synth_1/SS_Decoder.dcpZ12-2866h px 
Ė
;Read reference checkpoint from %s for incremental synthesis3154*	planAhead2g
eC:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/utils_1/imports/synth_1/SS_Decoder.dcpZ12-5825h px 
T
-Please ensure there are no constraint changes3725*	planAheadZ12-7989h px 
f
Command: %s
53*	vivadotcl25
3synth_design -top SS_Decoder -part xc7a100tcsg324-1Z4-113h px 
:
Starting synth_design
149*	vivadotclZ4-321h px 
{
@Attempting to get a license for feature '%s' and/or device '%s'
308*common2
	Synthesis2

xc7a100tZ17-347h px 
k
0Got license for feature '%s' and/or device '%s'
310*common2
	Synthesis2

xc7a100tZ17-349h px 
E
Loading part %s157*device2
xc7a100tcsg324-1Z21-403h px 
[
$Part: %s does not have CEAM library.966*device2
xc7a100tcsg324-1Z21-9227h px 

VNo compile time benefit to using incremental synthesis; A full resynthesis will be run2353*designutilsZ20-5440h px 
¢
ųFlow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px 
o
HMultithreading enabled for synth_design using a maximum of %s processes.4828*oasys2
2Z8-7079h px 
a
?Launching helper process for spawning children vivado processes4827*oasysZ8-7078h px 
N
#Helper process launched with PID %s4824*oasys2
11980Z8-7075h px 

%s*synth2{
yStarting RTL Elaboration : Time (s): cpu = 00:00:02 ; elapsed = 00:00:05 . Memory (MB): peak = 1401.621 ; gain = 449.078
h px 
é
*overwriting previous definition of %s '%s'6131*oasys2
module2
	debouncer2x
tC:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/sources_1/imports/°ųĒŠ¼³°č_lab5_7segment/SS_Decoder.v2
288@Z8-9873h px 
Õ
2previous definition of design element '%s' is here6195*oasys2
	debouncer2f
bC:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/sources_1/imports/veril/debouncer.v2
268@Z8-9937h px 
Ō
synthesizing module '%s'%s4497*oasys2

SS_Decoder2
 2x
tC:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/sources_1/imports/°ųĒŠ¼³°č_lab5_7segment/SS_Decoder.v2
308@Z8-6157h px 
Ņ
synthesizing module '%s'%s4497*oasys2
	debouncer2
 2x
tC:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/sources_1/imports/°ųĒŠ¼³°č_lab5_7segment/SS_Decoder.v2
38@Z8-6157h px 
é
'done synthesizing module '%s'%s (%s#%s)4495*oasys2
	debouncer2
 2
02
12x
tC:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/sources_1/imports/°ųĒŠ¼³°č_lab5_7segment/SS_Decoder.v2
38@Z8-6155h px 
ė
'done synthesizing module '%s'%s (%s#%s)4495*oasys2

SS_Decoder2
 2
02
12x
tC:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/sources_1/imports/°ųĒŠ¼³°č_lab5_7segment/SS_Decoder.v2
308@Z8-6155h px 

%s*synth2{
yFinished RTL Elaboration : Time (s): cpu = 00:00:03 ; elapsed = 00:00:07 . Memory (MB): peak = 1515.258 ; gain = 562.715
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
;
%s
*synth2#
!Start Handling Custom Attributes
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
 
%s*synth2
Finished Handling Custom Attributes : Time (s): cpu = 00:00:03 ; elapsed = 00:00:07 . Memory (MB): peak = 1515.258 ; gain = 562.715
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2
Finished RTL Optimization Phase 1 : Time (s): cpu = 00:00:03 ; elapsed = 00:00:07 . Memory (MB): peak = 1515.258 ; gain = 562.715
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
ŗ
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.0082

1515.2582
0.000Z17-268h px 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px 
>

Processing XDC Constraints
244*projectZ1-262h px 
=
Initializing timing engine
348*projectZ1-569h px 
Ā
Parsing XDC File [%s]
179*designutils2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc8Z20-179h px 
Ó
No ports matched '%s'.
584*	planAhead2
B[0]2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc2
178@Z12-584h px
į
"'%s' expects at least one object.
55*common2
set_property2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc2
178@Z17-55h px
Ó
No ports matched '%s'.
584*	planAhead2
B[1]2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc2
188@Z12-584h px
į
"'%s' expects at least one object.
55*common2
set_property2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc2
188@Z17-55h px
Ó
No ports matched '%s'.
584*	planAhead2
B[2]2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc2
198@Z12-584h px
į
"'%s' expects at least one object.
55*common2
set_property2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc2
198@Z17-55h px
Ó
No ports matched '%s'.
584*	planAhead2
B[3]2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc2
208@Z12-584h px
į
"'%s' expects at least one object.
55*common2
set_property2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc2
208@Z17-55h px
Ė
Finished Parsing XDC File [%s]
178*designutils2
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc8Z20-178h px 

ŁImplementation specific constraints were found while reading constraint file [%s]. These constraints will be ignored for synthesis but will be used in implementation. Impacted constraints are listed in the file [%s].
233*project2~
|C:/Users/user/Desktop/veril/project_3_1123/project_3_1123.srcs/constrs_1/imports/°ųĒŠ¼³°č_lab5_7segment/Nexys4DDR_Master.xdc2
.Xil/SS_Decoder_propImpl.xdcZ1-236h px 
H
&Completed Processing XDC Constraints

245*projectZ1-263h px 
¶
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002

00:00:002

1617.5982
0.000Z17-268h px 
l
!Unisim Transformation Summary:
%s111*project2'
%No Unisim elements were transformed.
Z1-111h px 
Ą
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2"
 Constraint Validation Runtime : 2

00:00:002
00:00:00.0052

1617.5982
0.000Z17-268h px 

VNo compile time benefit to using incremental synthesis; A full resynthesis will be run2353*designutilsZ20-5440h px 
¢
ųFlow is switching to default flow due to incremental criteria not met. If you would like to alter this behaviour and have the flow terminate instead, please set the following parameter config_implementation {autoIncr.Synth.RejectBehavior Terminate}2229*designutilsZ20-4379h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2
Finished Constraint Validation : Time (s): cpu = 00:00:06 ; elapsed = 00:00:15 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
D
%s
*synth2,
*Start Loading Part and Timing Information
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
9
%s
*synth2!
Loading part: xc7a100tcsg324-1
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
©
%s*synth2
Finished Loading Part and Timing Information : Time (s): cpu = 00:00:06 ; elapsed = 00:00:15 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
H
%s
*synth20
.Start Applying 'set_property' XDC Constraints
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
­
%s*synth2
Finished applying 'set_property' XDC Constraints : Time (s): cpu = 00:00:06 ; elapsed = 00:00:15 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2
Finished RTL Optimization Phase 2 : Time (s): cpu = 00:00:06 ; elapsed = 00:00:15 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
:
%s
*synth2"
 Start RTL Component Statistics 
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
9
%s
*synth2!
Detailed RTL Component Info : 
h p
x
 
(
%s
*synth2
+---Adders : 
h p
x
 
F
%s
*synth2.
,	   2 Input   28 Bit       Adders := 1     
h p
x
 
F
%s
*synth2.
,	   2 Input   27 Bit       Adders := 1     
h p
x
 
F
%s
*synth2.
,	   2 Input   26 Bit       Adders := 1     
h p
x
 
F
%s
*synth2.
,	   2 Input   25 Bit       Adders := 1     
h p
x
 
F
%s
*synth2.
,	   2 Input    5 Bit       Adders := 1     
h p
x
 
F
%s
*synth2.
,	   2 Input    4 Bit       Adders := 11    
h p
x
 
F
%s
*synth2.
,	   2 Input    3 Bit       Adders := 1     
h p
x
 
+
%s
*synth2
+---Registers : 
h p
x
 
H
%s
*synth20
.	               32 Bit    Registers := 2     
h p
x
 
H
%s
*synth20
.	               28 Bit    Registers := 1     
h p
x
 
H
%s
*synth20
.	               25 Bit    Registers := 1     
h p
x
 
H
%s
*synth20
.	                4 Bit    Registers := 4     
h p
x
 
H
%s
*synth20
.	                3 Bit    Registers := 3     
h p
x
 
H
%s
*synth20
.	                1 Bit    Registers := 19    
h p
x
 
'
%s
*synth2
+---Muxes : 
h p
x
 
F
%s
*synth2.
,	   7 Input   32 Bit        Muxes := 2     
h p
x
 
F
%s
*synth2.
,	   2 Input   32 Bit        Muxes := 1     
h p
x
 
F
%s
*synth2.
,	   2 Input   28 Bit        Muxes := 2     
h p
x
 
F
%s
*synth2.
,	   2 Input   27 Bit        Muxes := 1     
h p
x
 
F
%s
*synth2.
,	   2 Input   25 Bit        Muxes := 2     
h p
x
 
F
%s
*synth2.
,	   2 Input    8 Bit        Muxes := 4     
h p
x
 
F
%s
*synth2.
,	   4 Input    8 Bit        Muxes := 1     
h p
x
 
F
%s
*synth2.
,	   6 Input    8 Bit        Muxes := 1     
h p
x
 
F
%s
*synth2.
,	   2 Input    7 Bit        Muxes := 2     
h p
x
 
F
%s
*synth2.
,	   2 Input    4 Bit        Muxes := 16    
h p
x
 
F
%s
*synth2.
,	   7 Input    4 Bit        Muxes := 2     
h p
x
 
F
%s
*synth2.
,	   4 Input    4 Bit        Muxes := 1     
h p
x
 
F
%s
*synth2.
,	   2 Input    3 Bit        Muxes := 4     
h p
x
 
F
%s
*synth2.
,	   2 Input    2 Bit        Muxes := 4     
h p
x
 
F
%s
*synth2.
,	   2 Input    1 Bit        Muxes := 9     
h p
x
 
F
%s
*synth2.
,	   7 Input    1 Bit        Muxes := 7     
h p
x
 
F
%s
*synth2.
,	   4 Input    1 Bit        Muxes := 1     
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
=
%s
*synth2%
#Finished RTL Component Statistics 
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
6
%s
*synth2
Start Part Resource Summary
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
q
%s
*synth2Y
WPart Resources:
DSPs: 240 (col length:80)
BRAMs: 270 (col length: RAMB18 80 RAMB36 40)
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
9
%s
*synth2!
Finished Part Resource Summary
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
E
%s
*synth2-
+Start Cross Boundary and Area Optimization
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
H
&Parallel synthesis criteria is not met4829*oasysZ8-7080h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
Ŗ
%s*synth2
Finished Cross Boundary and Area Optimization : Time (s): cpu = 00:00:07 ; elapsed = 00:00:19 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
@
%s
*synth2(
&Start Applying XDC Timing Constraints
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
„
%s*synth2
Finished Applying XDC Timing Constraints : Time (s): cpu = 00:00:08 ; elapsed = 00:00:24 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
4
%s
*synth2
Start Timing Optimization
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2
}Finished Timing Optimization : Time (s): cpu = 00:00:08 ; elapsed = 00:00:25 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
3
%s
*synth2
Start Technology Mapping
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2~
|Finished Technology Mapping : Time (s): cpu = 00:00:08 ; elapsed = 00:00:25 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
-
%s
*synth2
Start IO Insertion
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
?
%s
*synth2'
%Start Flattening Before IO Insertion
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
B
%s
*synth2*
(Finished Flattening Before IO Insertion
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
6
%s
*synth2
Start Final Netlist Cleanup
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
9
%s
*synth2!
Finished Final Netlist Cleanup
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2x
vFinished IO Insertion : Time (s): cpu = 00:00:08 ; elapsed = 00:00:29 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
=
%s
*synth2%
#Start Renaming Generated Instances
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
¢
%s*synth2
Finished Renaming Generated Instances : Time (s): cpu = 00:00:08 ; elapsed = 00:00:29 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
:
%s
*synth2"
 Start Rebuilding User Hierarchy
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2
Finished Rebuilding User Hierarchy : Time (s): cpu = 00:00:08 ; elapsed = 00:00:29 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
9
%s
*synth2!
Start Renaming Generated Ports
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2
Finished Renaming Generated Ports : Time (s): cpu = 00:00:08 ; elapsed = 00:00:29 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
;
%s
*synth2#
!Start Handling Custom Attributes
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
 
%s*synth2
Finished Handling Custom Attributes : Time (s): cpu = 00:00:08 ; elapsed = 00:00:29 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
8
%s
*synth2 
Start Renaming Generated Nets
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2
Finished Renaming Generated Nets : Time (s): cpu = 00:00:08 ; elapsed = 00:00:29 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
9
%s
*synth2!
Start Writing Synthesis Report
h p
x
 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
/
%s
*synth2

Report BlackBoxes: 
h p
x
 
8
%s
*synth2 
+-+--------------+----------+
h p
x
 
8
%s
*synth2 
| |BlackBox name |Instances |
h p
x
 
8
%s
*synth2 
+-+--------------+----------+
h p
x
 
8
%s
*synth2 
+-+--------------+----------+
h p
x
 
/
%s*synth2

Report Cell Usage: 
h px 
2
%s*synth2
+------+-------+------+
h px 
2
%s*synth2
|      |Cell   |Count |
h px 
2
%s*synth2
+------+-------+------+
h px 
2
%s*synth2
|1     |BUFG   |     1|
h px 
2
%s*synth2
|2     |CARRY4 |    61|
h px 
2
%s*synth2
|3     |LUT1   |    17|
h px 
2
%s*synth2
|4     |LUT2   |    15|
h px 
2
%s*synth2
|5     |LUT3   |    18|
h px 
2
%s*synth2
|6     |LUT4   |    43|
h px 
2
%s*synth2
|7     |LUT5   |    26|
h px 
2
%s*synth2
|8     |LUT6   |    83|
h px 
2
%s*synth2
|9     |MUXF7  |     1|
h px 
2
%s*synth2
|10    |FDRE   |   283|
h px 
2
%s*synth2
|11    |IBUF   |    10|
h px 
2
%s*synth2
|12    |OBUF   |    21|
h px 
2
%s*synth2
+------+-------+------+
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 

%s*synth2
Finished Writing Synthesis Report : Time (s): cpu = 00:00:08 ; elapsed = 00:00:29 . Memory (MB): peak = 1617.598 ; gain = 665.055
h px 
l
%s
*synth2T
R---------------------------------------------------------------------------------
h p
x
 
`
%s
*synth2H
FSynthesis finished with 0 errors, 0 critical warnings and 1 warnings.
h p
x
 

%s
*synth2
Synthesis Optimization Runtime : Time (s): cpu = 00:00:06 ; elapsed = 00:00:28 . Memory (MB): peak = 1617.598 ; gain = 562.715
h p
x
 

%s
*synth2
Synthesis Optimization Complete : Time (s): cpu = 00:00:09 ; elapsed = 00:00:30 . Memory (MB): peak = 1617.598 ; gain = 665.055
h p
x
 
B
 Translating synthesized netlist
350*projectZ1-571h px 
ŗ
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002
00:00:00.0052

1617.5982
0.000Z17-268h px 
T
-Analyzing %s Unisim elements for replacement
17*netlist2
62Z29-17h px 
X
2Unisim Transformation completed in %s CPU seconds
28*netlist2
0Z29-28h px 
K
)Preparing netlist for logic optimization
349*projectZ1-570h px 
Q
)Pushed %s inverter(s) to %s load pin(s).
98*opt2
02
0Z31-138h px 
¶
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Netlist sorting complete. 2

00:00:002

00:00:002

1617.5982
0.000Z17-268h px 
l
!Unisim Transformation Summary:
%s111*project2'
%No Unisim elements were transformed.
Z1-111h px 
V
%Synth Design complete | Checksum: %s
562*	vivadotcl2

4ef463fbZ4-1430h px 
C
Releasing license: %s
83*common2
	SynthesisZ17-83h px 
~
G%s Infos, %s Warnings, %s Critical Warnings and %s Errors encountered.
28*	vivadotcl2
272
52
52
0Z4-41h px 
L
%s completed successfully
29*	vivadotcl2
synth_designZ4-42h px 
­
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
synth_design: 2

00:00:102

00:00:372

1617.5982

1082.836Z17-268h px 
c
%s6*runtcl2G
ESynthesis results are not added to the cache due to CRITICAL_WARNING
h px 
ø
I%sTime (s): cpu = %s ; elapsed = %s . Memory (MB): peak = %s ; gain = %s
268*common2
Write ShapeDB Complete: 2

00:00:002
00:00:00.0022

1617.5982
0.000Z17-268h px 
Ŗ
 The %s '%s' has been generated.
621*common2

checkpoint2W
UC:/Users/user/Desktop/veril/project_3_1123/project_3_1123.runs/synth_1/SS_Decoder.dcpZ17-1381h px 
”
Executing command : %s
56330*	planAhead2_
]report_utilization -file SS_Decoder_utilization_synth.rpt -pb SS_Decoder_utilization_synth.pbZ12-24828h px 
\
Exiting %s at %s...
206*common2
Vivado2
Tue Dec 17 00:38:33 2024Z17-206h px 


End Record