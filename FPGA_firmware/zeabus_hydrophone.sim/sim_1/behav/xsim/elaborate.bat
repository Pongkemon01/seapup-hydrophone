@echo off
REM ****************************************************************************
REM Vivado (TM) v2019.1 (64-bit)
REM
REM Filename    : elaborate.bat
REM Simulator   : Xilinx Vivado Simulator
REM Description : Script for elaborating the compiled design
REM
REM Generated by Vivado on Thu Feb 13 20:39:09 +0700 2020
REM SW Build 2552052 on Fri May 24 14:49:42 MDT 2019
REM
REM Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
REM
REM usage: elaborate.bat
REM
REM ****************************************************************************
echo "xelab -wto eaa226c700c643d99c29dd35889e833c --incr --debug typical --relax --mt 2 -L fifo_generator_v13_2_4 -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -L xpm --snapshot hydrophone_trigger_tb_behav xil_defaultlib.hydrophone_trigger_tb xil_defaultlib.glbl -log elaborate.log"
call xelab  -wto eaa226c700c643d99c29dd35889e833c --incr --debug typical --relax --mt 2 -L fifo_generator_v13_2_4 -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -L xpm --snapshot hydrophone_trigger_tb_behav xil_defaultlib.hydrophone_trigger_tb xil_defaultlib.glbl -log elaborate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0
