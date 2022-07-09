#!/bin/bash -f
# ****************************************************************************
# Vivado (TM) v2020.2 (64-bit)
#
# Filename    : simulate.sh
# Simulator   : Xilinx Vivado Simulator
# Description : Script for simulating the design by launching the simulator
#
# Generated by Vivado on Mon Jul 04 15:43:19 +07 2022
# SW Build 3064766 on Wed Nov 18 09:12:47 MST 2020
#
# Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
#
# usage: simulate.sh
#
# ****************************************************************************
set -Eeuo pipefail
# simulate design
echo "xsim fx3_interface_tb_func_synth -key {Post-Synthesis:sim_fx3s_interface:Functional:fx3_interface_tb} -tclbatch fx3_interface_tb.tcl -view /home/stp/seapup/zeabus_hydrophone/FPGA_firmware/fx3_interface_tb_func_synth.wcfg -log simulate.log"
xsim fx3_interface_tb_func_synth -key {Post-Synthesis:sim_fx3s_interface:Functional:fx3_interface_tb} -tclbatch fx3_interface_tb.tcl -view /home/stp/seapup/zeabus_hydrophone/FPGA_firmware/fx3_interface_tb_func_synth.wcfg -log simulate.log

