#!/bin/bash -f
# ****************************************************************************
# Vivado (TM) v2020.2 (64-bit)
#
# Filename    : simulate.sh
# Simulator   : Xilinx Vivado Simulator
# Description : Script for simulating the design by launching the simulator
#
# Generated by Vivado on Tue Jun 21 09:06:05 +07 2022
# SW Build 3064766 on Wed Nov 18 09:12:47 MST 2020
#
# Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
#
# usage: simulate.sh
#
# ****************************************************************************
set -Eeuo pipefail
# simulate design
echo "xsim poten_interface_tb_func_synth -key {Post-Synthesis:sim_poten_int:Functional:poten_interface_tb} -tclbatch poten_interface_tb.tcl -log simulate.log"
xsim poten_interface_tb_func_synth -key {Post-Synthesis:sim_poten_int:Functional:poten_interface_tb} -tclbatch poten_interface_tb.tcl -log simulate.log

