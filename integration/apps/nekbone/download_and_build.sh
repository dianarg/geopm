#!/bin/bash

system_names='mcfly'
if [[ $# -ne 1 ]] || [[ $1 == '--help' ]]; then
    echo "Usage: $0: system_name"
    echo "    Available systems: $system_names"
    exit -1
fi
system=$1

# Acquire the source:
wget https://asc.llnl.gov/CORAL-benchmarks/Science/nekbone-2.3.4.tar.gz

# Unpack the source:
tar zxvf nekbone-2.3.4.tar.gz

# Change directories to the unpacked files.
cd nekbone-2.3.4

# Patch nekbone with the patch utility:
system_patch=nekbone_build_$system.patch
if [[ ! -e ../$system_patch ]]; then
    echo "Error: system patch file does not exist: $system_patch" 1>&2
    exit -1
fi
patch -p1 < ../0001-Optimized-nekbone-code-for-Intel-hardware.patch
patch -p1 < ../0002-Marked-up-nekbone-code-for-use-with-GEOPM.patch
patch -p1 < ../0003-Increase-problem-size-for-Theta.patch
patch -p1 < ../$system_patch

# Build
cd test/example1
./makenek
