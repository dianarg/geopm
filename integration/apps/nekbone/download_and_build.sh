#!/bin/bash

# Acquire the source:
wget https://asc.llnl.gov/CORAL-benchmarks/Science/nekbone-2.3.4.tar.gz

# Unpack the source:
tar zxvf nekbone-2.3.4.tar.gz

# Change directories to the unpacked files.
cd nekbone-2.3.4

# Patch nekbone with the patch utility:
patch -p1 < ../0001-Optimized-nekbone-code-for-Intel-hardware.patch
patch -p1 < ../0002-Marked-up-nekbone-code-for-use-with-GEOPM.patch
patch -p1 < ../0003-Increase-problem-size-for-Theta.patch

# Build
