#!/bin/bash
#
#  Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in
#        the documentation and/or other materials provided with the
#        distribution.
#
#      * Neither the name of Intel Corporation nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

# GEOPM_PREFIX: Where to find lib and include directories for geopm.
if [ ! "$GEOPM_PREFIX" ]; then
    GEOPM_PREFIX=$HOME/build/geopm
fi

# GEOPM_BINDIR: Directory containing geopmbench.
if [ ! "$GEOPM_BINDIR" ]; then
    GEOPM_BINDIR=$GEOPM_PREFIX/bin
fi

# GEOPM_LIBDIR: Directory containing libgeopm.so.
if [ ! "$GEOPM_LIBDIR" ]; then
    GEOPM_LIBDIR=$GEOPM_PREFIX/lib
fi

# GEOPMPY_PKGDIR: Directory containing goepmpy.
if [ ! "$GEOPMPY_PKGDIR" ]; then
    GEOPMPY_PKGDIR=$GEOPM_PREFIX/lib/python2.7/site-packages
fi

export PATH=$GEOPM_BINDIR:$PATH
export PYTHONPATH=$GEOPMPY_PKGDIR:$PYTHONPATH
export LD_LIBRARY_PATH=$GEOPM_LIBDIR:$LD_LIBRARY_PATH

# geopmbench configuration
# run dgemm so that the app would hit TDP if not under a power cap
echo "{\"loop-count\": 500, \"big-o\": [8.0], \"region\": [\"dgemm\"]}" > dgemm_config.json

# power cap of 200, since package min is 96W and DRAM uses ~80W.
#power_cap=200
power_cap=230
geopmagent -a power_governor -p ${power_cap} > governor_${power_cap}.json
geopmsrun -N 4 -n 16 \
	  --geopm-ctl=process \
	  --geopm-agent=power_governor \
	  --geopm-policy=governor_${power_cap}.json \
	  --geopm-report=isc18_governor_${power_cap}.report \
	  --geopm-trace=isc18_governor_${power_cap}.trace \
	  geopmbench dgemm_config.json
