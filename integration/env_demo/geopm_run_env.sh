#!/bin/bash

# GEOPM RUNTIME ENVIRONMENT

if [ -f $HOME/.geopmrc ]; then
    source ~/.geopmrc
fi

GEOPM_INSTALL=${GEOPM_INSTALL:?Please define GEOPM_INSTALL in your environment.}

# for running integration only
GEOPM_SOURCE=${GEOPM_SOURCE:?Please define GEOPM_SOURCE in your environment.}
# for integration sbatch scripts
GEOPM_WORKDIR=${GEOPM_WORKDIR:?Please set GEOPM_WORKDIR in your environment.}

# GEOPM_LIB: Directory containing libgeopm.so.
GEOPM_LIB=$GEOPM_INSTALL/lib

# Use whichever python version was used to build geopmpy
GEOPMPY_PKGDIR=$(ls -dv $GEOPM_LIB/python*/site-packages 2>/dev/null)
if [ 0 -eq $? ]; then
    if [ 1 -ne $(echo "$GEOPMPY_PKGDIR" | wc -l) ]; then
        GEOPMPY_PKGDIR=$(echo "$GEOPMPY_PKGDIR" | tail -n1)
        echo 1>&2 "Warning: More than 1 python site-packages directory in $GEOPM_LIB"
        echo 1>&2 "         Remove all except one, or manually set GEOPMPY_PKGDIR."
        echo 1>&2 "         Assuming GEOPMPY_PKGDIR=${GEOPMPY_PKGDIR}."
    fi
else
    echo 1>&2 "Warning: Unable to find python site-packages in $GEOPM_LIB"
fi

export PYTHONPATH=${GEOPMPY_PKGDIR}:${PYTHONPATH}

# for integration scripts only
export PYTHONPATH=${GEOPM_SRC}/integration:${PYTHONPATH}

export PATH=$GEOPM_INSTALL/bin:$PATH
export LD_LIBRARY_PATH=$GEOPM_LIB:$LD_LIBRARY_PATH

export MANPATH=\
"${GEOPM_INSTALL}/share/man:"\
"${MANPATH}"

if [ ! -z ${GEOPM_SYSTEM_ENV} ]; then
    source ${GEOPM_SYSTEM_ENV}
fi
