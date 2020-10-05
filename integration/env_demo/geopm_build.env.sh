#!/bin/bash

# GEOPM BUILD ENVIRONMENT

if [ -f $HOME/.geopmrc ]; then
    source ~/.geopmrc
fi

if [ ! -z ${GEOPM_SYSTEM_ENV} ]; then
    source ${GEOPM_SYSTEM_ENV}
fi

# check that compiler wrappers are set

CC=${CC:?Please define CC in your environment}
CXX=${CXX:?Please define CXX in your environment}
MPICC=${MPICC:?Please define MPICC in your environment}
MPICXX=${MPICXX:?Please define MPICXX in your environment}
FC=${FC:?Please define FC in your environment}
F77=${F77:?Please define F77 in your environment}
MPIFC=${MPIFC:?Please define MPIFC in your environment}
MPIF77=${MPIF77:?Please define MPIF77 in your environment}

GEOPM_INSTALL=${GEOPM_INSTALL:?Please define GEOPM_INSTALL in your environment.}

# for building application only
GEOPM_APPS_SOURCES=${GEOPM_APPS_SOURCES:?Please define GEOPM_APPS_SOURCES in your environment.}

# GEOPM_INC: Directory containing geopm.h.
export GEOPM_INC=${GEOPM_INSTALL}/include

# GEOPM_LIB: Directory containing libgeopm.so; fortran mod is in subdir
export GEOPM_LIB=$GEOPM_INSTALL/lib
export GEOPM_FORTRAN_MOD_DIR=${GEOPM_LIB}/${FC}/modules/geopm-x86_64

# GEOPM_CFLAGS: Contains compile options for geopm.
export GEOPM_CFLAGS="-I${GEOPM_INC}"
export GEOPM_FFLAGS="-I${GEOPM_FORTRAN_MOD_DIR}"

# GEOPM_LDFLAGS: Contains link options for geopm.
export GEOPM_LDFLAGS="-L${GEOPM_LIB}"
export GEOPM_LDLIBS="-lgeopm"
export GEOPM_FORTRAN_LDLIBS="${GEOPM_LDLIBS} -lgeopmfortran"

export MANPATH=\
"${GEOPM_INSTALL}/share/man:"\
"${MANPATH}"
