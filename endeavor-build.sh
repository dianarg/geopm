#!/bin/bash

make dist
sdir=geopm-$(cat VERSION)
tball=$sdir.tar.gz
panfs=/panfs/users/$USER
scp $tball endeavor:$panfs
script="set -e; \
cd $panfs; \
tar xvf $tball; \
cd $sdir; \
source /opt/intel/compiler/latest/bin/compilervars.sh intel64; \
source /opt/intel/impi/latest/compilers_and_libraries/linux/mpi/intel64/bin/mpivars.sh; \
CC=icc CXX=icpc FORT=ifort MPICC=mpiicc MPICXX=mpiicpc ./configure --prefix=$panfs/build/$sdir --with-python=python2 --disable-doc; \
make -j8; \
make install;"
ssh endeavor $script

