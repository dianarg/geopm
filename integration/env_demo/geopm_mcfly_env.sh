#!/bin/bash

# Example system config pointed to by user's GEOPM_SYSTEM_ENV

CC=icc
CXX=icpc
MPICC=mpicc
MPICXX=mpic++
FC=ifort
F77=ifort
MPIFC=mpifort
MPIF77=mpifort

export CC CXX MPICC MPICXX FC F77 MPIFC MPIF77

# check modules for build...

# system-specific values
export GEOPM_SLURM_DEFAULT_QUEUE=all

export GEOPM_SBATCH_EXTRA_LINES="\
#SBATCH --disable-ear
#SBATCH --cpu-freq=Performance"
