#!/bin/bash
#SBATCH -N 32
#SBATCH -t 4:00:00
#SBATCH -o %j.out


source openfoam_env.sh

# TODO: doesn't work
#export I_MPI_PIN_PROCESSOR=allcores:map=spread
# OPA specific

set -x

export I_MPI_FABRICS=shm:ofi
export FI_PROVIDER=psm2

# |-------+-------+-------+----------|
# |   X   |   Y   |   Z   |  MCells  | Solve time (2 nodes)
# |-------+-------+-------+----------|
# |    20 |     8 |     8 |     0.35 | 20 sec
# |    60 |    24 |    24 |     5.38 | 178
# |    80 |    32 |    32 |    11.2  |
# |    90 |    36 |    36 |    15.5  |
# |   100 |    40 |    40 |    20    |
# |-------+-------+-------+----------|
# Other sizes:
# 42M cell: 130 52 52

# copied from prepare script:
# toy input
# NX=20
# NY=8
# NZ=8
# export NX=60
# export NY=24
# export NZ=24

export NX=130
export NY=52
export NZ=52

# TODO: make these inputs
export NODES=$SLURM_NNODES
# mcfly has 44 total, but reserve some for GEOPM + OS
export CORES_PER_NODE=34
export NPROCS=$(($NODES * $CORES_PER_NODE))

SETUP_RESULT_DIR=motorbike_${NPROCS}ranks_${NX}_${NY}_${NZ}

export OPENFOAM_APP_DIR=$HOME/geopm/integration/apps/openfoam
${OPENFOAM_APP_DIR}/prepare_input.sh
if [ $? -ne 0 ]; then
    echo "Failed to setup problem inputs."
    exit 1
fi

# bash setup
OUTPUT_DIR=${SLURM_JOB_ID}_with_geopm
mkdir $OUTPUT_DIR
cd ${OUTPUT_DIR}  # already in infra

cp -r ${OPENFOAM_APP_DIR}/${SETUP_RESULT_DIR} .
cd ${SETUP_RESULT_DIR}
. $WM_PROJECT_DIR/bin/tools/RunFunctions

# exec args
geopmlaunch srun -N $NODES -n $NPROCS --geopm-ctl=application --geopm-report=simplefoam.report -- simpleFoam -parallel

# bash cleanup
./Clean

cd ${OPENFOAM_APP_DIR}

###
export CORES_PER_NODE=36
export NPROCS=$(($NODES * $CORES_PER_NODE))
SETUP_RESULT_DIR=motorbike_${NPROCS}ranks_${NX}_${NY}_${NZ}
${OPENFOAM_APP_DIR}/prepare_input.sh
if [ $? -ne 0 ]; then
    echo "Failed to setup problem inputs."
    exit 1
fi

OUTPUT_DIR=${SLURM_JOB_ID}_no_geopm
mkdir $OUTPUT_DIR
cd ${OUTPUT_DIR}

cp -r ${OPENFOAM_APP_DIR}/${SETUP_RESULT_DIR} .
cd ${SETUP_RESULT_DIR}
. $WM_PROJECT_DIR/bin/tools/RunFunctions

# exec args

date
srun -N $NODES -n $NPROCS -- simpleFoam -parallel
date

# bash cleanup
./Clean
