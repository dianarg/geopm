#!/bin/bash

set -e

# TODO: fix me
source ${OPENFOAM_APP_DIR}/openfoam_env.sh

# Use 42M cell workload
# NX=130
# NY=52
# NZ=52

# toy input
# NX=20
# NY=8
# NZ=8

# # TODO: make these inputs
# #NODES=$SLURM_NNODES
# NODES=2
# # mcfly has 44 total, but reserve some for GEOPM + OS
# CORES_PER_NODE=42
# NPROCS=$(($NODES * $CORES_PER_NODE))

# reusable result of MESH and SETUP steps
MESH_RESULT_DIR=motorbike_mesh_${NX}_${NY}_${NZ}
SETUP_RESULT_DIR=motorbike_${NPROCS}ranks_${NX}_${NY}_${NZ}

if [ -d ${OPENFOAM_APP_DIR}/${SETUP_RESULT_DIR} ]; then
    echo "Found existing decomposed mesh in $SETUP_RESULT_DIR "
else
    echo "No decomposition for $NX $NY $NZ and ${NPROCS} ranks"
    if [ -d ${OPENFOAM_APP_DIR}/${MESH_RESULT_DIR} ]; then
        echo "Found existing mesh in $MESH_RESULT_DIR"
    else
        echo "No mesh for $NX $NY $NZ"
        cd ${OPENFOAM_APP_DIR}
        mkdir -p ${MESH_RESULT_DIR}
        cd ${MESH_RESULT_DIR}
        # this should have been checked out by build.sh
        cp -r ${OPENFOAM_APP_DIR}/OpenFOAM-Intel/benchmarks/motorbike/* .

        ./Mesh $NX $NY $NZ
    fi

    cd ${OPENFOAM_APP_DIR}
    mkdir -p ${SETUP_RESULT_DIR}
    cd ${SETUP_RESULT_DIR}
    cp -r ${OPENFOAM_APP_DIR}/${MESH_RESULT_DIR}/* .

    ./Setup ${NPROCS} ${NODES}
fi
