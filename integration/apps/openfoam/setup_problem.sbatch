#!/bin/bash
#SBATCH -N 8
#SBATCH -t 4:00:00
#SBATCH -o %j.out

GEOPM_SRC=$HOME/geopm
GEOPM_EXP=$GEOPM_SRC/integration/experiment

export MPI_ROOT=$(which mpiicc | grep -o ".*/^Ci/")
source $HOME/geopm_apps/integration/apps/openfoam/OpenFOAM-v2006/etc/bashrc
source ~/env.sh

module swap mvapich2 impi

which geopmlaunch
which potentialFoam
which simpleFoam
which mpirun

OUTDIR=${SLURM_JOB_ID}_openfoam_monitor

mkdir -p $OUTDIR
cd $OUTDIR


# This application does not use OpenMP, but the following
# helps GEOPM do a better job of pinning the ranks to cores
# TODO: specify get_cpus_per_rank() in openfoam.py
export OMP_NUM_THREADS=1

#### Mesh

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

# NX=20
# NY=8
# NZ=8

# NX=60
# NY=24
# NZ=24

NX=130
NY=52
NZ=52

NODES=$SLURM_NNODES
# mcfly has 44 total, but reserve some for GEOPM + OS
CORES_PER_NODE=42
NPROCS=$(($NODES * $CORES_PER_NODE))

# reusable result of MESH and SETUP steps
MESH_RESULT_DIR=motorbike_mesh_${NX}_${NY}_${NZ}
SETUP_RESULT_DIR=motorbike_${NPROCS}ranks_${NX}_${NY}_${NZ}

OPENFOAM_APP_DIR=$HOME/geopm_apps/integration/apps/openfoam

if [ -d ${OPENFOAM_APP_DIR}/$SETUP_RESULT_DIR ]; then
    echo "Found existing decomposed mesh in $SETUP_RESULT_DIR "
else
    echo "No decomposition for $NX $NY $NZ and ${NPROCS} ranks"
    if [ -d ${OPENFOAM_APP_DIR}/$MESH_RESULT_DIR ]; then
        echo "Found existing mesh in $MESH_RESULT_DIR"
    else
        ./Mesh $NX $NY $NZ

        #### Begin Mesh generation
        # Get workload from https://github.com/OpenFOAM/OpenFOAM-Intel.git
        cp -r ${OPENFOAM_APP_DIR}/motorbike_example .
        cd motorbike_example

        # TODO: might want to use more procs for meshing if it takes too long; update system/decomposeParDict-mesh.insystem/decomposeParDict-mesh.in
        SETUP_PROCS=16
        echo "Setting up Mesh with $SETUP_PROCS procs.  Blockmesh size = $NX x $NY x $NZ"
        mkdir -p constant/polyMesh
        mkdir -p constant/triSurface
        sed "s/NX/$NX/g;s/NY/$NY/g;s/NZ/$NZ/g" system/blockMeshDict-mesh.in > constant/polyMesh/blockMeshDict
        cp system/decomposeParDict-mesh.in system/decomposeParDict
        # Source tutorial run functions
        . $WM_PROJECT_DIR/bin/tools/RunFunctions
        # copy motorbike surface from resources directory
        cp $FOAM_TUTORIALS/resources/geometry/motorBike.obj.gz constant/triSurface/
        runApplication surfaceFeatureExtract > surfaceFeatureExtract.$LINENO.out
        runApplication blockMesh > blockMesh.$LINENO.out
        runApplication decomposePar > decomposePar.$LINENO.out
        #runParallel snappyHexMesh -overwrite
        srun -N $NODES -n 16 snappyHexMesh -overwrite -parallel > snappyHexMesh.$LINENO.out
        runApplication reconstructParMesh -constant > reconstructParMesh.$LINENO.out
        rm -rf processor*
        runApplication renumberMesh -constant -overwrite > renumberMesh.$LINENO.out
        ####### end Mesh

        cd ..
        cp -r motorbike_example ${OPENFOAM_APP_DIR}/$MESH_RESULT_DIR
    fi

    cp -r ${OPENFOAM_APP_DIR}/$MESH_RESULT_DIR .
    cd $MESH_RESULT_DIR

    ###### Setup
    echo "Running Setup with $NPROCS procs."
    rm -f log.*
    rm -rf processor*
    sed "s/NPROCS/$NPROCS/g" system/decomposeParDict-solve.in > system/decomposeParDict
    # Source tutorial run functions
    . $WM_PROJECT_DIR/bin/tools/RunFunctions
    runApplication decomposePar > decomposePar.$LINENO.out
    #- For parallel running
    ls -d processor* | xargs -I {} rm -rf ./{}/0
    ls -d processor* | xargs -I {} cp -r 0.org ./{}/0
    #runParallel potentialFoam
    srun -N $NODES -n $NPROCS potentialFoam -parallel > potentialFoam.$LINENO.out
    ##### end Setup

    # save off result of setup
    cd ..
    cp -r $MESH_RESULT_DIR ${OPENFOAM_APP_DIR}/$SETUP_RESULT_DIR
fi

##### Solve
cp -r ${OPENFOAM_APP_DIR}/$SETUP_RESULT_DIR .
cd $SETUP_RESULT_DIR

echo "Running Solve with $NPROCS procs."

# Source tutorial run functions
. $WM_PROJECT_DIR/bin/tools/RunFunctions

# NOTE: To launch without the runParallel, do the following:
#
#           mpirun -np $NPROCS simpleFoam -parallel
#
#runParallel simpleFoam

# IMPI launch: doesn't work....
#PPN=$(($NPROCS / $NODES))
#scontrol show hostname $SLURM_NODELIST > hostfile
#geopmlaunch impi -n $NPROCS -ppn $PPN -hosts hostfile -- simpleFoam -parallel
#mpiexec.hydra -n $NPROCS -ppn $PPN -hosts hostfile -- simpleFoam -parallel

# SRUN alone: works
#srun -N $NODES -n $NPROCS -- simpleFoam -parallel

# SRUN with geopm process mode -  stuck at end
#geopmlaunch srun -N $NODES -n $NPROCS  --geopm-report=simplefoam.report -- simpleFoam -parallel

# SRUN with geopm application mode - works!!!
geopmlaunch srun -N $NODES -n $NPROCS --geopm-ctl=application --geopm-report=simplefoam.report -- simpleFoam -parallel

./Clean

# application trace - VERY LARGE
#geopmlaunch srun -N $NODES -n $NPROCS --geopm-ctl=application --geopm-trace-profile=simplefoam.ptrace --geopm-report=simplefoam.report -- simpleFoam -parallel


##### end Solve
