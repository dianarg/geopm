# TODO: use common env scripts
GEOPM_SRC=$HOME/geopm
GEOPM_EXP=$GEOPM_SRC/integration/experiment

#module swap mvapich2 impi || true
module purge
module load intel impi

module list

export MPI_ROOT=$(which mpiicc | grep -o ".*/^Ci/")
echo $MPI_ROOT

export OPENFOAM_APP_DIR=${GEOPM_SRC}/integration/apps/openfoam

source ${OPENFOAM_APP_DIR}/OpenFOAM-v2006/etc/bashrc || true
source ~/env.sh

