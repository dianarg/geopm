# TODO: use common env scripts
GEOPM_SRC=$HOME/geopm
GEOPM_EXP=$GEOPM_SRC/integration/experiment

export MPI_ROOT=$(which mpiicc | grep -o ".*/^Ci/")
source $HOME/geopm_apps/integration/apps/openfoam/OpenFOAM-v2006/etc/bashrc || true
source ~/env.sh

module swap mvapich2 impi || true

# TODO: fix path
export OPENFOAM_APP_DIR=$HOME/geopm_apps/integration/apps/openfoam
