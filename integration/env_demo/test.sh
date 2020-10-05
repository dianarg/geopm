#!/bin/bash

# Top-level script that checks command line args

if [ $# -ne 4 ]; then
    echo "Usage $0 <num_nodes> <app> <exp_dir> <exp_type>"
    exit 1
fi


if [ -f $HOME/.geopmrc ]; then
    source ~/.geopmrc
fi
GEOPM_SOURCE=${GEOPM_SOURCE:?"Please define GEOPM_SOURCE in your environment"}

if [ ! -z ${GEOPM_SYSTEM_ENV} ]; then
    source ${GEOPM_SYSTEM_ENV}
fi

# check for additional sbatch arguments
if [ ! -z ${GEOPM_SLURM_ACCOUNT} ]; then
    SBATCH_ACCOUNT_LINE="#SBATCH -A $GEOPM_USER_ACCOUNT"
fi

if [ ! -z ${GEOPM_SLURM_DEFAULT_QUEUE} ]; then
   SBATCH_QUEUE_LINE="#SBATCH -p $GEOPM_SYSTEM_DEFAULT_QUEUE"
fi

NUM_NODES=$1
APP=$2
EXP_DIR=$3
EXP_TYPE=$4


cat > test.sbatch << EOF
#!/bin/bash
#SBATCH -N $NUM_NODES
#SBATCH -o %j.out
${SBATCH_QUEUE_LINE}
${SBATCH_ACCOUNT_LINE}
${GEOPM_SBATCH_EXTRA_LINES}

# not needed because geopm_run_env.sh will source it
#source ${HOME}/.geopmrc

source ${GEOPM_SOURCE}/integration/geopm_run_env.sh

echo ${GEOPM_SOURCE}/integration/experiment/${EXP_DIR}/run_${EXP_TYPE}_${APP}.py

EOF

# TODO: submit for user or just generate?
sbatch test.sbatch
