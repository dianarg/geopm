#!/bin/bash

source smoke_env.sh

#ALL_APPS="dgemm_tiny amg dgemm minife nekbone hpcg nasft"
#ALL_EXP="monitor power_sweep frequency_sweep power_balancer_energy barrier_frequency_sweep"

APPS_2NODE="2nodedummy"  #"dgemm_tiny dgemm nasft nekbone hpcg pennant hpl_mkl hpl_netlib"
# apps with no 2-node config
APPS_1NODE="1nodedummy"  #"minife amg"
ALL_EXP="monitor"  #"monitor power_sweep power_balancer_energy frequency_sweep barrier_frequency_sweep uncore_frequency_sweep"
for exp in $ALL_EXP; do
    for app in ${APPS_2NODE}; do
        SCRIPT=${EXP_DIR}/run_${exp}_${app}.py
        if [ -f $SCRIPT ]; then
            ./gen_sbatch.py --app=${app} --exp-type=${exp} --node-count=2
            sbatch ${app}_${exp}.sbatch
        else
            ${GEOPM_SOURCE}/integration/smoke/db_demo/smoke.py --app=${app} --exp-type=${exp} --result="No script"
        fi
    done

    for app in ${APPS_1NODE}; do
        SCRIPT=${EXP_DIR}/run_${exp}_${app}.py
        if [ -f $SCRIPT ]; then
            ./gen_sbatch.py --app=${app} --exp-type=${exp} --node-count=1
            sbatch ${app}_${exp}.sbatch
        else
            ${GEOPM_SOURCE}/integration/smoke/db_demo/smoke.py --app=${app} --exp-type=${exp} --result="No script"
        fi
    done

done
