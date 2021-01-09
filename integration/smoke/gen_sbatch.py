#!/usr/bin/env python

import textwrap
import argparse
import sys
import os


def run_script_path(app, exp_type):
    exp_dir = exp_type
    if exp_type in ['barrier_frequency_sweep', 'power_balancer_energy']:
        exp_dir = 'energy_efficient'
    return "{exp_dir}/run_{exp_type}_{app}.py".format(exp_dir=exp_dir,
                                                      exp_type=exp_type)


def show_test_result(method, app, exp_type, result):

    def save_db(result):
        smoke_script = "${GEOPM_SOURCE}/integration/smoke/db_demo/smoke.py"
        return "{} --jobid=$SLURM_JOB_ID --app={} --exp-type={} ".format(
            smoke_script, app, exp_type)

    show_str = ""
    if method == 'print':
        show_str = "echo {} {}: {}".format(app, exp_type, result)
    elif method == 'db':
        show_str = save_db(result)
    else:
        raise RuntimeError("Unknown method: " + method)

    return show_str


def args_for_experiment(exp_type):
    args = ""
    if exp_type in ['frequency_sweep', 'barrier_frequency_sweep']:
        args = "--min-frequency=1.9e9 --max-frequency=2.0e9"
    elif exp_type in ['power_sweep', 'power_balancer_energy']:
        args = "--min-power=190 --max-power=230"
    elif exp_type in ['uncore_frequency_sweep']:
        args = "--min-frequency=1.9e9 --max-frequency=2.0e9 --min-uncore-frequency=2.1e9 --max-uncore-frequency=2.2e9"
    else:
        raise RuntimeError("Unknown exp_type: {}".format(exp_type))
    return args


def gen_sbatch(app, exp_type, node_count):

    # TODO: pass in?
    geopm_exp = "/home/drguttma/integration/experiment"
    script = run_script_path(app, exp_type)
    if not os.path.exists(os.path.join(geopm_exp, script)):
        # Note: this early fail should be detected by the calling bash
        # script run_smoke_parallel.sh.  It indicates that no slurm sbatch
        # script was generated, so don't try to submit.
        sys.exit(1)

    # switch between using DB or just print
    # this would be better with a real class
    # could also append to a log file.
    # TODO: how to also handle reporting skipped tests in the same way?
    method = "print"
    pass_result = show_test_result(method, app, exp_type, "PASS")
    fail_result = show_test_result(method, app, exp_type, "FAIL")

    args = args_for_experiment(exp_type)

    template = textwrap.dedent('''\
    #!/bin/bash
    #SBATCH -N {node_count}
    #SBATCH -t 4:00:00
    #SBATCH -o %j.out
    #SBATCH -J {app}_{exp_type}

    if [ -f $HOME/.geopmrc ]; then
        source ~/.geopmrc
    fi
    GEOPM_SOURCE=${GEOPM_SOURCE:?"Please define GEOPM_SOURCE in your environment"}

    if [ ! -z ${GEOPM_SYSTEM_ENV} ]; then
        source ${GEOPM_SYSTEM_ENV}
    fi

    source ${GEOPM_SOURCE}/integration/config/run_env.sh
    OUTPUT_DIR=${{SLURM_JOB_NAME}}_${{SLURM_JOBID}}

    GEOPM_EXP=${{GEOPM_SOURCE}}/integration/experiment

    SCRIPT=$GEOPM_EXP/{script}
    if [ -f $SCRIPT ]; then
        $SCRIPT --output-dir=$OUTPUT_DIR \\
        --node-count=$SLURM_NNODES \\
        --trial-count=1 \\
        {args} \\
        # end

        result=$?
        if [ $result -eq 0 ]; then
            {pass_result}
        else
            {fail_result}
        fi
    else
        echo "${SCRIPT} not found"
    fi

    '''.format(script=script, exp_type=exp_type, app=app, args=args, node_count=node_count,
               pass_result=pass_result, fail_result=fail_result))

    filename = '{}_{}.sbatch'.format(app, exp_type)

    with open(filename, 'w') as outfile:
        outfile.write(template)

    return filename


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--app', type=str, required=True,
                        help='application name')
    parser.add_argument('--exp-type', type=str, dest='exp_type', required=True,
                        help='experiment type')
    parser.add_argument('--node-count', type=int, dest='node_count', required=True, help='node count')
    args = parser.parse_args()

    app_name = args.app
    exp_type = args.exp_type
    node_count = args.node_count

    sbatch_name = gen_sbatch(app=app_name, exp_type=exp_type, node_count=node_count)
    sys.stdout.write(sbatch_name)
    sys.exit(0)
