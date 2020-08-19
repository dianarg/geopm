#  Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in
#        the documentation and/or other materials provided with the
#        distribution.
#
#      * Neither the name of Intel Corporation nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

'''
Helper functions for running the frequency map and energy efficient agents.
'''

import time
import os
import json
from experiment import machine
from experiment import util
import geopmpy.io


def launch_frequency_map_agent(output_dir, iterations,
                               num_node, app_conf,
                               policy_file, reference_freq,
                               experiment_cli_args, cool_off_time=60):

    machine.init_output_dir(output_dir)
    rank_per_node = app_conf.get_rank_per_node()
    num_rank = num_node * rank_per_node

    report_sig = ["CYCLES_THREAD@package", "CYCLES_REFERENCE@package",
                  "TIME@package", "ENERGY_PACKAGE@package"]

    extra_cli_args = ['--geopm-report-signals=' + ','.join(report_sig)]
    extra_cli_args += experiment_cli_args

    # Reference run at fixed frequency
    options = {'FREQ_DEFAULT': reference_freq}
    ref_agent_conf = geopmpy.io.AgentConf(path=os.path.join(output_dir, 'ref.conf'),
                                          agent='frequency_map',
                                          options=options)
    ref_agent_conf.write()

    # Agent run with policy
    # AgentConf class isn't exactly right for this purpose

    # just set geopm command line here instead of in launch_run()
    # TODO: doesn't work
    # target_cli_args = extra_cli_args
    # target_cli_args += ['--geopm-agent', 'frequency_map']
    # target_cli_args += ['--geopm-policy', policy_file]

    with open(policy_file) as fid:
        options = json.load(fid)
    # TODO: check that policy_file != target.conf
    target_agent_conf = geopmpy.io.AgentConf(path=os.path.join(output_dir, 'target.conf'),
                                             agent='frequency_map',
                                             options=options)
    target_agent_conf.write()

    for iteration in range(iterations):
        # TODO: add monitor run

        run_id = 'ref_{}'.format(iteration)
        util.launch_run(agent_conf=ref_agent_conf,
                        app_conf=app_conf,
                        run_id=run_id,
                        output_dir=output_dir,
                        extra_cli_args=extra_cli_args,
                        num_node=num_node, num_rank=num_rank)  # raw launcher factory args

        run_id = 'target_{}'.format(iteration)
        util.launch_run(agent_conf=target_agent_conf,
                        app_conf=app_conf,
                        run_id=run_id,
                        output_dir=output_dir,
                        extra_cli_args=extra_cli_args,
                        num_node=num_node, num_rank=num_rank)  # raw launcher factory args

        # rest to cool off between runs
        time.sleep(cool_off_time)


def launch_energy_efficient_agent(output_dir, iterations,
                                  num_node, app_conf,
                                  reference_freq,
                                  experiment_cli_args, cool_off_time=60):

    machine.init_output_dir(output_dir)
    rank_per_node = app_conf.get_rank_per_node()
    num_rank = num_node * rank_per_node

    report_sig = ["CYCLES_THREAD@package", "CYCLES_REFERENCE@package",
                  "TIME@package", "ENERGY_PACKAGE@package"]

    extra_cli_args = ['--geopm-report-signals=' + ','.join(report_sig)]
    extra_cli_args += experiment_cli_args

    # Reference run at fixed frequency
    options = {'FREQ_DEFAULT': reference_freq}
    ref_agent_conf = geopmpy.io.AgentConf(path=os.path.join(output_dir, 'ref.conf'),
                                          agent='frequency_map',
                                          options=options)
    ref_agent_conf.write()

    # Agent run with unbounded policy
    options = {'FREQ_MIN': float('nan'),
               'FREQ_MAX': float('nan')}
    target_agent_conf = geopmpy.io.AgentConf(path=os.path.join(output_dir, 'target.conf'),
                                             agent='energy_efficient',
                                             options=options)
    target_agent_conf.write()

    for iteration in range(iterations):
        run_id = 'ref_{}'.format(iteration)
        util.launch_run(agent_conf=ref_agent_conf,
                        app_conf=app_conf,
                        run_id=run_id,
                        output_dir=output_dir,
                        extra_cli_args=extra_cli_args,
                        num_node=num_node, num_rank=num_rank)  # raw launcher factory args

        run_id = 'target_{}'.format(iteration)
        util.launch_run(agent_conf=target_agent_conf,
                        app_conf=app_conf,
                        run_id=run_id,
                        output_dir=output_dir,
                        extra_cli_args=extra_cli_args,
                        num_node=num_node, num_rank=num_rank)  # raw launcher factory args

        # rest to cool off between runs
        time.sleep(cool_off_time)
