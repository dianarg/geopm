#!/usr/bin/env python
#
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

import sys
import os
import time

import geopmpy.io

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from integration.util import try_launch


def launch_power_sweep(file_prefix, output_dir, iterations,
                       min_power, max_power, step_power, agent_types,
                       num_node, num_rank, app_conf):
    """
    Runs the application under a range of socket power limits.  Used
    by other analysis types to run either the PowerGovernorAgent or
    the PowerBalancerAgent.
    """
    name = file_prefix + "_power_sweep"
    for iteration in range(iterations):
        for power_cap in range(min_power, max_power+1, step_power):
            for agent in agent_types:
                options = {'power_budget': power_cap}
                # TODO: check for valid agent type; this should be reusable between all agent types
                agent_conf = geopmpy.io.AgentConf(path=name + '_agent.config',
                                                  agent=agent,
                                                  options=options)
                agent_conf.write()

                uid = '{}_{}_{}_{}'.format(name, agent, power_cap, iteration)
                report_path = os.path.join(output_dir, '{}.report'.format(uid))
                trace_path = os.path.join(output_dir, '{}.trace'.format(uid))
                profile_name = 'iteration_{}'.format(iteration)

                # TODO: these are not passed to launcher create()
                # some are generic enough they could be, though
                run_args = ['--geopm-report', report_path,
                            '--geopm-trace', trace_path,
                            '--geopm-profile', profile_name]
                # any arguments after run_args are passed directly to launcher
                try_launch(agent_conf, app_conf, run_args,
                           num_node=num_node, num_rank=num_rank)

                # rest to cool off between runs
                time.sleep(60)
