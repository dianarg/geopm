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
import pandas
import glob
import time

import geopmpy.io

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from integration.util import try_launch


class PowerSweep:
    """
    Runs the application under a range of socket power limits.  Used
    by other analysis types to run either the PowerGovernorAgent or
    the PowerBalancerAgent.
    """
    def __init__(self, profile_prefix, output_dir, detailed, iterations,
                 min_power, max_power, step_power, agent_types):
        self._name = profile_prefix + "_power_sweep"
        self._output_dir = output_dir
        self._iterations = iterations
        self._min_power = min_power
        self._max_power = max_power
        self._step_power = step_power
        self._agent_types = agent_types

    # TODO: use common AppConf interface to hold the args
    def launch(self, num_node, num_rank, launcher_name, args):
        for iteration in range(self._iterations):
            for power_cap in range(self._min_power, self._max_power+1, self._step_power):
                for agent in self._agent_types:
                    options = {'power_budget': power_cap}
                    # TODO: check for valid agent type; this should be reusable between all agent types
                    agent_conf = geopmpy.io.AgentConf(path=self._name + '_agent.config',
                                                      agent=agent,
                                                      options=options)
                    agent_conf.write()

                    uid = '{}_{}_{}_{}'.format(self._name, agent, power_cap, iteration)
                    report_path = os.path.join(self._output_dir, '{}.report'.format(uid))
                    trace_path = os.path.join(self._output_dir, '{}.trace'.format(uid))
                    profile_name = 'iteration_{}'.format(iteration)

                    try_launch(launcher_name=launcher_name, app_argv=args,
                               report_path=report_path, trace_path=trace_path,
                               profile_name=profile_name, agent_conf=agent_conf)

                    # rest to cool off between runs
                    time.sleep(60)

    # TODO: utility function?  only needs the name
    def find_report_files(self, search_pattern='*report'):
        """
        Uses the output dir and any custom name prefix to discover reports
        produced by launch.
        """
        # TODO: add output dir
        reports = glob.glob(self._name + search_pattern)
        reports = sorted(reports)
        return reports
