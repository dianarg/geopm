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
                 min_power, max_power, step_power, agent_type):
        self._name = "power_sweep"
        self._output_dir = output_dir
        self._iterations = iterations
        self._min_power = min_power
        self._max_power = max_power
        self._step_power = step_power
        self._agent_type = agent_type

    def launch(self, num_node, num_rank, launcher_name, args):

        for power_cap in range(self._min_power, self._max_power+1, self._step_power):
            # governor runs
            options = {'power_budget': power_cap}
            agent_conf = geopmpy.io.AgentConf(path=self._name + '_agent.config',
                                              agent=self._agent_type,
                                              options=options)
            agent_conf.write()

            for iteration in range(self._iterations):
                profile_name = self._name + '_' + str(power_cap)
                report_path = os.path.join(self._output_dir, profile_name + '_{}_{}.report'.format(self._agent_type, iteration))
                trace_path = os.path.join(self._output_dir, profile_name + '_{}_{}.trace'.format(self._agent_type, iteration))
                try_launch(launcher_name=launcher_name, app_argv=args,
                           report_path=report_path, trace_path=trace_path,
                           profile_name=profile_name, agent_conf=agent_conf)

    # TODO: given that this is a member, can't it use the min and max power?
    def find_report_files(self, search_pattern='*report'):
        """
        Uses the output dir and any custom naming convention to load the report
        produced by launch.
        """
        report_glob = os.path.join(self._output_dir, self._name + search_pattern)
        report_files = [os.path.basename(ff) for ff in glob.glob(report_glob)]
        reports = []
        for report in report_files:
            try:
                power = int(report.split('_')[1])
                reports.append(report)
            except:
                pass
        return reports

    def summary_process(self, parse_output):
        #parse_output.extract_index_from_profile(inplace=True)
        print(parse_output)

        # profile name has been changed to power cap
        df = parse_output.get_report_data(profile=(self._min_power, self._max_power),
                                          agent=self._agent_type,
                                          region='epoch')
        summary = pandas.DataFrame()
        for col in ['count', 'runtime', 'network_time', 'energy_pkg', 'energy_dram', 'frequency']:
            summary[col] = df[col].groupby(level='name').mean()
        summary.index.rename('power cap', inplace=True)
        return summary

    def summary(self, process_output):
        rs = 'Summary for {} with {} agent\n'.format(self._name, self._agent_type)
        rs += process_output.to_string()
        sys.stdout.write(rs + '\n')
