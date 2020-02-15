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

from __future__ import absolute_import
import sys
import os
import unittest

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_integration import geopm_context
import geopmpy.io
import geopmpy.analysis
from test_integration import util
from test_integration import geopm_test_launcher
#import geopmpy.topo
#import geopmpy.pio


@util.skip_unless_batch()
class TestIntegrationNetworkImbalance(unittest.TestCase):
    def setUp(self):
        self._tmp_files = []

    def tearDown(self):
        pass
        # if os.getenv('GEOPM_KEEP_FILES') is None:
        #     for ff in self._tmp_files:
        #         try:
        #             os.remove(ff)
        #         except OSError:
        #             pass

    # TODO: skip for long tests
    # TODO: new test infra style with single launch, multiple checks
    def test_energy_savings_network_imbalance(self):
        name = 'test_energy_savings_network_imbalance'

        loop_count = 100
        app_conf = geopmpy.io.BenchConf(name + '_app.config')
        self._tmp_files.append(app_conf.get_path())
        app_conf.append_region('dgemm-imbalance', 8.0)
        app_conf.append_region('reduce', 0.02)
        app_conf.set_loop_count(loop_count)
        # Update app config with imbalance
        alloc_nodes = geopm_test_launcher.TestLauncher.get_alloc_nodes()
        for nn in range(len(alloc_nodes) // 2):
            app_conf.append_imbalance(alloc_nodes[nn], 0.5)
        app_conf.write()

        # run at sticker baseline and with agent
        num_node = 4
        num_rank = 16
        if num_node > len(alloc_nodes):
            self.skipTest("Not enough nodes for test; {} required.".format(num_node))
        min_freq = geopm_test_launcher.geopmread("CPUINFO::FREQ_MIN board 0")
        sticker_freq = geopm_test_launcher.geopmread("CPUINFO::FREQ_STICKER board 0")
        run = ['monitor', 'energy_efficient']
        for agent in run:
            report_path = name + '_' + agent + '.report'
            trace_path = name + '_' + agent + '.trace'
            if agent == 'monitor':
                self._options = {}
            else:
                self._options = {'frequency_min': 1.5e9,
                                 'frequency_max': sticker_freq}
            agent_conf = geopmpy.io.AgentConf(name + '_agent.config', agent, self._options)
            self._tmp_files.append(agent_conf.get_path())
            launcher = geopm_test_launcher.TestLauncher(app_conf, agent_conf, report_path,
                                                        trace_path, region_barrier=True, time_limit=900)
            launcher.set_num_node(num_node)
            launcher.set_num_rank(num_rank)
            launcher.run(name + '_' + agent)

        # compare the app_total runtime and energy and assert within bounds
        report_path = name + '_' + run[0] + '.report'
        trace_path = name + '_' + run[0] + '.trace'
        sticker_out = geopmpy.io.AppOutput(report_path, trace_path + '*')
        report_path = name + '_' + run[1] + '.report'
        trace_path = name + '_' + run[1] + '.trace'
        nan_out = geopmpy.io.AppOutput(report_path, trace_path + '*')
        for nn in nan_out.get_node_names():
            sticker_app_total = sticker_out.get_app_total_data(node_name=nn)
            nan_app_total = nan_out.get_app_total_data(node_name=nn)
            runtime_savings_epoch = (sticker_app_total['runtime'].item() - nan_app_total['runtime'].item()) / sticker_app_total['runtime'].item()
            energy_savings_epoch = (sticker_app_total['energy-package'].item() - nan_app_total['energy-package'].item()) / sticker_app_total['energy-package'].item()
            sys.stdout.write('runtime savings: {}\n'.format(runtime_savings_epoch))
            sys.stdout.write('energy savings: {}\n'.format(energy_savings_epoch))

            self.assertLess(-0.1, runtime_savings_epoch)  # want -10% or better
            self.assertLess(0.0, energy_savings_epoch)

if __name__ == '__main__':
    unittest.main()
