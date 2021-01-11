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

"""Test that basic information expected when running the synthetic
benchmark is reflected in the reports.

"""

import sys
import unittest
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from integration.test import geopm_context
import geopmpy.io
import geopmpy.error
import util
import geopm_test_launcher


class TestIntegration_monitor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Create launcher, execute benchmark and set up class variables.

        """
        sys.stdout.write('(' + os.path.basename(__file__).split('.')[0] +
                         '.' + cls.__name__ + ') ...')
        test_name = 'test_monitor'
        cls._report_path = '{}.report'.format(test_name)
        cls._trace_path = '{}.trace'.format(test_name)
        cls._image_path = '{}.png'.format(test_name)
        cls._agent_conf_path = 'test_' + test_name + '-agent-config.json'
        # Clear out exception record for python 2 support
        geopmpy.error.exc_clear()
        # Set the job size parameters
        cls._num_node = 2
        num_rank = 4
        time_limit = 6000
        # Configure the test application
        #delay = 0.01
        cls._spin_bigo = 0.5
        cls._sleep_bigo = 1.0
        cls._loop_count = 20
        app_conf = geopmpy.io.BenchConf(test_name + '_app.config')
        app_conf.set_loop_count(cls._loop_count)
        app_conf.append_region('spin', cls._spin_bigo)
        app_conf.append_region('sleep', cls._sleep_bigo)

        # Configure the monitor agent
        agent_conf = geopmpy.io.AgentConf(test_name + '_agent.config')

        # Create the test launcher with the above configuration
        launcher = geopm_test_launcher.TestLauncher(app_conf,
                                                    agent_conf,
                                                    cls._report_path,
                                                    cls._trace_path,
                                                    time_limit=time_limit)
        launcher.set_num_node(cls._num_node)
        launcher.set_num_rank(num_rank)
        # Run the test application
        launcher.run('test_' + test_name)

        # Output to be reused by all tests
        #cls._report_output = geopmpy.io.RawReport(cls._report_path)
        cls._trace_output = geopmpy.io.AppOutput(traces=cls._trace_path + '*')
        cls._report = geopmpy.io.RawReport(cls._report_path)
        cls._node_names = cls._report.host_names()

    def tearDown(self):
        pass

    def test_meta_data(self):
        self.assertEqual(len(self._node_names), self._num_node)

    def test_count(self):
        '''Test that region and epoch counts in the report match expected
           number of executions.

        '''
        for node in self._node_names:
            trace_data = self._trace_output.get_trace_data(node_name=node)
            spin_data = self._report.raw_region(node, 'spin')
            sleep_data = self._report.raw_region(node, 'sleep')
            epoch_data = self._report.raw_epoch(node)
            self.assertEqual(self._loop_count, spin_data['count'])
            self.assertEqual(self._loop_count, sleep_data['count'])
            self.assertEqual(self._loop_count, epoch_data['count'])
            self.assertEqual(self._loop_count, trace_data['EPOCH_COUNT'][-1])

    def test_runtime(self):
        '''Test that region and application total runtimes match expected
           runtime from benchmark config

        '''
        for node in self._node_names:
            spin_data = self._report.raw_region(node, 'spin')
            sleep_data = self._report.raw_region(node, 'sleep')
            app_total = self._report.raw_totals(node)
            util.assertNear(self, self._loop_count * self._spin_bigo, spin_data['runtime (s)'])
            util.assertNear(self, self._loop_count * self._sleep_bigo, sleep_data['runtime (s)'])
            total_runtime = sleep_data['runtime (s)'] + spin_data['runtime (s)']
            #self.assertGreater(app_total['runtime (s)'], total_runtime)
            util.assertNear(self, total_runtime, app_total['runtime (s)'])

    def test_runtime_epoch(self):
        for node in self._node_names:
            spin_data = self._report.raw_region(node, 'spin')
            sleep_data = self._report.raw_region(node, 'sleep')
            epoch_data = self._report.raw_epoch(node)
            total_runtime = sleep_data['runtime (s)'] + spin_data['runtime (s)']
            util.assertNear(self, total_runtime, epoch_data['runtime (s)'])

    def test_epoch_data_valid(self):
        return
        name = 'test_epoch_data_valid'
        report_path = name + '.report'
        num_node = 1
        num_rank = 1
        big_o = 1.0
        loop_count = 10
        app_conf = geopmpy.io.BenchConf(name + '_app.config')
        self._tmp_files.append(app_conf.get_path())
        app_conf.set_loop_count(loop_count)
        app_conf.append_region('spin-unmarked', big_o)
        agent_conf = geopmpy.io.AgentConf(name + '_agent.config', self._agent, self._options)
        self._tmp_files.append(agent_conf.get_path())
        launcher = geopm_test_launcher.TestLauncher(app_conf, agent_conf, report_path)
        launcher.set_num_node(num_node)
        launcher.set_num_rank(num_rank)
        launcher.run(name)

        report = geopmpy.io.RawReport(report_path)
        node_names = report.host_names()
        self.assertEqual(num_node, len(node_names))
        for node in node_names:
            regions = report.region_names(node)
            self.assertTrue('model-init' not in regions)
            totals = report.raw_totals(node)
            unmarked = report.raw_region(node, 'unmarked-region')
            epoch = report.raw_epoch(node)

            # Epoch has valid data
            self.assertGreater(epoch['runtime (sec)'], 0)
            self.assertGreater(epoch['sync-runtime (sec)'], 0)
            self.assertGreater(epoch['package-energy (joules)'], 0)
            self.assertGreater(epoch['dram-energy (joules)'], 0)
            self.assertGreater(epoch['power (watts)'], 0)
            self.assertGreater(epoch['frequency (%)'], 0)
            self.assertGreater(epoch['frequency (Hz)'], 0)
            self.assertEqual(epoch['count'], loop_count)

            for signal in ['runtime (sec)', 'package-energy (joules)', 'dram-energy (joules)']:
                util.assertNear(self, totals[signal], unmarked[signal], msg='signal={}'.format(signal))
                util.assertNear(self, totals[signal], epoch[signal], msg='signal={}'.format(signal))
                util.assertNear(self, totals[signal], epoch[signal], msg='signal={}'.format(signal))

            util.assertNear(self, unmarked['runtime (sec)'], unmarked['sync-runtime (sec)'])
            util.assertNear(self, epoch['runtime (sec)'], epoch['sync-runtime (sec)'])


if __name__ == '__main__':
    unittest.main()
