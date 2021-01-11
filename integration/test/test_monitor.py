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
        delay = 0.01
        cls._loop_count = 100
        app_conf = geopmpy.io.BenchConf(test_name + '_app.config')
        app_conf.set_loop_count(cls._loop_count)
        app_conf.append_region('spin', delay)

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
        cls._report_output = geopmpy.io.RawReport(cls._report_path)
        cls._trace_output = geopmpy.io.AppOutput(traces=cls._trace_path + '*')

    def tearDown(self):
        pass

    def test_count(self):
        '''Test that region and epoch counts in the report match expected
           number of executions.

        '''
        #region_data = self._report_output.get_df()
        #epoch_data = self._report_output.get_epoch_df()
        #node_names = list(epoch_data['host'].unique())
        report = geopmpy.io.RawReport(report_path)
        region_data = report
        node_names = report.host_names()


        self.assertEqual(len(node_names), self._num_node)

        for nn in node_names:

            trace_data = self._trace_output.get_trace_data(node_name=nn)
            #spin_data = self._output.get_report_data(node_name=nn, region='spin')
            #epoch_data = self._output.get_report_data(node_name=nn, region='epoch')
            spin_data = region_data.loc[(region_data['region'] == 'spin') & (region_data['host'] == nn)]
            epoch_node_data = epoch_data.loc[epoch_data['host'] == nn]
            self.assertEqual(self._loop_count, spin_data['count'].item())
            self.assertEqual(self._loop_count, epoch_node_data['count'].item())
            self.assertEqual(self._loop_count, trace_data['EPOCH_COUNT'][-1])

    def test_runtime(self):
        #util.assertNear(self, delay * loop_count, spin_data['runtime'].item())
        pass
    def test_runtime(self):
        '''Test that region and application total runtimes match expected
           runtime from benchmark config

        '''
        name = 'test_runtime'
        report_path = name + '.report'
        num_node = 1
        num_rank = 5
        delay = 3.0
        app_conf = geopmpy.io.BenchConf(name + '_app.config')
        self._tmp_files.append(app_conf.get_path())
        app_conf.append_region('sleep', delay)
        agent_conf = geopmpy.io.AgentConf(name + '_agent.config', self._agent, self._options)
        self._tmp_files.append(agent_conf.get_path())
        launcher = geopm_test_launcher.TestLauncher(app_conf, agent_conf, report_path)
        launcher.set_num_node(num_node)
        launcher.set_num_rank(num_rank)
        launcher.run(name)
        self._output = geopmpy.io.AppOutput(report_path)
        node_names = self._output.get_node_names()
        self.assertEqual(num_node, len(node_names))


        for nn in node_names:
            report = self._output.get_report_data(node_name=nn, region='sleep')
            app_total = self._output.get_app_total_data(node_name=nn)
            util.assertNear(self, delay, report['runtime'].item())
            self.assertGreater(app_total['runtime'].item(), report['runtime'].item())
    def test_runtime_epoch(self):
        name = 'test_runtime_epoch'
        report_path = name + '.report'
        num_node = 1
        num_rank = 5
        delay = 3.0
        app_conf = geopmpy.io.BenchConf(name + '_app.config')
        self._tmp_files.append(app_conf.get_path())
        app_conf.append_region('sleep', delay)
        app_conf.append_region('spin', delay)
        agent_conf = geopmpy.io.AgentConf(name + '_agent.config', self._agent, self._options)
        self._tmp_files.append(agent_conf.get_path())
        launcher = geopm_test_launcher.TestLauncher(app_conf, agent_conf, report_path)
        launcher.set_num_node(num_node)
        launcher.set_num_rank(num_rank)
        launcher.run(name)
        self._output = geopmpy.io.AppOutput(report_path)
        node_names = self._output.get_node_names()
        self.assertEqual(num_node, len(node_names))
        for nn in node_names:
            spin_data = self._output.get_report_data(node_name=nn, region='spin')
            sleep_data = self._output.get_report_data(node_name=nn, region='sleep')
            epoch_data = self._output.get_report_data(node_name=nn, region='epoch')
            total_runtime = sleep_data['runtime'].item() + spin_data['runtime'].item()
            util.assertNear(self, total_runtime, epoch_data['runtime'].item())

    def test_epoch_data_valid(self):
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
        for nn in node_names:
            regions = report.region_names(nn)
            self.assertTrue('model-init' not in regions)
            totals = report.raw_totals(nn)
            unmarked = report.raw_region(nn, 'unmarked-region')
            epoch = report.raw_epoch(nn)

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
