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

"""
POWER_BALANCER

Tests that the power balancer agent shifts power between nodes and sockets
to reduce runtime under a power cap.

"""

import sys
import unittest
import os
import glob
import pandas
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from integration.test import geopm_context
import geopmpy.io
import geopmpy.error
from integration.experiment import power_sweep, balancer_comparison
from integration.apps import apps

from integration.test import util
if util.do_launch():
    # Note: this import may be moved outside of do_launch if needed to run
    # commands on compute nodes such as geopm_test_launcher.geopmread
    from integration.test import geopm_test_launcher


@util.skip_unless_run_long_tests()
@util.skip_unless_batch()
class TestIntegration_power_balancer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Create launcher, execute benchmark and set up class variables.

        """
        cls._show_details = True

        cls._test_name = 'test_power_balancer'
        cls._num_node = 4
        cls._agent_list = ['power_governor', 'power_balancer']
        cls._do_launch = util.do_launch()
        cls._tmp_files = []

        if cls._do_launch:
            loop_count = 500
            fam, mod = geopm_test_launcher.get_platform()
            alloc_nodes = geopm_test_launcher.TestLauncher.get_alloc_nodes()
            num_node = len(alloc_nodes)
            if (len(alloc_nodes) != cls._num_node):
               err_fmt = 'Warning: <test_power_balancer> Allocation size ({}) is different than expected ({})\n'
               sys.stderr.write(err_fmt.format(num_node, cls._num_node))
               cls._num_node = num_node
            num_rank = 2 * cls._num_node
            power_budget = 180
            if fam == 6 and mod == 87:
                # budget for KNL
                power_budget = 130

            for app_name in ['geopmbench', 'socket_imbalance', 'geopmbench-balanced']:
                app_conf = None
                if app_name == 'geopmbench':
                    app_conf = geopmpy.io.BenchConf(cls._test_name + '_app.config')
                    cls._tmp_files.append(app_conf.get_path())
                    app_conf.append_region('dgemm-imbalance', 8.0)
                    app_conf.set_loop_count(loop_count)
                    # Update app config with imbalance
                    for nn in range(len(alloc_nodes) // 2):
                        app_conf.append_imbalance(alloc_nodes[nn], 0.5)
                elif app_name == 'socket_imbalance':
                    script_dir = os.path.dirname(os.path.realpath(__file__))
                    exec_path = os.path.join(script_dir, '.libs', 'test_power_balancer')
                    app_conf = apps.AppConf(exec_path)
                elif app_name == 'geopmbench-balanced':
                    app_conf = geopmpy.io.BenchConf(cls._test_name + '-balanced_app.config')
                    cls._tmp_files.append(app_conf.get_path())
                    app_conf.append_region('dgemm', 8.0)
                    app_conf.set_loop_count(loop_count)
                else:
                    raise RuntimeError('No application config for app name {}'.format(app_name))

                # TODO: time_limit for launcher = 2700
                # TODO: need logging about power cap
                #launcher.write_log(run_name, 'Power cap = {}W'.format(power_budget))
                power_sweep.launch_power_sweep(file_prefix=cls._test_name + '_' + app_name,
                                               machine_config=cls._test_name + '.machine',
                                               output_dir='.',
                                               iterations=1,
                                               min_power=power_budget,
                                               max_power=power_budget,
                                               step_power=1,
                                               agent_types=cls._agent_list,
                                               num_node=cls._num_node,
                                               num_rank=num_rank,
                                               app_conf=app_conf)

    def get_power_data(self, app_name, agent, report_path, trace_path):
        output = geopmpy.io.AppOutput(report_path, trace_path + '*')
        node_names = output.get_node_names()
        self.assertEqual(self._num_node, len(node_names))

        new_output = geopmpy.io.RawReport(report_path)
        power_budget = new_output.meta_data()['Policy']['POWER_PACKAGE_LIMIT_TOTAL']

        power_limits = []
        # Total power consumed will be Socket(s) + DRAM
        for nn in node_names:
            tt = output.get_trace_data(node_name=nn)

            first_epoch_index = tt.loc[tt['EPOCH_COUNT'] == 0][:1].index[0]
            epoch_dropped_data = tt[first_epoch_index:]  # Drop all startup data

            power_data = epoch_dropped_data[['TIME', 'ENERGY_PACKAGE', 'ENERGY_DRAM']]
            power_data = power_data.diff().dropna()
            power_data.rename(columns={'TIME': 'ELAPSED_TIME'}, inplace=True)
            power_data = power_data.loc[(power_data != 0).all(axis=1)]  # Will drop any row that is all 0's

            pkg_energy_cols = [s for s in power_data.keys() if 'ENERGY_PACKAGE' in s]
            dram_energy_cols = [s for s in power_data.keys() if 'ENERGY_DRAM' in s]
            power_data['SOCKET_POWER'] = power_data[pkg_energy_cols].sum(axis=1) / power_data['ELAPSED_TIME']
            power_data['DRAM_POWER'] = power_data[dram_energy_cols].sum(axis=1) / power_data['ELAPSED_TIME']
            power_data['COMBINED_POWER'] = power_data['SOCKET_POWER'] + power_data['DRAM_POWER']

            pandas.set_option('display.width', 100)
            power_stats = '\nPower stats from {} {} :\n{}\n'.format(agent, nn, power_data.describe())
            if not self._skip_launch:
                run_name = '{}_{}_{}'.format(self._test_name, agent, app_name)
                with open(run_name + '.log', 'a') as outfile:
                    outfile.write(power_stats)
            if self._show_details:
                sys.stdout.write(power_stats)

            # Get final power limit set on the node
            if agent == 'power_balancer':
                power_limits.append(epoch_dropped_data['ENFORCED_POWER_LIMIT'][-1])

        # Check average limit for job is not exceeded
        if agent == 'power_balancer':
            avg_power_limit = sum(power_limits) / len(power_limits)
            self.assertLessEqual(avg_power_limit, power_budget)

        node_names = output.get_node_names()
        runtime_list = []
        for node_name in node_names:
            epoch_data = output.get_report_data(node_name=node_name, region='dgemm')
            runtime_list.append(epoch_data['runtime'].values.item())
        return runtime_list

    def balancer_test_helper(self, app_name):
        # Require that the balancer moves the maximum dgemm runtime at
        # least 1/4 the distance to the mean dgemm runtime under the
        # governor.
        output = geopmpy.io.RawReportCollection(self._test_name + '_' + app_name + '_*report')

        # check average power does not exceed policy
        # TODO: why did this need to use the trace before?  A: power_data.describe() for verbose output
        # also checks that assigned power limits across the 4 nodes did not exceed the budget
        # this check using the report only cannot confirm that.  would be simple with issue #1211
        app_df = output.get_app_df()
        policy_power = app_df['POWER_PACKAGE_LIMIT_TOTAL'].unique().item()
        gov_avg_power = app_df.loc[app_df['Agent'] == 'power_governor']['power (watts)'].mean()
        self.assertLessEqual(gov_avg_power, policy_power, "power_governor average power exceeded policy: {}".format(gov_avg_power))

        bal_avg_power = app_df.loc[app_df['Agent'] == 'power_balancer']['power (watts)'].mean()
        self.assertLessEqual(bal_avg_power, policy_power, "power_governor average power exceeded policy: {}".format(bal_avg_power))

        # compare runtime with balancer vs. governor
        app_result = balancer_comparison.balancer_comparison(app_df)

        if self._show_details:
            sys.stdout.write("\nAverage runtime stats:\n")
            sys.stdout.write("{}\n".format(app_result))
            sys.stdout.write("\nAverage power:\n")
            sys.stdout.write("power_governor: {}\n".format(gov_avg_power))
            sys.stdout.write("power_balancer: {}\n".format(bal_avg_power))
            # sys.stdout.write("governor runtime: {}, balancer runtime: {}, margin: {}, runtime_pct: {}\n".format(
            #     result['max_runtime_gov'], result['max_runtime_bal'], result['margin'], result['runtime_pct']))

        self.assertGreater(app_result['max_runtime_gov'].item() - app_result['margin'].item(),
                           app_result['max_runtime_bal'].item(),
                           "governor runtime: {}, balancer runtime: {}, margin: {}, runtime_pct: {}\n".format(
                               app_result['max_runtime_gov'], app_result['max_runtime_bal'], app_result['margin'], app_result['runtime_pct']))

        # TODO: previous version of test used dgemm region only
        # comparison based on epoch is more correct
        epoch_data = output.get_epoch_df()
        print(epoch_data)
        epoch_result = balancer_comparison.balancer_comparison(epoch_data)

        self.assertGreater(epoch_result['max_runtime_gov'].item() - epoch_result['margin'].item(),
                           epoch_result['max_runtime_bal'].item(),
                           "governor runtime: {}, balancer runtime: {}, margin: {}, runtime_pct: {}\n".format(
                               epoch_result['max_runtime_gov'], epoch_result['max_runtime_bal'], epoch_result['margin'], epoch_result['runtime_pct']))

    def test_power_balancer_geopmbench(self):
        self.balancer_test_helper('geopmbench')

    def test_power_balancer_geopmbench_balance(self):
        self.balancer_test_helper('geopmbench-balanced')

    def test_power_balancer_socket_imbalance(self):
        self.balancer_test_helper('socket_imbalance')


if __name__ == '__main__':
    unittest.main()
