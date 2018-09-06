#!/usr/bin/env python
#
#  Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
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

import os
import sys
import unittest
from collections import defaultdict
from StringIO import StringIO
try:
    import pandas
    import geopm_context
    import geopmpy.analysis
    g_skip_analysis_test = False
    g_skip_analysis_ex = None
except ImportError as ex:
    g_skip_analysis_test = True
    g_skip_analysis_ex = "Warning, analysis and plotting requires the pandas and matplotlib modules to be installed: {}".format(ex)

region_id = {
    'epoch':  '9223372036854775808',
    'dgemm':  '11396693813',
    'stream': '20779751936'
}
version = '0.3.0'
power_budget = 400
tree_decider = 'static'
leaf_decider = 'simple'
agent = 'energy_efficient'
node_name = 'mynode'

# for input data frame
index_names = ['version', 'name', 'power_budget', 'tree_decider',
               'leaf_decider', 'agent', 'node_name', 'iteration', 'region']
numeric_cols = ['count', 'energy_pkg', 'frequency', 'mpi_runtime', 'runtime', 'id']
gen_val = {
    'count': 1,
    'energy_pkg': 14000.0,
    'frequency': 1e9,
    'mpi_runtime': 10,
    'runtime': 50,
    'id': 'bad'
}
ratio_inds = [0, 1, 2, 3, 6, 5, 4]
regions = ['epoch', 'dgemm', 'stream']
iterations = range(1, 4)


# TODO: profile name should affect performance. it can hide bugs if all the numbers are the same
# however the functions that generate expected output need to also take this into account
def make_mock_sweep_report_df(name_prefix, freqs, best_fit_freq, best_fit_perf,
                              metric_of_interest=None, best_fit_metric_perf=None,
                              baseline_freq=None, baseline_metric_perf=None):
    ''' Make a mock report dataframe for the fixed frequency sweeps.'''
    input_data = {}
    for col in numeric_cols:
        input_data[col] = {}
        for freq in freqs:
            prof_name = '{}_freq_{}'.format(name_prefix, freq)
            for it in iterations:
                for region in regions:
                    gen_val['id'] = region_id[region]  # return unique region id
                    index = (version, prof_name, power_budget, tree_decider,
                             leaf_decider, agent, node_name, it, region)
                    value = gen_val[col]
                    # force best performance for requested best fit freq
                    if col == 'runtime':
                        if freq == best_fit_freq[region]:
                            value = best_fit_perf[region]
                        else:
                            # make other frequencies have worse performance
                            value = best_fit_perf[region] * 2.0
                    elif metric_of_interest == col:
                        if freq == best_fit_freq[region]:
                            value = best_fit_metric_perf[region]
                        elif baseline_freq and baseline_metric_perf and freq == baseline_freq:
                            value = baseline_metric_perf[region]
                    input_data[col][index] = value

    df = pandas.DataFrame.from_dict(input_data)
    df.index.rename(index_names, inplace=True)
    return df


def make_mock_report_df(name_prefix, metric, metric_perf):
    ''' Make a mock report dataframe for a single run.'''
    input_data = {}
    for col in numeric_cols:
        input_data[col] = {}
        for it in iterations:
            for region in regions:
                gen_val['id'] = region_id[region]  # return unique region id
                index = (version, name_prefix, power_budget, tree_decider,
                         leaf_decider, agent, node_name, it, region)
                value = gen_val[col]
                if col == metric:
                    value = metric_perf[region]
                input_data[col][index] = value

    df = pandas.DataFrame.from_dict(input_data)
    df.index.rename(index_names, inplace=True)
    return df


def get_expected_baseline_output_df(profile_names, column,
                                    baseline_metric_perf, profile_metric_perf):
    ''' Create dataframe of the savings values expected to be produced by baseline analysis.'''
    expected_cols = [column]

    metric_change = defaultdict(float)
    for prof in profile_names:
        baseline_perf = baseline_metric_perf['epoch']
        optimal_perf = profile_metric_perf[prof]['epoch']
        metric_change[prof] = ((baseline_perf - optimal_perf) / baseline_perf) * 100

    # arrange into list of lists for DataFrame constructor
    expected_data = []
    for prof in profile_names:
        row = [metric_change[prof]]
        expected_data.append(row)
    expected_df = pandas.DataFrame(expected_data, index=profile_names,
                                   columns=expected_cols)
    return expected_df


class TestPowerSweepAnalysis(unittest.TestCase):
    def setUp(self):
        if g_skip_analysis_test:
            self.skipTest(g_skip_analysis_ex)
        self._name_prefix = 'prof'
        self._use_agent = True
        self._min_power = 160
        self._max_power = 200
        self._step_power = 20
        config = {'profile_prefix': self._name_prefix,
                  'output_dir': '.',
                  'verbose': True,
                  'iterations': 1,
                  'min_power': self._min_power, 'max_power': self._max_power,
                  'step_power': self._step_power}
        self._sweep_analysis = geopmpy.analysis.PowerSweepAnalysis(**config)
        self._tmp_files = []

    def tearDown(self):
        for ff in self._tmp_files:
            try:
                os.remove(ff)
            except OSError:
                pass


    def test_something(self):
        x = '''

    def test_online_baseline_comparison_report(self):
        baseline_freq = max(self._freqs)
        best_fit_freq = {'dgemm': self._max_freq, 'stream': self._min_freq, 'epoch': self._mid_freq}

        best_fit_perf = {'dgemm': 23.0, 'stream': 34.0, 'epoch': 45.0}

        # injected energy values
        baseline_metric_perf = {'dgemm': 23456.0, 'stream': 34567.0, 'epoch': 45678.0}
        best_fit_metric_perf = {'dgemm': 23000.0, 'stream': 34000.0, 'epoch': 45000.0}
        optimal_metric_perf = {'dgemm': 22345.0, 'stream': 33456.0, 'epoch': 44567.0}

        sweep_reports = make_mock_sweep_report_df(self._name_prefix, self._freqs,
                                                  best_fit_freq, best_fit_perf,
                                                  'energy_pkg', best_fit_metric_perf,
                                                  baseline_freq, baseline_metric_perf)

        prof_name = self._name_prefix + '_online'
        single_run_report = make_mock_report_df(prof_name, 'energy_pkg', optimal_metric_perf)
        parse_out = sweep_reports.append(single_run_report)
        parse_out.sort_index(ascending=True, inplace=True)

        energy_result = self._online_analysis.summary_process(parse_out)

        expected_energy_df = get_expected_baseline_output_df([prof_name], 'energy_pkg',
                                                             baseline_metric_perf,
                                                             {prof_name: optimal_metric_perf})

        result = energy_result.loc[pandas.IndexSlice['epoch', int(baseline_freq * 1e-6)], 'energy_savings']
        expected = float(expected_energy_df.loc[prof_name])
        self.assertEqual(expected, result)
'''

if __name__ == '__main__':
    unittest.main()
