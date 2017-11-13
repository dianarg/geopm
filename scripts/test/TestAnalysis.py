#!/usr/bin/env python
#
#  Copyright (c) 2015, 2016, 2017, Intel Corporation
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
import unittest
from collections import defaultdict
import random
import pandas
import geopmpy.analysis as ga


# constants; don't care for this analysis
region_id = {
    'epoch': '1234',
    'other': '5678',
    'dgemm': '2468',
    'stream': '1357',
}
version = '0.3.0'
power_budget = 400
tree_decider = 'static'
leaf_decider = 'simple'
node_name = 'mynode'
# for input data frame
index_names = ['version', 'name', 'power_budget', 'tree_decider',
               'leaf_decider', 'node_name', 'iteration', 'region']
numeric_cols = ['count', 'energy', 'frequency', 'mpi_runtime', 'runtime', 'id']
gen_val = {
    'count': lambda: 1,
    'energy': lambda: 14000.0 + 1e3*random.random(),  # TODO: change to random
    'frequency': lambda: 1e9,
    'mpi_runtime': lambda: 10,
    'runtime': lambda: 50 + 10*random.random(),  # TODO: change to random
    'id': lambda: 'bad'
}


def make_mock_sweep_report_df(name_prefix, freqs, best_fit_freq, best_fit_perf,
                              metric_of_interest=None, best_fit_metric_perf=None,
                              baseline_freq=None, baseline_metric_perf=None):
    # dimensions to vary data
    ratio_inds = [0, 1, 2, 3, 6, 5, 4]
    # TODO: regions hardcoded in analysis.py
    regions = ['epoch', 'dgemm', 'stream']
    iterations = range(1, 4)

    # create input data
    input_data = {}
    for col in numeric_cols:
        input_data[col] = {}
        for ratio_idx in ratio_inds:
            for freq in freqs:
                prof_name = '{}_{}_{}'.format(name_prefix, freq, ratio_idx)
                for it in iterations:
                    for region in regions:
                        gen_val['id'] = lambda: region_id[region]  # return unique region id
                        index = (version, prof_name, power_budget, tree_decider,
                                 leaf_decider, node_name, it, region)
                        value = gen_val[col]()
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


def make_mock_optimal_report_df(name_prefix, metric, metric_perf):
    # dimensions to vary data
    ratio_inds = [0, 1, 2, 3, 6, 5, 4]
    # TODO: hardcoded in analysis.py
    regions = ['epoch', 'dgemm', 'stream']
    iterations = range(1, 4)

    # create input data
    input_data = {}
    for col in numeric_cols:
        input_data[col] = {}
        for ratio_idx in ratio_inds:
            prof_names = {
                'offline per-phase': '{}_optimal_{}'.format(name_prefix, ratio_idx),
                'online per-phase': '{}_adaptive_{}'.format(name_prefix, ratio_idx),
            }
            for short_name, prof_name in prof_names.items():
                for it in iterations:
                    for region in regions:
                        gen_val['id'] = lambda: region_id[region]  # return unique region id
                        index = (version, prof_name, power_budget, tree_decider,
                                 leaf_decider, node_name, it, region)
                        value = gen_val[col]()
                        if col == metric:
                            value = metric_perf[short_name][region]
                        input_data[col][index] = value

    df = pandas.DataFrame.from_dict(input_data)
    df.index.rename(index_names, inplace=True)
    return df


# TODO: this should not need to know about the input data frame at all;
# just need to give it the profile names
def get_expected_output_df(input_data, column, baseline_freq, best_fit_freq,
                           baseline_metric_perf, optimal_metric_perf):
    # create expected output
    # for output data frame
    ratio_inds = [0, 1, 2, 3, 6, 5, 4]  # TODO: repeated above
    expected_index = ratio_inds
    profiles = ['offline application', 'offline per-phase', 'online per-phase']
    expected_cols = profiles

    energy_change = defaultdict(dict)
    for index, dp in input_data.iterrows():
        _, prof, _, _, _, _, it, reg = index
        mix_idx = int(prof[-1])
        if str(baseline_freq) in prof:
            profile = 'baseline'
            # no need for energy change
            continue
        elif str(best_fit_freq) in prof:
            profile = 'offline application'
        elif 'optimal' in prof:
            profile = 'offline per-phase'
        elif 'adaptive' in prof:
            profile = 'online per-phase'
        else:
            # ignore other sweep reports
            continue
        energy_change[mix_idx][profile] = (baseline_metric_perf['epoch'] - optimal_metric_perf[profile]['epoch']) / baseline_metric_perf['epoch']

    # arrange into list of lists for DataFrame constructor
    expected_data = []
    for idx in ratio_inds:  # TODO: remove reordering of mix indices
        row = []
        for prof in profiles:
            if prof != 'baseline':
                change = energy_change[idx][prof]
                row.append(change)
        expected_data.append(row)
    expected_df = pandas.DataFrame(expected_data, index=expected_index,
                                   columns=expected_cols)
    return expected_df


class TestAnalysis(unittest.TestCase):
    def setUp(self):
        pass

    def test_nothing(self):
        analysis = ga.MockAnalysis('name', 2, 3, 'args')
        analysis.launch()
        parse_out = analysis.parse()
        out = analysis.report_process(parse_out)
        analysis.report(out)
        out = analysis.plot_process(parse_out)
        analysis.plot(out)

    def test_calculate_optimal_freq(self):
        name_prefix = 'prof'
        analysis = ga.Sc17Analysis('name', 2, 3, 'args')
        best_fit_freq = {'dgemm': 2.0e9, 'stream': 1.2e9, 'epoch': 1.5e9}
        best_fit_perf = {'dgemm': 23.0, 'stream': 34.0, 'epoch': 45.0}

        # TODO: min and max hardcoded in analysis.py
        freqs = [1.2e9, 1.3e9, 1.4e9, 1.5e9, 1.6e9, 1.7e9, 1.8e9, 1.9e9, 2.0e9, 2.1e9]

        parse_out = make_mock_sweep_report_df(name_prefix, freqs, best_fit_freq, best_fit_perf)

        result = analysis.calculate_optimal_freq(parse_out, name_prefix, 1)
        for region in ['epoch', 'dgemm', 'stream']:
            self.assertEqual(best_fit_freq[region], result[region])

    def test_baseline_comparison_report(self):
        analysis = ga.Sc17Analysis('name', 2, 3, 'args')

        baseline_freq = 2.1e9
        best_fit_freq = {'dgemm': 2.0e9, 'stream': 1.2e9, 'epoch': 1.5e9}
        best_fit_perf = {'dgemm': 23.0, 'stream': 34.0, 'epoch': 45.0}

        # injected energy values
        baseline_metric_perf = {'dgemm': 23456.0, 'stream': 34567.0, 'epoch': 45678.0}
        best_fit_metric_perf = {'dgemm': 23000.0, 'stream': 34000.0, 'epoch': 45000.0}
        optimal_metric_perf = {
            'offline application': best_fit_metric_perf,
            'offline per-phase': {'dgemm': 12345.0, 'stream': 23456.0, 'epoch': 34567.0},
            'online per-phase': {'dgemm': 22345.0, 'stream': 33456.0, 'epoch': 44567.0},
        }
        # TODO: min and max hardcoded in analysis.py
        freqs = [1.2e9, 1.3e9, 1.4e9, 1.5e9, 1.6e9, 1.7e9, 1.8e9, 1.9e9, 2.0e9, 2.1e9]

        name_prefix = 'test_plugin_simple_freq_multi_node'  # TODO: hardcoded in analysis.py
        sweep_reports = make_mock_sweep_report_df(name_prefix, freqs, best_fit_freq, best_fit_perf,
                                                  'energy', best_fit_metric_perf,
                                                  baseline_freq, baseline_metric_perf)

        optimal_reports = make_mock_optimal_report_df(name_prefix, 'energy', optimal_metric_perf)
        parse_out = sweep_reports.append(optimal_reports)

        expected_energy_df = get_expected_output_df(parse_out, 'energy',
                                                    2.1e9, 1.5e9,  # baseline freq, epoch best fit
                                                    baseline_metric_perf, optimal_metric_perf)

        energy_result, runtime_result = analysis.report_process(parse_out)

        expected_columns = expected_energy_df.columns
        expected_index = expected_energy_df.index
        result_columns = energy_result.columns
        result_index = energy_result.index
        self.assertEqual(len(expected_columns), len(result_columns))
        self.assertEqual(len(expected_index), len(result_index))
        self.assertTrue((expected_columns == result_columns).all())
        self.assertTrue((expected_index == result_index).all())
        print (expected_energy_df == energy_result)
        print 'expected'
        print expected_energy_df
        print 'result'
        print energy_result
        self.assertTrue((expected_energy_df == energy_result).all().all())


if __name__ == '__main__':
    random.seed(0)
    unittest.main()
