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
import unittest
import pandas

from experiment import report


class TestExperimentReport(unittest.TestCase):
    def setUp(self):
        profiles = ['bal_0', 'bal_0', 'bal_1', 'bal_1',
                    'gov_0', 'gov_0', 'gov_1', 'gov_1', ]
        hosts = ['mcfly1', 'mcfly2', 'mcfly1', 'mcfly2',
                 'mcfly1', 'mcfly2', 'mcfly1', 'mcfly2', ]
        self.runtimes = [6, 7, 8, 9,
                         5.4, 5.5, 3.3, 2.3]
        network_times = [1, 2, 3, 4,
                         1, 2, 3, 4]
        self.energies = [88, 99, 44, 55,
                         444, 555, 666, 777]
        self.powers = [200, 201, 202, 203,
                       190, 191, 192, 193]
        self.report_data = {'Profile': profiles,
                            'runtime (sec)': self.runtimes,
                            'network-time (sec)': network_times,
                            'package-energy (joules)': self.energies,
                            'power (watts)': self.powers,
                            'host': hosts,
                            }

    def test_prepare_columns(self):
        df = pandas.DataFrame(self.report_data)
        report.prepare_columns(df)
        self.assertTrue('trial' in df.columns)

    def test_df_with_fom(self):
        foms = [45, 56, 67, 78,
                33.4, 55.5, 66.6, 77.5]
        self.report_data['FOM'] = foms
        df = pandas.DataFrame(self.report_data)
        report.prepare_columns(df)

        # use runtime as performance metric
        ddf = df.copy()
        report.prepare_metrics(ddf, 'runtime')
        self.assertEqual(self.runtimes, ddf['runtime'].tolist())
        self.assertEqual(self.energies, ddf['energy_perf'].tolist())
        # use FOM as performance metric
        ddf = df.copy()
        report.prepare_metrics(ddf, 'FOM')
        self.assertEqual(foms, ddf['FOM'].tolist())
        self.assertEqual((ddf['FOM'] / ddf['power']).tolist(), ddf['energy_perf'].tolist())

        result = report.energy_perf_summary(df, 'Profile', ['gov', 'bal'], 'gov', 'FOM', False)
        sys.stdout.write('{}\n\n'.format(result))
        result = report.energy_perf_summary(df, 'Profile', ['gov', 'bal'], None, 'FOM', True)
        sys.stdout.write('{}\n\n'.format(result))

    def test_df_no_fom(self):
        df = pandas.DataFrame(self.report_data)
        report.prepare_columns(df)

        # use inverse runtime as performance metric
        ddf = df.copy()
        report.prepare_metrics(ddf, 'FOM')
        self.assertEqual((1.0 / ddf['runtime']).tolist(), ddf['FOM'].tolist())
        self.assertEqual((ddf['FOM'] / ddf['power']).tolist(), ddf['energy_perf'].tolist())

        # for metric in ['runtime', 'FOM']:
        #     sys.stdout.write('{}:\n{}\n'.format(metric, df))
        #     result = report.energy_perf_summary(df, 'Profile', ['gov', 'bal'], 'gov', metric, False)
        #     sys.stdout.write('{}\n\n'.format(result))
        #     result = report.energy_perf_summary(df, 'Profile', ['gov', 'bal'], None, metric, True)
        #     sys.stdout.write('{}\n\n'.format(result))

    def test_df_bad_fom(self):
        bad_foms = [1, 1, float('nan'), 1, 1, 1, 1, 1]
        self.report_data['FOM'] = bad_foms
        df = pandas.DataFrame(self.report_data)
        report.prepare_columns(df)
        # use inverse runtime as performance metric
        ddf = df.copy()
        report.prepare_metrics(ddf, 'FOM')
        self.assertEqual((1.0 / ddf['runtime']).tolist(), ddf['FOM'].tolist())
        self.assertEqual((ddf['FOM'] / ddf['power']).tolist(), ddf['energy_perf'].tolist())

        # for metric in ['runtime', 'FOM']:
        #     sys.stdout.write('{}:\n{}\n'.format(metric, df))
        #     result = report.energy_perf_summary(df, 'Profile', ['gov', 'bal'], 'gov', metric, False)
        #     sys.stdout.write('{}\n\n'.format(result))
        #     result = report.energy_perf_summary(df, 'Profile', ['gov', 'bal'], None, metric, True)
        #     sys.stdout.write('{}\n\n'.format(result))


if __name__ == '__main__':
    unittest.main()
