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

"""
Tests that the scripts used for SC17 and the newer analysis.py classes give the same results.
"""

import unittest
import subprocess
from collections import defaultdict
from ast import literal_eval
import geopmpy.analysis
import ee_analyzer
import parse_chosen_freq


class TestEnergyEfficiencyAnalysisCheck(unittest.TestCase):
    """
    Tests whether output from EnergyEfficiencyAnalysis matches output
    from the ee_analyzer used for SC17.
    """
    @classmethod
    def setUpClass(cls):
        super(TestEnergyEfficiencyAnalysisCheck, cls).setUpClass()
        cls._name_prefix = 'test_plugin_simple_freq_multi_node'
        cls._output_dir = 'test_integration/sc17_analysis_check/example_reports'  # TODO: broken; must be run from root
        cls._analysis = geopmpy.analysis.EnergyEfficiencyAnalysis(cls._name_prefix,
                                                                  cls._output_dir+'/*',
                                                                  1, 1, None)
        cls._plotter = ee_analyzer.Sc17Plotter(cls._output_dir)

        cls.run_plotter()
        cls.scrape_plotter_adaptive_freqs()
        cls._plotter_results += 'END SC17 PLOTTER'
        cls.run_analysis()

        print 'SC17 plotter'
        print cls._plotter_results
        print
        print 'analysis.py'
        print cls._analysis_results

    def setUp(self):
        pass

    @classmethod
    def run_plotter(cls):
        cls._plotter.parse()
        cls._plotter_results = cls._plotter.print_csv()

    @classmethod
    def scrape_plotter_adaptive_freqs(cls):
        subprocess.call(['./test_integration/sc17_analysis_check/extract_chosen_freq.sh', cls._output_dir])
        cls._plotter_results += parse_chosen_freq.adaptive_freq(cls._output_dir)

    @classmethod
    def run_analysis(cls):
        cls._analysis.find_files()
        parse_output = cls._analysis.parse()
        process_output = cls._analysis.report_process(parse_output)
        cls._analysis_results = cls._analysis.report(process_output)

    def parse_sc17_result_output(self, savings_index, ending_cut, scale_factor=1.0):
        # parse results from sc17 plotter script
        cls = self.__class__
        res1 = cls._plotter_results
        plotter_vals = {'best fit': [], 'offline': [], 'online': []}
        raw_csv = res1.split('savings')[savings_index].split(ending_cut)[0]
        rows = raw_csv.strip().split('\n')
        for row in rows[1:]:
            fields = row.split(',')
            plotter_vals['best fit'].append(float(fields[1])*scale_factor)
            plotter_vals['offline'].append(float(fields[3])*scale_factor)
            plotter_vals['online'].append(float(fields[2])*scale_factor)
        return plotter_vals

    def parse_sc17_frequencies(self):
        cls = self.__class__
        res1 = cls._plotter_results

        raw_csv = res1.split('Adaptive')[1].split('END')[0]
        rows = raw_csv.strip().split('\n')
        freq_vals = defaultdict(list)
        region = None
        for row in rows:
            if ',' in row:
                vals = row.split(',')[1:]
                vals = [float(v) for v in vals]
                freq_vals[region].append((min(vals), max(vals)))
            else:
                region = row
        return freq_vals

    def parse_analysis_result_output(self, first_cut, second_cut, columns):
        # parse results from analysis script
        cls = self.__class__
        res2 = cls._analysis_results
        analysis_vals = defaultdict(list)
        table = res2.split(first_cut)[1]
        if second_cut:
            table = table.split(second_cut)[0]
        rows = table.strip().split('\n')
        for row in rows[1:]:
            row = row.replace(', ', ',')  # for tuples in output
            fields = row.split()
            ind = 1
            for col in columns:
                analysis_vals[col].append(literal_eval(fields[ind]))
                ind += 1
        return analysis_vals

    def test_check_energy_savings(self):
        columns = ['best fit', 'offline', 'online']
        plotter_vals = self.parse_sc17_result_output(1, 'Runtime')
        analysis_vals = self.parse_analysis_result_output('Energy', 'Runtime', columns)

        # do comparison
        mixes = [0, 1, 2, 3, 6, 5, 4]
        failed = False
        for row in range(7):
            for col in columns:
                if abs(plotter_vals[col][row] - analysis_vals[col][row]) > 1e-4:
                    print 'mismatch for {} mix {}'.format(col, mixes[row])
                    print plotter_vals[col][row], analysis_vals[col][row]
                    failed = True
        assert not failed

    def test_runtime_savings(self):
        columns = ['best fit', 'offline', 'online']
        plotter_vals = self.parse_sc17_result_output(2, 'Normalized', -100.0)
        analysis_vals = self.parse_analysis_result_output('Runtime', 'Application', columns)

        # do comparison
        mixes = [0, 1, 2, 3, 6, 5, 4]
        failed = False
        for row in range(7):
            for col in columns:
                if abs(plotter_vals[col][row] - analysis_vals[col][row]) > 1e-4:
                    print 'mismatch for {} mix {}'.format(col, mixes[row])
                    print plotter_vals[col][row], analysis_vals[col][row]
                    failed = True
        assert not failed

    def test_chosen_frequencies(self):
        columns = ['dgemm', 'stream']
        plotter_vals = self.parse_sc17_frequencies()
        analysis_vals = self.parse_analysis_result_output('chosen frequencies', None, columns)

        mixes = [0, 1, 2, 3, 6, 5, 4]
        failed = False
        for row in range(7):
            for col in columns:
                if plotter_vals[col][row] != analysis_vals[col][row]:
                    print 'mismatch for {} mix {}'.format(col, mixes[row])
                    print plotter_vals[col][row], analysis_vals[col][row]
                    failed = True
        assert not failed


if __name__ == '__main__':
    unittest.main()
