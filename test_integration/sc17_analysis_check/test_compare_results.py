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

'''
Tests that the scripts used for SC17 and the newer analysis.py classes give the same results.
'''

import unittest
import geopmpy.analysis
import ee_analyzer


class TestEnergyEfficiencyAnalysisCheck(unittest.TestCase):
    ''' Tests whether output from EnergyEfficiencyAnalysis matches output
        from the ee_analyzer used for SC17.'''
    @classmethod
    def setUpClass(cls):
        super(TestEnergyEfficiencyAnalysisCheck, cls).setUpClass()
        cls.m_name_prefix = 'test_plugin_simple_freq_multi_node'
        cls.m_output_dir = 'test_integration/sc17_analysis_check/example_reports'  # TODO: broken; must be run from root
        cls.m_analysis = geopmpy.analysis.EnergyEfficiencyAnalysis(cls.m_name_prefix,
                                                                   cls.m_output_dir,
                                                                   1, 1,
                                                                   None)
        cls.m_plotter = ee_analyzer.Sc17Plotter(cls.m_output_dir)

        cls.run_plotter()
        cls.run_analysis()

    def setUp(self):
        pass

    @classmethod
    def run_plotter(cls):
        cls.m_plotter.parse()
        cls.m_plotter_results = cls.m_plotter.print_csv()

    @classmethod
    def run_analysis(cls):
        parse_output = cls.m_analysis.parse()
        process_output = cls.m_analysis.report_process(parse_output)
        cls.m_analysis_results = cls.m_analysis.report(process_output)

    def parse_sc17_result_output(self, savings_index, ending_cut, scale_factor=1.0):
        # parse results from sc17 plotter script
        cls = self.__class__
        res1 = cls.m_plotter_results
        plotter_vals = {'best fit': [], 'offline': [], 'online': []}
        raw_csv = res1.split('savings')[savings_index].split(ending_cut)[0]
        rows = raw_csv.strip().split('\n')
        for row in rows[1:]:
            fields = row.split(',')
            plotter_vals['best fit'].append(float(fields[1])*scale_factor)
            plotter_vals['offline'].append(float(fields[3])*scale_factor)
            plotter_vals['online'].append(float(fields[2])*scale_factor)
        return plotter_vals

    def parse_analysis_result_output(self, first_cut, second_cut):
        cls = self.__class__
        # parse results from analysis script
        res2 = cls.m_analysis_results
        analysis_vals = {'best fit': [], 'offline': [], 'online': []}
        table = res2.split(first_cut)[1]
        if second_cut:
            table = table.split(second_cut)[0]
        rows = table.strip().split('\n')
        for row in rows[1:]:
            fields = row.split()
            analysis_vals['best fit'].append(float(fields[1]))
            analysis_vals['offline'].append(float(fields[2]))
            analysis_vals['online'].append(float(fields[3]))
        return analysis_vals

    def test_check_energy_savings(self):
        cls = self.__class__
        print 'SC17 plotter'
        print cls.m_plotter_results
        print
        print 'analysis.py'
        print cls.m_analysis_results

        plotter_vals = self.parse_sc17_result_output(1, 'Runtime')
        analysis_vals = self.parse_analysis_result_output('Energy', 'Runtime')

        # do comparison
        mixes = [0, 1, 2, 3, 6, 5, 4]
        failed = False
        for row in range(7):
            for col in ['best fit', 'offline', 'online']:
                if abs(plotter_vals[col][row] - analysis_vals[col][row]) > 1e-4:
                    print 'mismatch for {} mix {}'.format(col, mixes[row])
                    print plotter_vals[col][row], analysis_vals[col][row]
                    failed = True
        assert not failed

    def test_runtime_savings(self):
        cls = self.__class__
        cls = self.__class__
        print 'SC17 plotter'
        print cls.m_plotter_results
        print
        print 'analysis.py'
        print cls.m_analysis_results

        plotter_vals = self.parse_sc17_result_output(2, 'Normalized', -100.0)
        analysis_vals = self.parse_analysis_result_output('Runtime', None)

        # do comparison
        mixes = [0, 1, 2, 3, 6, 5, 4]
        failed = False
        for row in range(7):
            for col in ['best fit', 'offline', 'online']:
                if abs(plotter_vals[col][row] - analysis_vals[col][row]) > 1e-4:
                    print 'mismatch for {} mix {}'.format(col, mixes[row])
                    print plotter_vals[col][row], analysis_vals[col][row]
                    failed = True
        assert not failed

    def test_chosen_frequencies(self):
        cls = self.__class__
        res1 = cls.m_plotter_results
        res2 = cls.m_analysis_results


if __name__ == '__main__':
    unittest.main()
