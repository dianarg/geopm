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

# import geopmpy.plotter
import geopmpy.io
import code
import sys
import pandas
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy
import os
import code
import cPickle as pickle


class Sc17Plotter:
    def __init__(self, output_path):
        self.m_output_path = output_path
        self.m_energy_by_percent = {}
        self.m_runtime_by_percent = {}
        self.m_norm_runtime_by_percent = {}
        self.m_best_fit_freq_by_percent = {}
        self.m_big_o_by_percent = {}
        self.m_ratio_idx_by_percent = {}
        self.use_cached_data_frames = False
        self.m_result_df = {}
        #self.m_result_df = pandas.DataFrame()
        self.m_result_df['best_fit_energy'] = pandas.Series(8)
        self.m_result_df['dynamic_energy'] = pandas.Series(8)
        self.m_result_df['adaptive_energy'] = pandas.Series(8)
        #self.m_result_df['best_fit_energy'] = pandas.DataFrame()
        #self.m_result_df['dynamic_energy'] = pandas.DataFrame()
        #self.m_result_df['adaptive_energy'] = pandas.DataFrame()

        self.m_result_df['best_fit_runtime'] = pandas.Series(8)
        self.m_result_df['dynamic_runtime'] = pandas.Series(8)
        self.m_result_df['adaptive_runtime'] = pandas.Series(8)

    def parse(self):
        print '=' * 60
        ee_mix_file_path = self.m_output_path

        name = 'test_plugin_simple_freq_multi_node'
        # for SC17 numbers, used max=2.2 and sticker = 2.1, baseline=sticker
        is_baseline_sticker = True
        step_freq = 100e6
        min_freq = 1.2e9
        max_freq = 2.2e9
        sticker_freq = 2.1e9
        default_freq = sticker_freq - 4 * step_freq
        if default_freq < min_freq:
            default_freq = min_freq

        num_step = 1 + int((max_freq - min_freq) / step_freq)
        freq_sweep = [step_freq * ss + min_freq for ss in range(num_step)]
        freq_sweep.reverse()

        perf_margin = 0.1
        idx = pandas.IndexSlice

        sweep_runtime_by_mix_by_freq = {}
        sweep_energy_by_mix_by_freq = {}
        target_runtime_by_mix = {}

        mix_ratios = [(1.0, 0.25), (1.0, 0.5), (1.0, 0.75), (1.0, 1.0),
                      (0.25, 1.0), (0.5, 1.0), (0.75, 1.0)]
        for (ratio_idx, ratio) in enumerate(mix_ratios):
            optimal_freq = dict()
            min_runtime = dict()
            is_once = True

            sweep_runtime_by_mix_by_freq[ratio_idx] = {}
            sweep_energy_by_mix_by_freq[ratio_idx] = {}
            target_runtime_by_mix[ratio_idx] = {}
            # parse freq sweeps to calculate optimal freq
            # individual runs decide based on min runtime, and do not use this epoch freq
            # so this data is valid for the graph comparison
            for freq in freq_sweep:
                sweep_runtime_by_mix_by_freq[ratio_idx][freq] = {}
                sweep_energy_by_mix_by_freq[ratio_idx][freq] = {}

                pkl_file = ee_mix_file_path + '/{}_freq_{}_mix_{}.pkl'.format(name, freq, ratio_idx)
                if self.use_cached_data_frames:
                    with open(pkl_file, 'rb') as f:
                        region_df = pickle.load(f)
                else:
                    report_path = '{}_freq_{}_mix_{}.report'.format(name, freq, ratio_idx)
                    report_path = ee_mix_file_path + '/*/' + report_path
                    region_df = geopmpy.io.AppOutput(report_path, verbose=False).get_report_df()
                    with open(pkl_file, 'wb') as f:
                        pickle.dump(region_df, f)

                region_mean_runtime = region_df.groupby(level='region')['runtime'].mean()
                region_mean_energy = region_df.groupby(level='region')['energy'].mean()

                for region in ['dgemm', 'stream', 'epoch']:
                    sweep_runtime_by_mix_by_freq[ratio_idx][freq][region] = region_mean_runtime[region]
                    sweep_energy_by_mix_by_freq[ratio_idx][freq][region] = region_mean_energy[region]
                    if is_once:
                        min_runtime[region] = region_mean_runtime[region]
                        optimal_freq[region] = freq
                    elif min_runtime[region] > region_mean_runtime[region]:
                        min_runtime[region] = region_mean_runtime[region]
                        optimal_freq[region] = freq
                    elif min_runtime[region] * (1 + perf_margin) > region_mean_runtime[region]:
                        optimal_freq[region] = freq
                is_once = False
                for region in ['dgemm', 'stream', 'epoch']:
                    target_runtime_by_mix[ratio_idx][region] = min_runtime[region] * (1 + perf_margin)

            # create freq map
            freq_map_str = 'stream:{},dgemm:{}'.format(optimal_freq['stream'], optimal_freq['dgemm'])

            # set baseline and best fit freqs
            if is_baseline_sticker:
                baseline_freq = sticker_freq
                best_fit_freq = optimal_freq['epoch']
            else:
                baseline_freq = optimal_freq['epoch']
                if baseline_freq < default_freq:
                    baseline_freq = default_freq
                best_fit_freq = baseline_freq

            # Gather the output from all runs for this mix ratio
            pkl_file = ee_mix_file_path + '/{}_mix_{}.pkl'.format(name, ratio_idx)
            if self.use_cached_data_frames:
                with open(pkl_file, 'rb') as f:
                    report_df = pickle.load(f)
            else:
                report_glob = '{}_*_mix_{}.report'.format(name, ratio_idx)
                report_glob = ee_mix_file_path + '/*/' + report_glob
                report_output = geopmpy.io.AppOutput(report_glob, verbose=False)
                report_df = report_output.get_report_df()
                with open(pkl_file, 'wb') as f:
                    pickle.dump(report_df, f)

            # remove name from index - allows arith ops between data frames with different names
            baseline_name = '{}_{}_{}'.format(name, baseline_freq, ratio_idx)
            best_fit_name = '{}_{}_{}'.format(name, best_fit_freq, ratio_idx)
            dynamic_optimal_name = '{}_optimal_{}'.format(name, ratio_idx)
            adaptive_optimal_name = '{}_adaptive_{}'.format(name, ratio_idx)

            baseline_df = report_df.loc[idx[:, baseline_name, :, :, :, :, :, :], ]
            best_fit_df = report_df.loc[idx[:, best_fit_name, :, :, :, :, :, :], ]
            dynamic_optimal_df = report_df.loc[idx[:, dynamic_optimal_name, :, :, :, :, :, :], ]
            adaptive_optimal_df = report_df.loc[idx[:, adaptive_optimal_name, :, :, :, :, :, :], ]

            baseline_df = baseline_df.reset_index('name', drop=True)
            best_fit_df = best_fit_df.reset_index('name', drop=True)
            dynamic_optimal_df = dynamic_optimal_df.reset_index('name', drop=True)
            adaptive_optimal_df = adaptive_optimal_df.reset_index('name', drop=True)

            # energy savings
            best_fit_energy_savings = (baseline_df['energy'] - best_fit_df['energy']) / baseline_df['energy']
            best_fit_energy_savings = best_fit_energy_savings.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

            dynamic_energy_savings = (baseline_df['energy'] - dynamic_optimal_df['energy']) / baseline_df['energy']
            dynamic_energy_savings = dynamic_energy_savings.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

            adaptive_energy_savings = (baseline_df['energy'] - adaptive_optimal_df['energy']) / baseline_df['energy']
            adaptive_energy_savings = adaptive_energy_savings.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

            self.m_result_df['best_fit_energy'][ratio_idx+1] = best_fit_energy_savings
            self.m_result_df['dynamic_energy'][ratio_idx+1] = dynamic_energy_savings
            self.m_result_df['adaptive_energy'][ratio_idx+1] = adaptive_energy_savings

            # runtime savings
            best_fit_runtime_savings = (baseline_df['runtime'] - best_fit_df['runtime']) / baseline_df['runtime']
            best_fit_runtime_savings = best_fit_runtime_savings.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

            dynamic_runtime_savings = (baseline_df['runtime'] - dynamic_optimal_df['runtime']) / baseline_df['runtime']
            dynamic_runtime_savings = dynamic_runtime_savings.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

            adaptive_runtime_savings = (baseline_df['runtime'] - adaptive_optimal_df['runtime']) / baseline_df['runtime']
            adaptive_runtime_savings = adaptive_runtime_savings.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

            self.m_result_df['best_fit_runtime'][ratio_idx+1] = best_fit_runtime_savings*-100.0
            self.m_result_df['dynamic_runtime'][ratio_idx+1] = dynamic_runtime_savings*-100.0
            self.m_result_df['adaptive_runtime'][ratio_idx+1] = adaptive_runtime_savings*-100.0

            # normalized runtime
            best_fit_norm_runtime = best_fit_df['runtime'] / baseline_df['runtime']
            best_fit_norm_runtime = best_fit_norm_runtime.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

            dynamic_norm_runtime = dynamic_optimal_df['runtime'] / baseline_df['runtime']
            dynamic_norm_runtime = dynamic_norm_runtime.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

            adaptive_norm_runtime = adaptive_optimal_df['runtime'] / baseline_df['runtime']
            adaptive_norm_runtime = adaptive_norm_runtime.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

            # percent of runtime that is stream at baseline freq
            baseline_runtime = baseline_df['runtime']
            baseline_stream_runtime = baseline_runtime.loc[idx[:, :, :, :, :, :, 'stream'], ].mean()
            baseline_epoch_runtime = baseline_runtime.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()
            percent_stream = baseline_stream_runtime / baseline_epoch_runtime

            sys.stderr.write('\nMix ratio index: {}\n'.format(ratio_idx))
            sys.stderr.write('DGEMM-STREAM: {}-{}\n'.format(ratio[0], ratio[1]))
            sys.stderr.write('Baseline frequency: {}\n'.format(baseline_freq))
            sys.stderr.write('Best fit frequency: {}\n'.format(best_fit_freq))
            sys.stderr.write('Frequency map dynamic: {}\n'.format(freq_map_str))
            sys.stderr.write('Energy savings ratio (best fit): {}\n'.format(best_fit_energy_savings))
            sys.stderr.write('Energy savings ratio (dynamic): {}\n'.format(dynamic_energy_savings))
            sys.stderr.write('Energy savings ratio (adaptive): {}\n'.format(adaptive_energy_savings))
            sys.stderr.write('Runtime savings ratio (best fit): {}\n'.format(best_fit_runtime_savings))
            sys.stderr.write('Runtime savings ratio (dynamic): {}\n'.format(dynamic_runtime_savings))
            sys.stderr.write('Runtime savings ratio (adaptive): {}\n'.format(adaptive_runtime_savings))
            sys.stderr.write('Runtime % of stream: {}\n'.format(percent_stream))

            # for summary to be used with excel
            self.m_energy_by_percent[percent_stream] = (best_fit_energy_savings, adaptive_energy_savings, dynamic_energy_savings)
            self.m_runtime_by_percent[percent_stream] = (best_fit_runtime_savings, adaptive_runtime_savings, dynamic_runtime_savings)
            self.m_norm_runtime_by_percent[percent_stream] = (best_fit_norm_runtime, adaptive_norm_runtime, dynamic_norm_runtime)
            self.m_best_fit_freq_by_percent[percent_stream] = best_fit_freq
            self.m_big_o_by_percent[percent_stream] = mix_ratios[ratio_idx]
            self.m_ratio_idx_by_percent[percent_stream] = ratio_idx

            # end mix loop

        for mix_ratio, runtime_by_freq in sorted(sweep_runtime_by_mix_by_freq.items()):
            print mix_ratio, mix_ratios[mix_ratio], 'target:', target_runtime_by_mix[mix_ratio]
            for region in ['dgemm', 'stream', 'epoch']:
                print region
                for freq, region_runtime in sorted(runtime_by_freq.items()):
                    row = str(freq/1.0e9)
                    row += ',' + str(region_runtime[region])
                    row += ',' + str(sweep_energy_by_mix_by_freq[mix_ratio][freq][region])
                    print row
        # endef


    def make_savings_bar(self, name):
        f, ax = plt.subplots()
        bar_width = 0.35

        pcts = sorted(self.m_ratio_idx_by_percent.keys())
        print 'pcts', pcts
        index = numpy.array([self.m_ratio_idx_by_percent[p] for p in pcts])
        print 'index', index
        xticks = numpy.arange(len(index))
        print 'xticks', xticks

        df = self.m_result_df
        print 'df', df
        print 'df eng', df['best_fit_'+name]
        eng = df['best_fit_'+name]
        print 'eng', eng
        values = numpy.array([eng[p+1] for p in index])
        print 'y val best fit', values

        print self.m_result_df
        print df
        print df['best_fit_'+name]
        ax.bar(xticks - bar_width / 2,
               values,
               width=bar_width,
               color='blue',
               align='center',
               label='best fit')


        eng = df['dynamic_'+name]
        values = numpy.array([eng[p+1] for p in index])
        print 'y val dynamic', values
        ax.bar(xticks + bar_width / 2,
               values,
               width=bar_width,
               color='cyan',
               align='center',
               label='dynamic')


        eng = df['adaptive_'+name]
        values = numpy.array([eng[p+1] for p in index])
        print 'y val adaptive', values
        ax.bar(xticks + bar_width / 2,
               values,
               width=bar_width,
               color='orange',
               align='center',
               label='adaptive')


        ax.set_xticks(xticks)
        ax.set_xticklabels([format(p, '.2f') for p in pcts])

        ax.set_ylabel(name + ' Savings from Baseline (%)')
        ax.grid(axis='y', linestyle='--', color='black')

        plt.title(name + ' Savings')
        plt.margins(0.02, 0.01)
        plt.axis('tight')
        legend = plt.legend(shadow=True, fancybox=True, fontsize=14, loc='best')
        #legend.set_zorder(11)
        plt.tight_layout()

        plt.savefig(name + '_bar1.png')
        plt.close()

    def print_csv(self):
        rs = 'Frequency\n'
        for pct in sorted(self.m_best_fit_freq_by_percent.keys()):
            rs += '{},{}\n'.format(pct, self.m_best_fit_freq_by_percent[pct])

        rs += 'Big O\n'
        for pct in sorted(self.m_big_o_by_percent.keys()):
            rs += '{},{}\n'.format(pct, self.m_big_o_by_percent[pct])

        rs += 'Energy savings\n'
        rs += 'PCT stream, best fit, adaptive, static per phase\n'
        for pct in sorted(self.m_energy_by_percent.keys()):
            data = self.m_energy_by_percent[pct]
            rs += '{},{},{},{}\n'.format(pct, data[0], data[1], data[2])

        rs += 'Runtime savings\n'
        rs += 'PCT stream, best fit, adaptive, static per phase\n'
        for pct in sorted(self.m_runtime_by_percent.keys()):
            data = self.m_runtime_by_percent[pct]
            rs += '{},{},{},{}\n'.format(pct, data[0], data[1], data[2])

        rs += 'Normalized runtime\n'
        rs += 'PCT stream, best fit, adaptive, static per phase\n'
        for pct in sorted(self.m_norm_runtime_by_percent.keys()):
            data = self.m_norm_runtime_by_percent[pct]
            rs += '{},{},{},{}\n'.format(pct, data[0], data[1], data[2])
        return rs

if __name__ == '__main__':
    os.environ['COLUMNS'] = str(140)
    pandas.set_option('display.width', 140)       # Same tweak for Pandas

    plotter = Sc17Plotter(sys.argv[1])
    plotter.parse()
    sys.stderr.write(plotter.print_csv())
    #plotter.make_savings_bar('energy')
    #plotter.make_savings_bar('runtime')
    # code.interact(local=dict(globals(), **locals()))
    print plotter.m_ratio_idx_by_percent
