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

"""POWER_SWEEP

"""

import sys
import unittest
import os
import glob
import math
import pandas
import matplotlib.pyplot as plt

import geopmpy.io
import geopmpy.error


def load_files():
    #self._output_dir = '.'
    #output = geopmpy.io.AppOutput(self._prefix+'*.report', None, dir_name=self._output_dir, verbose=True)
    #output.extract_index_from_profile(inplace=True)
    if len(sys.argv) < 2:
        print('filename path')
        sys.exit(1)
    target_report_dir = sys.argv[1]  # '/home/drguttma/output/last_dgemm'

    prefix = '*power_governor'
    report_glob = prefix + '*report'
    if target_report_dir:
        report_glob = target_report_dir + '/' + report_glob

    cc = geopmpy.io.RawReportCollection(report_glob)
    return cc


def test_summary():
    df = load_files()

    sys.stdout.write('{}\n'.format(df))


    return

    # profile name has been changed to power cap
    df = output.get_report_data(profile=(self._min_power, self._max_power),
                                agent=self._agent_type,
                                region='epoch')
    summary = pandas.DataFrame()
    for col in ['count', 'runtime', 'network_time', 'energy_pkg', 'energy_dram', 'frequency']:
        summary[col] = df[col].groupby(level='name').mean()
    summary.index.rename('power cap', inplace=True)

    rs = 'Summary for power sweep'
    #rs = 'Summary for {} with {} agent\n'.format(self._name, self._agent_type)
    rs += summary.to_string()
    sys.stdout.write(rs + '\n')
    report = geopmpy.io.RawReport(self._report_path)


def generate_histogram(data, profile_name, min_drop, max_drop, label, bin_size,
                       xprecision):
    data = data[label]
    print("DRG ", len(data))
    fontsize = 12
    fig_size = (8, 4)
    verbose = True

    # TODO: fix in analysis.py
    if 'nekbone' in profile_name:
        profile_name = profile_name.replace('nekbone', 'Nekbone')
    elif 'dgemm' in profile_name:
        profile_name = profile_name.replace('dgemm', 'DGEMM')
    elif 'minife' in profile_name:
        profile_name = profile_name.replace('minife', 'MiniFE')
    elif 'amg' in profile_name:
        profile_name = profile_name.replace('amg', 'AMG')

    if label.lower() == 'power':
        axis_units = 'W'
        title_units = 'W'
        range_factor = 1
        title = '{}: Histogram of Power (No Capping)'.format(profile_name)
        bar_color = 'red'
    elif label.lower() == 'frequency':
        axis_units = 'GHz'
        title_units = 'MHz'
        range_factor = 1000
        title = '{} Histogram of Achieved Frequency'.format(profile_name)
        bar_color = 'blue'
    elif label.lower() == 'energy':
        axis_units = 'J'
        title_units = 'J'
        range_factor = 1
        title = '{} Histogram of Energy'.format(profile_name)
        bar_color = 'cyan'
    else:
        raise RuntimeError("<geopmpy>: Unknown type for histogram: {}".format(label))

    plt.figure(figsize=fig_size)
    bins = [round(bb*bin_size, 3) for bb in range(int(min_drop/bin_size), int(max_drop/bin_size)+2)]
    n, bins, patches = plt.hist(data, rwidth=0.8, bins=bins, color=bar_color)
    for n, b in zip(n, bins):
        plt.annotate(int(n) if int(n) != 0 else "", xy=(b+bin_size/2.0, n+2.5),
                     horizontalalignment='center',
                     fontsize=fontsize-4)
    min_max_range = (max(data) - min(data)) * range_factor
    mean = data.mean() * range_factor

    n = len(data)
    trim_pct = 0.05
    trimmed_data = data[int(n*trim_pct):n-int(trim_pct*n)]
    trimmed_min_max = (max(trimmed_data) - min(trimmed_data)) * range_factor
    plt.title('{}\nMin-max Var.: {} {}; {}% Min-max Var.: {} {}; Mean: {} {}'
              .format(title, round(min_max_range, 3), title_units,
                      int((trim_pct)*100), round(trimmed_min_max, 3), title_units,
                      round(mean, 3), title_units),
              fontsize=fontsize)
    plt.xlabel('{} ({})'.format(label.title(), axis_units), fontsize=fontsize)
    plt.ylabel('Node Count', fontsize=fontsize)
    plt.xticks([b+bin_size/2.0 for b in bins],
               [' [{start:.{prec}f}, {end:.{prec}f})'.format(start=b, end=b+bin_size, prec=xprecision) for b in bins],
               rotation='vertical',
               fontsize=fontsize-4)
    _, ylabels = plt.yticks()
    plt.setp(ylabels, fontsize=fontsize-4)

    plt.margins(0.02, 0.2)
    plt.axis('tight')

    plt.tight_layout()
    output_dir = 'figures'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    filename = '{}_{}_histo'.format(profile_name.replace('@', '_').replace(' ', '_'), label)
    # todo: could be a method
    for ext in ['png']:
        full_path = os.path.join(output_dir, '{}.{}'.format(filename, ext))
        plt.savefig(full_path)
        if verbose:
            sys.stdout.write('    {}\n'.format(full_path))
    plt.close()


def test_achieved_freq_histogram_package():

    # TODO:
    min_freq = 1.0e9
    max_freq = 2.2e9
    step_freq = 100e6
    sticker_freq = 2.0e9

    cc = load_files()
    report_df = cc.get_epoch_df()

    #report_df['frequency'] = report_df['frequency (Hz)']
    report_df['power_limit'] = report_df['POWER_PACKAGE_LIMIT_TOTAL']

    temp_df = report_df.copy()
    report_df['frequency'] = report_df['CYCLES_THREAD@package-0'] / report_df['CYCLES_REFERENCE@package-0']
    temp_df['frequency'] = temp_df['CYCLES_THREAD@package-1'] / temp_df['CYCLES_REFERENCE@package-1']
    report_df['freq_package'] = 0
    temp_df['freq_package'] = 1
    report_df.set_index(['power_limit', 'host', 'freq_package'], inplace=True)
    temp_df.set_index(['power_limit', 'host', 'freq_package'], inplace=True)
    report_df = report_df.append(temp_df)

    # convert percent to GHz frequency based on sticker
    report_df['frequency'] *= sticker_freq / 1e9

    profiles = report_df['POWER_PACKAGE_LIMIT_TOTAL'].unique()
    power_caps = sorted(profiles)  # list(range(self._min_power, self._max_power+1, self._step_power))
    gov_freq_data = {}
    bal_freq_data = {}
    for target_power in power_caps:
        governor_data = report_df.loc[report_df["Agent"] == "power_governor"]
        governor_data = governor_data.loc[governor_data['POWER_PACKAGE_LIMIT_TOTAL'] == target_power]
        gov_freq_data[target_power] = governor_data.groupby(['host', 'freq_package']).mean()['frequency'].sort_values()
        gov_freq_data[target_power] = pandas.DataFrame(gov_freq_data[target_power])
        print(gov_freq_data[target_power])

        balancer_data = report_df.loc[report_df["Agent"] == "power_balancer"]
        balancer_data = balancer_data.loc[balancer_data['POWER_PACKAGE_LIMIT_TOTAL'] == target_power]
        bal_freq_data[target_power] = governor_data.groupby(['host', 'freq_package']).mean()['frequency'].sort_values()
        bal_freq_data[target_power] = pandas.DataFrame(gov_freq_data[target_power])
    print(gov_freq_data, len(gov_freq_data))

    # plot histograms
    min_drop = min_freq / 1e9
    max_drop = (max_freq - step_freq) / 1e9
    bin_size = step_freq / 1e9 / 2.0
    name = 'APP'
    for target_power in power_caps:
        gov_data = gov_freq_data[target_power]
        bal_data = bal_freq_data[target_power]

        profile_name = name + "@" + str(target_power) + "W Governor"
        generate_histogram(gov_data, profile_name, min_drop, max_drop, 'frequency',
                           bin_size, 3)
        profile_name = name + "@" + str(target_power) + "W Balancer"
        generate_histogram(bal_data, profile_name, min_drop, max_drop, 'frequency',
                           bin_size, 3)


def test_achieved_freq_histogram_node():

    # TODO:
    min_freq = 1.0e9
    max_freq = 2.2e9
    step_freq = 100e6
    sticker_freq = 2.1e9
    #min_power = min(profiles)
    #max_power = max(profiles)

    cc = load_files()
    report_df = cc.get_epoch_df()

    # '''    CYCLES_REFERENCE
    # CYCLES_THREAD
    # ENERGY_PACKAGE
    # TIME'''

    report_df['power_limit'] = report_df['POWER_PACKAGE_LIMIT_TOTAL']

    #balancer_data = parse_output.get_report_data(profile=profile, agent="power_balancer", region='epoch')
    balancer_data = report_df.loc[report_df["Agent"] == "power_balancer"]
    # TODO: power cap filter
    bal_freq_data[target_power] = balancer_data.groupby('host').mean()['frequency'].sort_values()

    # convert percent to GHz frequency based on sticker
    gov_freq_data[target_power] /= 1e9
    bal_freq_data[target_power] /= 1e9
    gov_freq_data[target_power] = pandas.DataFrame(gov_freq_data[target_power])
    bal_freq_data[target_power] = pandas.DataFrame(bal_freq_data[target_power])

    print(gov_freq_data)
    process_output = gov_freq_data, bal_freq_data
    gov_freq_data, bal_freq_data = process_output

    min_drop = min_freq / 1e9
    max_drop = (max_freq - step_freq) / 1e9
    bin_size = step_freq / 1e9 / 2.0
    name = 'DGEMM'
    for target_power in power_caps:
        gov_data = gov_freq_data[target_power]
        bal_data = bal_freq_data[target_power]

        profile_name = name + "@" + str(target_power) + "W Governor"
        generate_histogram(gov_data, profile_name, min_drop, max_drop, 'frequency',
                           bin_size, 3)
        profile_name = name + "@" + str(target_power) + "W Balancer"
        #generate_histogram(bal_data, profile_name, min_drop, max_drop, 'frequency',
        #                   bin_size, 3)


def test_runtime_energy_plot():
    def generate_runtime_energy_plot(df, name, output_dir='.'):
        """
        Creates a plot comparing the runtime and energy of a region on two axes.
        """
        f, ax = plt.subplots()

        ax.plot(df.index, df['energy'], color='DarkBlue', label='Energy', marker='o', linestyle='-')
        ax.set_xlabel('Power (W)')
        ax.set_ylabel('Energy (J)')

        ax2 = ax.twinx()
        ax2.plot(df.index, df['runtime'], color='Red', label='Runtime', marker='s', linestyle='-')
        ax2.set_ylabel('Runtime (s)')

        lines, labels = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()

        plt.legend(lines + lines2, labels + labels2, shadow=True, fancybox=True, loc='best')
        plt.title('{}'.format(name))
        f.tight_layout()
        plt.savefig(os.path.join(output_dir, '{}_runtime_energy.{}'.format(name.replace(' ', '_'), 'png')))
        plt.close()

    cc = load_files()
    report_df = cc.get_df()
    report_df = report_df.loc[report_df['region'] == 'dgemm']
    print(report_df)
    print(cc.get_epoch_df())

    # this should be some policy; could also be fixed frequency
    grouped = report_df.groupby('POWER_PACKAGE_LIMIT_TOTAL')
    result_df = pandas.DataFrame()
    #result_df['runtime'] = grouped['runtime (sec)'].mean()
    result_df['runtime'] = grouped['sync-runtime (sec)'].mean()
    result_df['energy'] = grouped['package-energy (joules)'].mean()
    generate_runtime_energy_plot(result_df, 'Runtime & Energy vs. Power')


if __name__ == '__main__':
    #test_summary()
    #test_runtime_energy_plot()
    test_achieved_freq_histogram_package()
