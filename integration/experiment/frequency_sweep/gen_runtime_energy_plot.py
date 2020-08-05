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

'''
Example power sweep experiment using geopmbench.
'''

import sys
import os
import pandas
import argparse
import matplotlib.pyplot as plt
import numpy

import geopmpy.io

from experiment import common_args


def generate_runtime_energy_plot(df, name, output_dir='.'):
    """
    Creates a plot comparing the runtime and energy of a region on two axes.
    """
    f, ax = plt.subplots()

    ax.plot(df.index, df['energy_pkg'], color='DarkBlue', label='Energy', marker='o', linestyle='-')
    ax.set_xlabel('Frequency (GHz)')
    ax.set_ylabel('Energy (J)')

    ax2 = ax.twinx()
    ax2.plot(df.index, df['runtime'], color='Red', label='Runtime', marker='s', linestyle='-')
    ax2.set_ylabel('Runtime (s)')

    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    plt.legend(lines + lines2, labels + labels2, shadow=True, fancybox=True, loc='best')
    plt.title('{}'.format(name))
    f.tight_layout()
    plt.savefig(os.path.join(output_dir, '{}_freq_energy.{}'.format(name.replace(' ', '_'), 'png')))
    plt.close()

def summary_process(self, parse_output):
    output = {}
    report_df = parse_output.get_report_df()
    output['region_freq_map'] = self._region_freq_map(parse_output)
    output['means_df'] = self._region_means_df(report_df)
    return output

def summary(self, process_output):
    if self._verbose:
        sys.stdout.write(all_region_data_pretty(process_output['means_df']))
    sys.stdout.write(self._region_freq_str_pretty(process_output['region_freq_map']))

def plot_process(self, report_df):
    report_df = parse_output.get_report_df()
    regions = report_df.index.get_level_values('region').unique().tolist()
    return {region: self._runtime_energy_sweep(parse_output, region)
            for region in regions}

def plot(self, process_output):
    for region, df in process_output.items():
        geopmpy.plotter.generate_runtime_energy_plot(df, region, self._output_dir)

def _region_freq_map(self, parse_output):
    """
    Calculates the best-fit frequencies for each region for a single
    mix ratio.
    """
    optimal_freq = dict()
    min_runtime = dict()

    report_df = parse_output.get_report_df()
    freq_pname = FreqSweepAnalysis.get_freq_profiles(report_df, self._name)

    is_once = True
    for freq, profile_name in freq_pname:
        # Since we are still attempting to perform a run in the turbo range, we need to skip this run when
        # determining the best per region frequencies below.  The profile_name that corresponds to the
        # turbo run is always the first in the list.
        if not self._enable_turbo and (freq, profile_name) == freq_pname[0]:
            continue

        prof_df = parse_output.get_report_data(profile=profile_name)
        region_mean_runtime = prof_df.groupby(level='region')
        for region, region_df in region_mean_runtime:
            runtime = region_df['runtime'].mean()
            if is_once:
                min_runtime[region] = runtime
                optimal_freq[region] = freq
            elif min_runtime[region] > runtime:
                min_runtime[region] = runtime
                optimal_freq[region] = freq
            elif min_runtime[region] * (1.0 + self._perf_margin) > runtime:
                optimal_freq[region] = freq
        is_once = False
    return optimal_freq

def _region_freq_str(self, region_freq_map):
    """
    Format the mapping of region names to their best-fit frequencies.
    """
    return json.dumps(region_freq_map)

def _region_freq_str_pretty(self, region_freq_map):
    s = '\nRegion frequency map: \n'
    for k, v in region_freq_map.items():
        s += '    {}: {}\n'.format(k, v)
    return s


def _runtime_energy_sweep(df, region):

    # freq_pname = FreqSweepAnalysis.get_freq_profiles(df, self._name)
    # data = []
    # freqs = []
    # for freq, profile_name in freq_pname:
    #     freqs.append(freq)
    #     freq_df = parse_output.get_report_data(profile=profile_name)

    #     region_mean_runtime = freq_df.groupby(level='region')['runtime'].mean()
    #     region_mean_energy = freq_df.groupby(level='region')['energy_pkg'].mean()
    #     data.append([region_mean_runtime[region],
    #                  region_mean_energy[region]])

    result = df.groupby(['region', 'freq_mhz']).mean()
    print(result)

    # return pandas.DataFrame(data,
    #                         index=freqs,
    #                         columns=['runtime', 'energy_pkg'])


def _region_means_df(report_df):
    idx = pandas.IndexSlice

    perf_margin = 0.1

    cols = ['energy_pkg', 'runtime', 'network_time', 'frequency', 'count']

    means_df = report_df.groupby(['region', 'freq_mhz'])[cols].mean()
    # Define ref_freq to be three steps back from the end of the list.  The end of the list should always be
    # the turbo frequency.
    # TODO: this doesn't work if there are fewer than 4 frequencies.  Also
    # shouldn't this compare with the sticker or last freq?  turbo is not in the list
    # unless included in the range
    ref_freq = report_df.index.get_level_values('freq_mhz').unique().tolist()[-1]

    # Calculate the energy/runtime comparisons against the ref_freq
    ref_energy = means_df.loc[idx[:, ref_freq], ]['energy_pkg'].reset_index(level='freq_mhz', drop=True)
    es = pandas.Series((means_df['energy_pkg'] - ref_energy) / ref_energy, name='DCEngVar_%')
    means_df = pandas.concat([means_df, es], axis=1)

    ref_runtime = means_df.loc[idx[:, ref_freq], ]['runtime'].reset_index(level='freq_mhz', drop=True)
    rs = pandas.Series((means_df['runtime'] - ref_runtime) / ref_runtime, name='TimeVar_%')
    means_df = pandas.concat([means_df, rs], axis=1)

    bs = pandas.Series(means_df['runtime'] * (1.0 + perf_margin), name='runtime_bound')
    means_df = pandas.concat([means_df, bs], axis=1)

    # Calculate power and kwh
    p = pandas.Series(means_df['energy_pkg'] / means_df['runtime'], name='power')
    means_df = pandas.concat([means_df, p], axis=1)

    # Modify column order so that runtime bound occurs just after runtime
    cols = means_df.columns.tolist()
    tmp = cols.pop(7)
    cols.insert(2, tmp)
    return means_df[cols]


def plot_runtime_energy(report_df, output_dir):
    # rename some columns
    report_df['freq_mhz'] = report_df['FREQ_DEFAULT']
    report_df['runtime'] = report_df['runtime (sec)']
    report_df['network_time'] = report_df['network-time (sec)']
    report_df['energy_pkg'] = report_df['package-energy (joules)']
    report_df['frequency'] = report_df['frequency (Hz)']
    report_df = report_df.set_index(['Agent', 'freq_mhz', 'Profile', 'host', 'region'])

    # select only columns of interest
    report_df = report_df[['runtime', 'network_time', 'energy_pkg',
                           'frequency', 'count']]

    # find region of interest
    # TODO: command line option to override
    max_freq = report_df.index.get_level_values(level='freq_mhz').max()
    import code
    code.interact(local=dict(globals(), **locals()))
    temp = report_df.loc[max_freq]
    longest_region = 8

    _runtime_energy_sweep(report_df, 'na')

    means_df = _region_means_df(report_df)

    generate_runtime_energy_plot(df, region, soutput_dir)
    print(means_df)


def midden():
    edf = report_data

    # rename some columns
    edf['power_limit'] = edf['POWER_PACKAGE_LIMIT_TOTAL']
    edf['runtime'] = edf['runtime (sec)']
    edf['network_time'] = edf['network-time (sec)']
    edf['energy_pkg'] = edf['package-energy (joules)']
    edf['frequency'] = edf['frequency (Hz)']

    edf = edf.set_index(['Agent', 'power_limit', 'Profile', 'host'])

    df = pandas.DataFrame()
    units = {
        'energy': 'J',
        'energy_pkg': 'J',
        'runtime': 's',
        'frequency': '% of sticker',
        'power': 'W',
    }

    for ext in output_types:
        if not os.path.exists(os.path.join(output_dir, 'figures')):
            os.mkdir(os.path.join(output_dir, 'figures'))
        full_path = os.path.join(output_dir, 'figures', '{}.{}'.format(file_name, ext))
        plt.savefig(full_path)
        if detailed:
            sys.stdout.write('    {}\n'.format(full_path))
    sys.stdout.flush()
    plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    common_args.add_output_dir(parser)
    common_args.add_show_details(parser)
    # TODO: move to common args and replace in others
    parser.add_argument('--label', action='store', default="",
                        help='name of the application to use for plot titles')

    args, _ = parser.parse_known_args()
    output_dir = args.output_dir

    try:
        output = geopmpy.io.RawReportCollection("*report", dir_name=output_dir)
    except:
        sys.stderr.write('<geopm> Error: No report data found in {}; run a frequency sweep before using this analysis\n'.format(output_dir))
        sys.exit(1)


    plot_runtime_energy(output.get_df(),
                        output_dir=output_dir)
