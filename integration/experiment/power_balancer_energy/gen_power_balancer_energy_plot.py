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
Plot energy and runtime impacts of running the power balancer at a
range of budgets.
'''

import argparse
import sys
import os
import numpy
import pandas
import matplotlib.pyplot as plt

import geopmpy.io
import geopmpy.hash
from experiment import common_args


# TODO: we assume profile name is unique for baseline, but we might want the agent also


def extract_prefix(name):
    ''' Remove the iteration number after the last underscore. '''
    return '_'.join(name.split('_')[:-1])


def summary(df, baseline, targets, show_details):
    # remove iteration from end of profile name
    df['Profile'] = df['Profile'].apply(extract_prefix)

    # rename some columns
    df['energy_pkg'] = df['package-energy (joules)']
    df['runtime'] = df['runtime (sec)']

    # choose columns of interest
    df = df[['Profile', 'Agent', 'energy_pkg', 'runtime']]
    if show_details:
        sys.stdout.write('{}\n'.format(df))

    # check that baseline is present
    profiles = df['Profile'].unique()
    if baseline not in profiles:
        raise RuntimeError('Baseline profile prefix "{}" not present in data.'.format(baseline))
    base_data = df.loc[df['Profile'] == baseline]

    base_runtime = base_data['runtime'].mean()
    base_energy = base_data['energy_pkg'].mean()

    data = []
    xnames = []
    for target_name in targets:
        # check that target is present
        if target_name not in profiles:
            raise RuntimeError('Target profile prefix "{}" not present in data.'.format(target_name))
        target_data = df.loc[df['Profile'] == target_name]
        target_runtime = target_data['runtime'].mean()
        target_energy = target_data['energy_pkg'].mean()
        #pct_runtime = (base_runtime - target_runtime) / base_runtime
        #pct_energy = (base_energy - target_energy) / base_energy
        norm_runtime = target_runtime / base_runtime
        norm_energy = target_energy / base_energy
        data.append([target_runtime, target_energy, norm_runtime, norm_energy])
        xnames.append(target_name)

    result = pandas.DataFrame(data, index=xnames, columns=['runtime', 'energy_pkg', 'runtime_norm', 'energy_norm'])
    return result


def plot_bars(df, output_dir):

    labels = df.index.format()
    title = os.path.commonprefix(labels)
    labels = list(map(lambda x: x[len(title):], labels))
    title = title.rstrip('_')
    points = numpy.arange(len(df.index))
    bar_width = 0.35

    plt.bar(points - bar_width / 2, df['runtime_norm'], width=bar_width, color='red')
    plt.bar(points + bar_width / 2, df['energy_norm'], width=bar_width, color='cyan')
    plt.axhline(y=1.0, color='orange', linestyle='dotted')

    plt.xticks(points, labels=labels, rotation='vertical')
    plt.subplots_adjust(bottom=0.25)
    plt.title(title)
    if not os.path.exists(os.path.join(output_dir, 'figures')):
        os.mkdir(os.path.join(output_dir, 'figures'))
    plt.savefig(os.path.join(output_dir, 'figures', '1240_bar.png'))


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    common_args.add_output_dir(parser)
    common_args.add_show_details(parser)

    parser.add_argument('--baseline', dest='baseline',
                        action='store', default=None,
                        help='baseline profile name')
    # TODO: if empty, use all other besides baseline ?
    # or could be positional args if required
    parser.add_argument('--targets', dest='targets',
                        action='store', default=None,
                        help='comma-separated list of profile names to compare')
    # could return list in format that --targets ingests
    parser.add_argument('--show-profiles', dest='show_profiles',
                        action='store_true', default=False,
                        help='show all profiles present in the discovered reports')

    args, extra_cli_args = parser.parse_known_args()

    if (not args.baseline or not args.targets) and not args.show_profiles:
        sys.stderr.write('Profile prefixes for baseline and targets must be provided.  Show all profiles with --show-profiles.\n')
        parser.print_help()
        sys.exit(1)

    output_dir = args.output_dir
    show_details = args.show_details
    try:
        rrc = geopmpy.io.RawReportCollection("*report", dir_name=output_dir)
    except:
        sys.stderr.write('<geopm> Error: No report data found in {}\n'.format(output_dir))
        sys.exit(1)

    if args.show_profiles:
        profile_list = rrc.get_app_df()['Profile'].unique()
        profile_list = sorted(list(set(map(extract_prefix, profile_list))))
        sys.stdout.write(','.join(profile_list) + '\n')
        sys.exit(0)

    baseline = args.baseline
    targets = args.targets.split(',')

    result = summary(rrc.get_app_df(), baseline, targets, show_details)
    if show_details:
        sys.stdout.write('{}\n'.format(result))

    plot_bars(result, output_dir)
