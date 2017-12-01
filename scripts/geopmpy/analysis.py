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
GEOPM Analysis - Used to run applications and analyze results for specific GEOPM use cases.
"""

import argparse
import sys
import os
import glob
from collections import defaultdict
import geopmpy.io
import geopmpy.launcher
import geopmpy.plotter
from geopmpy import __version__

import pandas

def sys_freq_avail():
    """
    Returns a list of the available frequencies on the current platform.
    """
    step_freq = 100e6
    with open('/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq') as fid:
        min_freq = 1e3 * float(fid.readline())
    with open('/proc/cpuinfo') as fid:
        for line in fid.readlines():
            if line.startswith('model name\t:'):
                sticker_freq = float(line.split('@')[1].split('GHz')[0]) * 1e9
                break
        max_freq = sticker_freq + step_freq

    num_step = 1 + int((max_freq - min_freq) / step_freq)
    result = [step_freq * ss + min_freq for ss in range(num_step)]
    return result


class Analysis(object):
    """
    Base class for different types of analysis that use the data from geopm
    reports and/or logs. Implementations should define how to launch experiments,
    parse and process the output, and process text reports or graphical plots.
    """
    def __init__(self, name, output_dir, num_rank, num_node, app_argv):
        self._name = name
        self._output_dir = output_dir
        self._num_rank = num_rank
        self._num_node = num_node
        self._app_argv = app_argv
        self._report_paths = None
        self._trace_paths = None

    def launch(self):
        """
        Run experiment and set data paths corresponding to output.
        """
        raise NotImplementedError('Analysis base class does not implement the launch method()')

    def find_files(self):
        """
        Uses the output dir and any custom naming convention to load the report and trace data
        produced by launch.
        """
        raise NotImplementedError('Analysis base class does not implement the find_files method()')

    def set_data_paths(self, report_paths, trace_paths=None):
        """
        Set directory paths for report and trace files to be used in the analysis.
        """
        if not self._report_paths and not self._trace_paths:
            self._report_paths = report_paths
            self._trace_paths = trace_paths
        else:
            raise RuntimeError('Analysis object already has report and trace paths populated.')

    def parse(self):
        """
        Load any necessary data from the application result files into memory for analysis.
        """
        return geopmpy.io.AppOutput(self._report_paths, self._trace_paths, verbose=False)

    def plot_process(self, parse_output):
        """
        Process the parsed data into a form to be used for plotting (e.g. pandas dataframe).
        """
        raise NotImplementedError('Analysis base class does not implement the plot_process method()')

    def report_process(self, parse_output):
        """
        Process the parsed data into a form to be used for the text report.
        """
        raise NotImplementedError('Analysis base class does not implement the report_process method()')

    def report(self, process_output):
        """
        Print a text report of the results.
        """
        raise NotImplementedError('Analysis base class does not implement the report method()')

    def plot(self, process_output):
        """
        Generate graphical plots of the results.
        """
        raise NotImplementedError('Analysis base class does not implement the plot method()')


# TODO: might want to move to tests?
class MockAnalysis(Analysis):
    def __init__(self, name, output_dir, num_rank, num_node, app_argv):
        super(MockAnalysis, self).__init__(name, output_dir, num_rank, num_node, app_argv)
        self._output_path = 'mock_analysis.out'

    def __del__(self):
        os.unlink(self._output_path)

    def launch(self, geopm_ctl='process', do_geopm_barrier=False):
        with open(self._output_path, 'w') as fid:
            fid.write("This is the mock analysis output.\n")

    def parse(self):
        with open(self._output_path) as fid:
            result = fid.read()
        return result

    def report_process(self, parse_output):
        return parse_output

    def plot_process(self, parse_output):
        return self.report_process(parse_output)

    def report(self, process_output):
        sys.stdout.write('Report: {}'.format(process_output))

    def plot(self, process_output):
        sys.stdout.write('Plot: {}'.format(process_output))


class BalancerAnalysis(Analysis):
    pass


class FreqSweepAnalysis(Analysis):
    """
    Runs the application at each available frequency. Compared to the baseline
    frequency, finds the lowest frequency for each region at which the performance
    will not be degraded by more than a given margin.
    """
    def __init__(self, name, output_dir, num_rank, num_node, app_argv):
        super(FreqSweepAnalysis, self).__init__(name, output_dir, num_rank, num_node, app_argv)
        self._perf_margin = 0.1

    def launch(self, geopm_ctl='process', do_geopm_barrier=False):
        ctl_conf = geopmpy.io.CtlConf(self._name + '_app.config',
                                      'dynamic',
                                      {'power_budget': 400,
                                       'tree_decider': 'static_policy',
                                       'leaf_decider': 'simple_freq',
                                       'platform': 'rapl'})
        if 'GEOPM_SIMPLE_FREQ_RID_MAP' in os.environ:
            del os.environ['GEOPM_SIMPLE_FREQ_RID_MAP']
        if 'GEOPM_SIMPLE_FREQ_ADAPTIVE' in os.environ:
            del os.environ['GEOPM_SIMPLE_FREQ_ADAPTIVE']
        for freq in sys_freq_avail():
            profile_name = self._name + '_freq_{}'.format(freq)
            report_path = os.path.join(self._output_dir, profile_name + '.report')
            self._report_paths.append(report_path)
            if self._app_argv and not os.path.exists(report_path):
                os.environ['GEOPM_SIMPLE_FREQ_MIN'] = str(freq)
                os.environ['GEOPM_SIMPLE_FREQ_MAX'] = str(freq)
                argv = ['dummy', '--geopm-ctl', geopm_ctl,
                                 '--geopm-policy', ctl_conf.get_path(),
                                 '--geopm-report', report_path,
                                 '--geopm-profile', profile_name]
                if do_geopm_barrier:
                    argv.append('--geopm-barrier')
                argv.append('--')
                argv.extend(self._app_argv)
                launcher = geopmpy.launcher.factory(argv, num_rank, num_node)
                launcher.run()
            elif os.path.exists(report_path):
                sys.sterr.write('<geopmpy>: Warning: output file "{}" exists, skipping run.\n'.format(report_path))
            else:
                raise RuntimeError('<geopmpy>: output file "{}" does not exist, but no application was specified.\n'.format(report_path))

    def find_files(self):
        report_glob = os.path.join(self._output_dir, self._name + '_freq_*.report')
        self.set_data_paths(glob.glob(report_glob))

    def report_process(self, parse_output):
        return self._region_freq_map(parse_output.get_report_df())

    def plot_process(self, parse_output):
        return parse_output.get_report_df()

    def report(self, process_output):
        region_freq_str = self._region_freq_str(process_output)
        sys.stdout.write('Region frequency map: {}\n'.format(region_freq_str))
        return region_freq_str

    def plot(self, process_output):
        pass

    def _region_freq_map(self, report_df):
        """
        Calculates the best-fit frequencies for each region for a single
        mix ratio.
        This function assumes that the frequency set occurs in the report
        name following the last occurenace of the string '_freq_'
        """
        optimal_freq = dict()
        min_runtime = dict()

        profile_name_list = report_df.index.get_level_values('name').unique().tolist()

        freq_list = [float(pn.split('_freq_')[-1].split('_')[0])
                     for pn in profile_name_list
                     if '_freq_' in pn]
        freq_pname = zip(freq_list, profile_name_list)
        freq_pname.sort(reverse=True)

        is_once = True
        for freq, profile_name in freq_pname:
            region_mean_runtime = report_df.loc[pandas.IndexSlice[:, profile_name, :, :, :, :, :, :], ].groupby(level='region')
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
        result = ['{}:{}'.format(key, value)
                  for (key, value) in region_freq_map.iteritems()]
        result = ','.join(result)
        return result


class EnergyEfficiencyAnalysis(FreqSweepAnalysis):
    """
    Used to compare runs of an auto frequency plugin to runs at a fixed
    frequency for the application. The best-fit frequency is determined using the
    analysis from the FreqSweepAnalysis base class.
    """

    def __init__(self, name, output_dir, num_rank, num_node, app_argv):
        super(EnergyEfficiencyAnalysis, self).__init__(name, output_dir, num_rank, num_node, app_argv)

    def __del__(self):
        pass

    def launch(self, geopm_ctl='process', do_geopm_barrier=False):
        """
        Run the frequency sweep, then run the auto plugin in offline and online mode.
        """
        super(EnergyEfficiencyAnalysis, self).launch(geopm_ctl, do_geopm_barrier)

        # TODO:
        # call parent launch to do sweep
        # then run other plugins: offline vs online
        pass

    def find_files(self):
        report_glob = os.path.join(self._output_dir, self._name + '*.report')
        self.set_data_paths(glob.glob(report_glob))

    def parse(self):
        app_output = super(EnergyEfficiencyAnalysis, self).parse()
        return app_output.get_report_df()

    def report_process(self, parse_output):
        df = parse_output
        name_prefix = self._name
        step_freq = 100e6
        # for SC17 numbers, used max=2.2 and sticker = 2.1, baseline=sticker
        baseline_freq = 2.1e9
        sticker_freq = 2.1e9
        default_freq = sticker_freq - 4 * step_freq
        is_baseline_sticker = True  # TODO: could be parameter? need one test for each

        runtime_data = []
        energy_data = []
        app_freq_data = []
        online_freq_data = []
        series_names = ['offline application', 'offline per-phase', 'online per-phase']
        regions = {'epoch':  '9223372036854775808',
                   'dgemm':  '11396693813',
                   'stream': '20779751936'}  # TODO: get from data; get_region_id_map function
        #region_name_list = df.index.get_level_values('region').unique().tolist()

        # TODO: this would be easier to plot if these ratios were in order
        mix_ratios = [(1.0, 0.25), (1.0, 0.5), (1.0, 0.75), (1.0, 1.0),
                      (0.25, 1.0), (0.5, 1.0), (0.75, 1.0)]
        for (ratio_idx, ratio) in enumerate(mix_ratios):
            optimal_freq = self._region_freq_map(df) # FIXME may be combining all mix ratios 
            # set baseline and best fit freqs
            if is_baseline_sticker:
                baseline_freq = sticker_freq
                best_fit_freq = optimal_freq['epoch']
            else:
                baseline_freq = optimal_freq['epoch']
                if baseline_freq < default_freq:
                    baseline_freq = default_freq
                best_fit_freq = baseline_freq

            prof_name = {
                'baseline': '{}_{}_{}'.format(name_prefix, baseline_freq, ratio_idx),
                'offline application': '{}_{}_{}'.format(name_prefix, best_fit_freq, ratio_idx),
                'offline per-phase': '{}_optimal_{}'.format(name_prefix, ratio_idx),
                'online per-phase': '{}_adaptive_{}'.format(name_prefix, ratio_idx),
            }
            frame = {}
            runtime = {}
            energy = {}

            for prof, pname in prof_name.items():
                frame[prof] = df.loc[idx[:, pname, :, :, :, :, :, :], ]
                frame[prof].reset_index('name', drop=True, inplace=True)
                runtime[prof] = frame[prof]['runtime']
                energy[prof] = frame[prof]['energy']

            # calculate savings for each run, then get mean
            runtime_temp = []
            energy_temp = []
            for sname in series_names:
                runtime_savings = (runtime['baseline'] - runtime[sname]) / runtime['baseline']
                # show runtime as positive percent
                runtime_savings = runtime_savings.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean() * -100.0
                energy_savings = (energy['baseline'] - energy[sname]) / energy['baseline']
                energy_savings = energy_savings.loc[idx[:, :, :, :, :, :, 'epoch'], ].mean()

                runtime_temp.append(runtime_savings)
                energy_temp.append(energy_savings)
            runtime_data.append(runtime_temp)
            energy_data.append(energy_temp)

            # calculate chosen frequencies for each mix ratio
            # TODO: this is ignoring the hostname for now, because we get display the entire range
            adaptive_freq = defaultdict(list)
            log_glob = os.path.join(self._output_dir,
                                    '{}_adaptive_{}.log'.format(self._name, ratio_idx))
            files = glob.glob(log_glob)
            for filename in files:
                with open(filename) as infile:
                    last_freq = {}
                    for line in infile:
                        if 'is_new' in line:
                            line = line.strip().split()
                            if len(line) != 9:
                                # some interleaving problem or uninteresting line
                                continue
                            last_freq[line[4]] = float(line[6])
                for rid, freq in last_freq.items():
                    adaptive_freq[rid].append(freq)
            for rid in adaptive_freq:
                adaptive_freq[rid] = (min(adaptive_freq[rid]), max(adaptive_freq[rid]))

            app_freq_temp = []
            online_freq_temp = []
            for region in regions:
                app_freq_temp.append(optimal_freq[region])
                if region != 'epoch':
                    online_freq_temp.append(adaptive_freq[regions[region]])

            app_freq_data.append(app_freq_temp)
            online_freq_data.append(online_freq_temp)
            # end mix ratio loop

        # produce combined output
        energy_result_df = pandas.DataFrame(energy_data, columns=series_names)
        runtime_result_df = pandas.DataFrame(runtime_data, columns=series_names)
        app_freq_df = pandas.DataFrame(app_freq_data, columns=regions)
        # online does not need epoch
        del regions['epoch']
        online_freq_df = pandas.DataFrame(online_freq_data, columns=regions)

        # fix for ordering of mixes
        order = [0, 1, 2, 3, 6, 5, 4]
        energy_result_df = energy_result_df.reindex(order)
        runtime_result_df = runtime_result_df.reindex(order)
        app_freq_df = app_freq_df.reindex(order)
        online_freq_df = online_freq_df.reindex(order)

        return energy_result_df, runtime_result_df, app_freq_df, online_freq_df

    def plot_process(self, parse_output):
        return self.report_process(parse_output)

    def report(self, process_output):
        rs = 'Report\n'
        energy_result_df, runtime_result_df, app_freq_df, online_freq_df = process_output
        rs += 'Energy\n'
        rs += '{}\n'.format(energy_result_df)
        rs += 'Runtime\n'
        rs += '{}\n'.format(runtime_result_df)
        rs += 'Application best-fit frequencies\n'
        rs += '{}\n'.format(app_freq_df)
        rs += 'Online chosen frequencies\n'
        rs += '{}\n'.format(online_freq_df)
        print rs
        return rs

    def plot(self, process_output):
        sys.stdout.write('Plot')
        energy_result_df, runtime_result_df, app_freq_df, online_freq_df = process_output
        geopmpy.plotter.generate_bar_plot_sc17(energy_result_df, 'Energy-to-Solution Decrease')
        geopmpy.plotter.generate_bar_plot_sc17(runtime_result_df, 'Time-to-Solution Increase')
        geopmpy.plotter.generate_app_best_freq_plot_sc17(app_freq_df, 'Offline auto per-phase best-fit frequency')
        geopmpy.plotter.generate_best_freq_plot_sc17(online_freq_df, 'Online auto per-phase best-fit frequency')


def main(argv):
    help_str = """
Usage: {argv_0} [-h|--help] [--version]
       {argv_0} -t ANALYSIS_TYPE -n NUM_RANK -N NUM_NODE [-o OUTPUT_DIR] [-p PROFILE_PREFIX] [-l] -- EXEC [EXEC_ARGS]

geopmanalysis - Used to run applications and analyze results for specific
                GEOPM use cases.

  -h, --help            show this help message and exit

  -t, --analysis_type   type of analysis to perform. Available
                        ANALYSIS_TYPE values: freq_sweep, balancer, and baseline_comp.
  -n, --num_rank        total number of application ranks to launch with
  -N, --num_node        number of compute nodes to launch onto
  -o, --output_dir      the output directory for reports, traces, and plots (default '.')
  -p, --profile_prefix  prefix to prepend to profile name when launching
  -l, --level           controls the level of detail provided in the analysis.
                        level 0: run application and generate reports and traces only
                        level 1: print analysis of report and trace data (default)
                        level 2: create plots from report and trace data
  -s, --skip_launch     do not launch jobs, only analyze existing data
  --version             show the GEOPM version number and exit

""".format(argv_0=sys.argv[0])
    version_str = """\
GEOPM version {version}
Copyright (C) 2015, 2016, 2017, Intel Corporation. All rights reserved.
""".format(version=__version__)

    if '--help' in argv or '-h' in argv:
        sys.stdout.write(help_str)
        exit(0)
    if '--version' in argv:
        sys.stdout.write(version_str)
        exit(0)

    analysis_type_map = {'freq_sweep': FreqSweepAnalysis,
                         'balancer': BalancerAnalysis,
                         'test': MockAnalysis,
                         'baseline_comp': EnergyEfficiencyAnalysis}
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter, add_help=False)

    parser.add_argument('-t', '--analysis_type',
                        action='store', required=True, default='REQUIRED OPTION')
    parser.add_argument('-n', '--num_rank',
                        action='store', required=True, default=None, type=int)
    parser.add_argument('-N', '--num_node',
                        action='store', required=True, default=None, type=int)
    parser.add_argument('-o', '--output_dir',
                        action='store', default='.')
    parser.add_argument('-p', '--profile_prefix',
                        action='store', default='')
    parser.add_argument('-l', '--level',
                        action='store', default=1, type=int)
    parser.add_argument('app_argv', metavar='APP_ARGV',
                        action='store', nargs='*')
    parser.add_argument('-s', '--skip_launch',
                        action='store_true', default=False)

    args = parser.parse_args(argv)
    if args.analysis_type not in analysis_type_map:
        raise SyntaxError('Analysis type: "{}" unrecognized.'.format(args.analysis_type))

    if False:
        if not args.profile_prefix:
            raise RuntimeError('profile_prefix is required to skip launch.')

    analysis = analysis_type_map[args.analysis_type](args.profile_prefix,
                                                     args.output_dir,
                                                     args.num_rank,
                                                     args.num_node,
                                                     args.app_argv)

    if args.skip_launch:
        analysis.find_files()
    else:
        analysis.launch()

    if args.level > 0:
        parse_output = analysis.parse()
        process_output = analysis.report_process(parse_output)
        analysis.report(process_output)
        if args.level > 1:
            process_output = analysis.plot_process(parse_output)
            analysis.plot(process_output)
