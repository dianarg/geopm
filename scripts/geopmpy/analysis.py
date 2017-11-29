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
import geopmpy.io
import geopmpy.launcher

from geopmpy import __version__

import pandas

def sys_freq_avail():
    step_freq = 100e6
    with open("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq") as fid:
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
    def __init__(self, name, output_dir, num_rank, num_node, app_argv):
        self._name = name
        self._output_dir = output_dir
        self._num_rank = num_rank
        self._num_node = num_node
        self._app_argv = app_argv
        self._report_paths = []
        self._trace_paths = []

    def launch(self):
        """Run experiment and set data paths corresponding to output.
        """
        raise NotImplementedError('Analysis base class does not implement the launch method()')

    def set_data_paths(self, report_paths, trace_paths=None):
        if not self._report_paths and not self._trace_paths:
            self._report_paths = report_paths
            self._trace_paths = trace_paths
        else:
            raise RuntimeError('Analysis object already has report and trace paths populated.')

    def parse(self):
        return geopmpy.io.AppOutput(self._report_paths, self._trace_paths)

    def plot_process(self, parse_output):
        raise NotImplementedError('Analysis base class does not implement the plot_process method()')

    def report_process(self, parse_output):
        raise NotImplementedError('Analysis base class does not implement the report_process method()')

    def report(self, process_output):
        raise NotImplementedError('Analysis base class does not implement the report method()')

    def plot(self, process_output):
        raise NotImplementedError('Analysis base class does not implement the plot method()')


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
    def __init__(self, name, output_dir, num_rank, num_node, app_argv):
        super(FreqSweepAnalysis, self).__init__(name, output_dir, num_rank, num_node, app_argv)

    def launch(self, geopm_ctl='process', do_geopm_barrier=False):
        ctl_conf = geopmpy.io.CtlConf(self._name + '_app.config',
                                      'dynamic',
                                      {'power_budget':400,
                                       'tree_decider':'static_policy',
                                       'leaf_decider':'simple_freq',
                                       'platform':'rapl'})
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
        perf_margin = 0.1
        optimal_freq = dict()
        min_runtime = dict()
        is_once = True

        profile_name_list = report_df.index.get_level_values('name').unique().tolist()

        freq_list = [float(pn.split('_freq_')[-1].split('_')[0])
                     for pn in profile_name_list
                     if '_freq_' in pn]
        freq_pname = zip(freq_list, profile_name_list)
        freq_pname.sort(reverse=True)

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
                elif min_runtime[region] * (1.0 + perf_margin) > runtime:
                    optimal_freq[region] = freq
            is_once = False
        return optimal_freq

    def _region_freq_str(self, region_freq_map):
        result = ['{}:{}'.format(key, value)
                  for (key, value) in region_freq_map.iteritems()]
        result = ','.join(result)
        return result

def main(argv):
    help_str ="""
Usage: {argv_0} [-h|--help] [--version]
       {argv_0} -t ANALYSIS_TYPE -n NUM_RANK -N NUM_NODE [-o OUTPUT_DIR] [-p PROFILE_PREFIX] [-l] -- EXEC [EXEC_ARGS]

geopmanalysis - Used to run applications and analyze results for specific
                GEOPM use cases.

  -h, --help            show this help message and exit

  -t, --analysis_type   type of analysis to perform. Available
                        ANALYSIS_TYPE values: freq_sweep, and balancer.
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

    analysis_type_map = {'freq_sweep':FreqSweepAnalysis,
                         'balancer':BalancerAnalysis,
                         'test':MockAnalysis}
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
    analysis = analysis_type_map[args.analysis_type](args.profile_prefix, args.output_dir, args.num_rank, args.num_node, args.app_argv)
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
