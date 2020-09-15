#!/usr/bin/env python
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
import os
import textwrap
import glob
import yattag
import argparse
import csv
import json


def add_license(doc):
    license_text = textwrap.dedent(''' \
    <!DOCTYPE html>
    <!--
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
      -->
    ''')
    doc.asis(license_text)


def add_head(doc):
    with doc.tag('head'):
        doc.asis('<meta name="viewport" content="width=device-width, initial-scale=1">')
        with doc.tag('style'):
            # TODO: indentation looks funky here
            stylesheet = textwrap.dedent('''
            img {
            display: block;
            margin-left: auto;
            margin-right: auto;
            border:1px solid black;
            }
            h2 {text-align: center; font-family: sans-serif;}
            .note {max-width: 750px; margin: 0 auto;}
            h3 {text-align: center;}
            table, th, td {border:1px solid black; border-collapse: collapse;}
            th, td {padding: 5px;}
            ''')
            doc.asis(stylesheet)


def make_toc(doc, items):
    with doc.tag('div', klass='note'):
        for link_name in items:
            with doc.tag('a', href='#{}'.format(link_name)):
                doc.text(link_name)
            doc.stag('br')


class ResultInfo:
    def __init__(self, path, description):
        self.path = path
        self.description = description


def benchmark_results(doc, bench_name, result_dirs):
    doc.line('h3', bench_name, id=bench_name)
    with doc.tag('div', klass='note'):
        if len(result_dirs) == 0:
            doc.line('h4', 'TODO')
        for result in result_dirs:
            rdir = result.path
            # discover text tables
            # TODO: super brittle
            stat_logs = glob.glob(os.path.join(rdir, '*_stats.log'))
            # discover plots in figures subdir
            images = glob.glob(os.path.join(rdir, 'figures', '*.png'))
            with doc.tag('p'):
                with doc.tag('b'):
                    doc.text('{}: '.format(rdir))
                with doc.tag('a', href=rdir):
                    doc.text('raw data')
            doc.line('p', result.description)
            for log in stat_logs:
                with doc.tag('a', href=log):
                    doc.text(os.path.basename(log))
                doc.stag('br')
            for img in images:
                doc.stag('img', src=img)
            doc.stag('br')


class SmokeStatus:
    def __init__(self, path, run_status, analysis_status):
        self.path = path
        self.run_status = run_status
        self.analysis_status = analysis_status


def weekly_status(doc):
    experiments = ['monitor', 'power_sweep', 'frequency_sweep', 'energy_efficiency']
    apps = ['dgemm_tiny', 'dgemm', 'nekbone', 'minife', 'gadget', 'amg', 'hpcg']
    status = {}
    status_file = os.path.join('smoke', 'status.csv')
    try:
        with open(status_file) as infile:
            reader = csv.DictReader(infile, delimiter=',')
            for row in reader:
                if (row['exp_type'], row['app']) in status:
                    sys.stderr.write('Warning: multiple results for {} with {}'.format(row['exp_type'], row['app']))
                status[(row['exp_type'], row['app'])] = SmokeStatus(run_status=row['run_status'],
                                                                    analysis_status=row['analysis_status'],
                                                                    path=row['path'])
    except Exception as ex:
        sys.stderr.write('Warning: {} missing or wrong format: {}'.format(status_file, ex))

    with doc.tag('div', klass='note'):
        doc.line('h3', 'Integration smoke tests')
        doc.line('p', 'Check that: 1) run scripts produce the expected reports; ' +
                      '2) reports can be used with analysis scripts for the experiment type to produce the expected plots and tables. ' +
                      'No evaluation of performance.')
        # TODO: pandas df to table
        with doc.tag('table'):
            #doc.stag('col')
            # header row: experiment type
            with doc.tag('tr'):
                doc.stag('td', rowspan=2)
                for exp in experiments:
                    doc.line('th', exp, colspan=3)
            # header row: test type
            with doc.tag('tr'):
                for exp in experiments:
                    doc.line('th', 'run script')
                    doc.line('th', 'analysis')
                    doc.line('th', 'raw data')
            for app in apps:
                with doc.tag('tr'):
                    doc.line('td', app)
                    for exp in experiments:
                        if (exp, app) in status:
                            doc.line('td', status[(exp, app)].run_status)
                            doc.line('td', status[(exp, app)].analysis_status)
                            with doc.tag('td'):
                                doc.line('a', 'data', href=status[(exp, app)].path)
                        else:
                            doc.line('td', 'not run', colspan=3, style='text-align: center;')


def discover_experiments():
    summary_file = os.path.join('data', 'summary.json')
    experiments = {}
    try:
        with open(summary_file) as infile:
            summary = json.load(infile)
            for app in summary["apps"]:
                name = app["name"]
                jobs = app["jobs"]
                experiments[name] = []
                for jj in jobs:
                    experiments[name].append(ResultInfo(os.path.join('data', jj["subdir"]),
                                                        jj["description"]))
    except Exception as ex:
        sys.stderr.write('Warning: {} not found or bad format: {}'.format(summary_file, ex))
    return experiments


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true', default=False)
    args = parser.parse_args()

    doc = yattag.Doc()
    add_license(doc)
    with doc.tag('html'):
        add_head(doc)
        with doc.tag('body'):
            doc.line('h2', 'Experiments')
            experiments = discover_experiments()
            make_toc(doc, experiments.keys())
            for name, jobs in experiments.items():
                benchmark_results(doc, name, jobs)

            weekly_status(doc)

    result = yattag.indent(doc.getvalue())
    debug = True
    if args.debug:
        sys.stdout.write(result + '\n')
    with open('index.html', 'w') as outfile:
        outfile.write(result)
