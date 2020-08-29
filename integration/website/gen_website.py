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


class ResultInfo:
    def __init__(self, path, description):
        self.path = path
        self.description = description


def benchmark_results(bench_name, result_dirs):
    doc.line('h3', bench_name)
    for result in result_dirs:
        rdir = result.path
        images = glob.glob(os.path.join(rdir, 'figures', '*.png'))
        with doc.tag('div', klass='note'):
            with doc.tag('p'):
                with doc.tag('b'):
                    doc.text('{}: '.format(rdir))
                with doc.tag('a', href=rdir):
                    doc.text('raw data')
            doc.line('p', result.description)
            for img in images:
                doc.stag('img', src=img)
            doc.stag('br')


class SmokeStatus:
    def __init__(self, path, run_status, analysis_status):
        self.path = path
        self.run_status = run_status
        self.analysis_status = analysis_status


def weekly_status(doc):
    experiments = ['monitor', 'power_sweep', 'frequency_sweep']
    apps = ['dgemm_tiny', 'dgemm', 'nekbone', 'minife', 'gadget', 'amg', 'hpcg']
    # TODO: currently evaluated by hand; load from disk
    status = {
              ('frequency_sweep', 'dgemm_tiny'): SmokeStatus('smoke/49667_dgemm_tiny_frequency_sweep', 'PASS', 'PASS'),
              ('frequency_sweep', 'dgemm'): SmokeStatus('smoke/49667_dgemm_frequency_sweep', 'PASS', 'PASS'),
              ('frequency_sweep', 'minife'): SmokeStatus('smoke/49667_minife_frequency_sweep', 'PASS', 'PASS'),
              ('frequency_sweep', 'nekbone'): SmokeStatus('smoke/49667_nekbone_frequency_sweep', 'PASS', 'PASS'),
              ('monitor', 'dgemm_tiny'): SmokeStatus('smoke/49668_dgemm_tiny_monitor', 'PASS', 'PASS'),
              ('monitor', 'dgemm'): SmokeStatus('smoke/49668_dgemm_monitor', 'PASS', 'PASS'),
              ('monitor', 'minife'): SmokeStatus('smoke/49668_minife_monitor', 'PASS', 'PASS'),
              ('monitor', 'nekbone'): SmokeStatus('smoke/49668_nekbone_monitor', 'PASS', 'PASS'),
              ('power_sweep', 'dgemm_tiny'): SmokeStatus('smoke/49669_dgemm_tiny_power_sweep', 'PASS', 'PASS'),
              ('power_sweep', 'dgemm'): SmokeStatus('smoke/49669_dgemm_power_sweep', 'PASS', 'PASS'),
              ('power_sweep', 'minife'): SmokeStatus('smoke/49669_minife_power_sweep', 'PASS', 'PASS'),
              ('power_sweep', 'nekbone'): SmokeStatus('smoke/49669_nekbone_power_sweep', 'PASS', 'PASS'),
    }
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true', default=False)
    args = parser.parse_args()

    doc = yattag.Doc()
    add_license(doc)
    with doc.tag('html'):
        add_head(doc)
        with doc.tag('body'):
            nekbone_jobs = [ResultInfo(os.path.join('data', '49666_nekbone_energy_efficiency'),
                                       'Nekbone with inserted MPI_Barriers, sweeping over frequency setting for barrier')]
            benchmark_results('Nekbone', nekbone_jobs)
            weekly_status(doc)

    result = yattag.indent(doc.getvalue())
    debug = True
    if args.debug:
        sys.stdout.write(result + '\n')
    with open('index.html', 'w') as outfile:
        outfile.write(result)
