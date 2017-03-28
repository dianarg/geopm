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
    geopm_launcher.py [launcher-args...] [--geopm-ctl=<mode>] executable [exec-args...]
        mode: One of the following strings
              "process" - Launch geopm controller as an mpi process.
              "pthread" - Launch geopm controller as a posix thread.
              "application" - Launch geopm controller as a separate MPI application (geopmctl).
"""
import sys
import os
import optparse
import subprocess

class SubsetOptionParser(optparse.OptionParser):
    def _process_args(self, largs, rargs, values):
        while rargs:
            try:
                optparse.OptionParser._process_args(self, largs, rargs, values)
            except (optparse.BadOptionError, optparse.AmbiguousOptionError) as e:
                largs.append(e.opt_str)

class PassThroughError(Exception):
    """
    Exception raised when geopm is not to be used.
    """

class Config(object):
    def __init__(self, argv):
        if '--geopm-ctl' not in argv:
            raise PassThroughError('The --geopm-ctl flag is not specified.')
        # Parse the subset of arguements used by geopm
        parser = SubsetOptionParser()
        parser.add_option('--geopm-ctl', dest='ctl', nargs=1, type='string')
        parser.add_option('--geopm-policy', dest='policy', nargs=1, type='string')
        parser.add_option('--geopm-report', dest='report', nargs=1, type='string')
        parser.add_option('--geopm-trace', dest='trace', nargs=1, type='string')
        parser.add_option('--geopm-shmkey', dest='shmkey', nargs=1, type='string')
        parser.add_option('--geopm-timeout', dest='timeout', nargs=1, type='string')
        parser.add_option('--geopm-plugin', dest='plugin', nargs=1, type='string')
        parser.add_option('--geopm-debug-attach', dest='debug_attach', nargs=1, type='string')
        parser.add_option('--geopm-barrier', dest='barrier', action='store_true', default=False)
        opts, args = parser.parse_args(argv[1:])
        self.ctl = opts.ctl
        self.policy = opts.policy
        self.report = opts.report
        self.trace = opts.trace
        self.shmkey = opts.shmkey
        self.timeout = opts.timeout
        self.plugin = opts.plugin
        self.debug_attach = opts.debug_attach
        self.barrier = opts.barrier
        if self.ctl not in ('process', 'pthread', 'application'):
            raise SyntaxError('--geopm-ctl must be one of: "process", "pthread", or "application"')

class Launcher(object):
    def __init__(self, argv):
        self.argv = argv[1:]
        try:
            self.config = Config(argv)
            self.parse_alloc()
            self.modify_alloc()
        except PassThroughError:
            self.config = None

    def run(self):
        argv_mod = [self.mpiexec()]
        if self.config is not None:
            argv_mod.extend(self.num_node_option())
            argv_mod.extend(self.num_rank_option())
            argv_mod.extend(self.affinity_option())
        argv_mod.extend(self.argv)
        subprocess.check_call(argv_mod, env=self.environ())

    def environ(self):
        result = dict(sys.environ)
        if self.config is not None:
            result['GEOPM_POLICY'] = self.config.policy
            if self.config.report:
                result['GEOPM_REPORT'] = self.config.report
            if self.config.trace:
                result['GEOPM_TRACE'] = self.config.trace
            if self.config.shmkey:
                result['GEOPM_SHMKEY'] = self.config.shmkey
            if self.config.timeout:
                result['GEOPM_PROFILE_TIMEOUT'] = self.config.timeout
            if self.config.plugin:
                result['GEOPM_PLUGIN_PATH'] = self.config.timeout
            if self.config.debug_attach:
                result['GEOPM_DEBUG_ATTACH'] = self.config.debug_attach
            if self.config.barrier:
                result['GEOPM_REGION_BARRIER'] = 'true'
        return result

    def parse_alloc(self):
        raise NotImplementedError('Launcher.parse_alloc() undefined in the base class')

    def mpiexec(self):
        raise NotImplementedError('Launcher.mpiexec() undefined in the base class')

    def num_node_option(self):
        raise NotImplementedError('Launcher.num_node_option() undefined in the base class')

    def num_rank_option(self):
        raise NotImplementedError('Launcher.num_rank_option() undefined in the base class')

    def affinity_option(self):
        raise NotImplementedError('Launcher.affinity_option() undefined in the base class')

class SrunLauncher(Launcher):
    def __init__(self, argv):
        Launcher.__init__(argv)

    def parse_alloc(self):
        # Parse the subset of arguements used by geopm
        parser = SubsetOptionParser()
        parser.add_option('-n', '--ntasks', dest='num_rank', nargs=1, type='int')
        parser.add_option('-N', '--nodes', dest='num_node', nargs=1, type='int')
        parser.add_option('-c', '--cpus-per-task', dest='cpu_per_rank', nargs=1, type='int')
        opts, args = parser.parse_args(self.argv[1:])

        # Check required arguements
        if opts.num_rank is None:
            raise SyntaxError('Number of tasks must be specified with -n.')
        if opts.num_node is None:
            raise SyntaxError('Number of nodes must be specified with -N.')
        if '--cpu_bind' in args:
            raise SyntaxError('The option --cpu_bind must not be specified, this is controlled by geopm_srun.')

        self.argv = args
        self.num_rank = opts.num_rank
        self.num_node = opts.num_node
        rank_per_node = num_rank / num_node
        if opts.cpu_per_rank is None:
            self.cpu_per_rank = int(os.environ.get('OMP_NUM_THREADS', '1'))
        else:
            self.cpu_per_rank = opts.cpu_per_rank
        if self.config.ctl == 'process':
            self.num_rank += num_node

    def num_node_option(self):
        return ['-N', str(self.num_node)]

    def num_rank_option(self):
        return ['-n', str(self.num_rank)]

    def mpiexec_option(self):
        return 'srun'

    def affinity_option(self):
        if self.config.ctl == 'application':
            raise NotImplementedError('Launch with geopmctl not supported')
        result_base = '--cpu_bind=v,mask_cpu:'
        mask_list = []
        if mode == 'process':
            mask_list.append('0x1')
            binary_mask = self.cpu_per_rank * '1' + '0'
        elif mode == 'pthread':
            binary_mask = (self.cpu_per_rank + 1) * '1'
        for ii in range(num_rank):
            hex_mask = '0x{:x}'.format(int(binary_mask, 2))
            mask_list.append(hex_mask)
            if ii == 0 and mode == 'pthread':
                binary_mask = num_thread * '1' + '0'
            binary_mask = binary_mask + num_thread * '0'
        return result_base + ','.join(mask_list)

def srun_main():
    help_msg = """\
GEOPM options:
      --geopm-ctl=ctl         use geopm runtime and launch geopm with the
                              "ctl" method, one of "process", "pthread" or
                              "application"
      --geopm-policy=pol      use the geopm policy file or shared memory
                              region "pol"
      --geopm-report=path     create geopm report files with base name "path"
      --geopm-trace=path      create geopm trace files with base name "path"
      --geopm-shmkey=key      use shared memory keys for geopm starting with
                              "key"
      --geopm-timeout=sec     appliation waits "sec" seconds for handshake
                              with geopm
      --geopm-plugin=path     look for geopm plugins in "path", a : separated
                              list of directories
      --geopm-debug-attach=rk attach serial debugger to rank "rk"
      --geopm-barrier         apply node local barriers when application enters
                              or exits a geopm region

"""
    err = 0
    args = sys.argv[1:]
    # Print srun help if requested
    if '--help' in args or '-h' in args:
        args.insert(0, 'srun')
        pid = subprocess.Popen(args)
        pid.wait()
        sys.stdout.write(help_msg)
        err = pid.returncode
    else:
        ctl, args = swap_args(args)
        if ctl is None:
            pid = subprocess.Popen(args)
            pid.wait()
            err = pid.returncode
        if ctl in ('process', 'pthread'):
            env = os.environ;
            env['GEOPM_PMPI_CTL'] = ctl
            sys.stdout.write('GEOPM_PMPI_CTL={ctl} '.format(ctl=ctl) + ' '.join(args) + '\n')
            pid = subprocess.Popen(args, env=env)
            pid.wait()
            err = pid.returncode
        elif ctl == 'application':
            raise NotImplementedError('Launching geopm as separate MPI application not yet supported by geopm_srun.')
    sys.exit(err)

if __name__ == '__main__':
    try:
        err = main()
    except Exception as e:
        sys.stderr.write("<geopm_srun> {err}\n".format(err=e))
        err = -1
    sys.exit(err)
