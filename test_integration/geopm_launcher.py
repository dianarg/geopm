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


import sys
import os
import optparse
import socket
import subprocess
import datetime
import signal


def get_resource_manager():
    slurm_hosts = ['mr-fusion', 'KNP12']
    alps_hosts = ['theta']

    result = os.environ.get('GEOPM_RM')

    if not result:
        hostname = socket.gethostname()

        if any(hostname.startswith(word) for word in slurm_hosts):
            result = "SLURM"
        elif any(hostname.startswith(word) for word in alps_hosts):
            result = "ALPS"
        else:
            try:
                exec_str = 'srun --version'
                subprocess.check_call(exec_str, shell=True)
                sys.stderr.write('Warning: Unrecognized host: "{hh}", using SLURM\n'.format(hh=hostname))
                result = "SLURM"
            except subprocess.CalledProcessError:
                try:
                    exec_str = 'aprun --version'
                    subprocess.check_call(exec_str, shell=True)
                    sys.stderr.write("Warning: Unrecognized host: \"{hh}\", using ALPS\n".format(hh=hostname))
                    result = "ALPS"
                except subprocess.CalledProcessError:
                    raise LookupError('Unable to determine resource manager, set GEOPM_RM environment variable to "SLURM" or "ALPS"')

    return result;


def factory(ctl_conf, report_path,
            trace_path=None, host_file=None, time_limit=1):
    resource_manager = get_resource_manager()
    if resource_manager == "SLURM":
        return SrunLauncher(ctl_conf, report_path,
                            trace_path, host_file, time_limit)
    elif resource_manager == "ALPS":
        return AlpsLauncher(ctl_conf, report_path,
                            trace_path, host_file, time_limit)

class Launcher(object):
    def __init__(self, num_rank, num_node, ctl_conf, report_path,
                 trace_path=None, host_file=None, time_limit=None, region_barrier=False):
        self._ctl_conf = ctl_conf
        self._report_path = report_path
        self._trace_path = trace_path
        self._host_file = host_file
        self._time_limit = time_limit
        self._region_barrier = region_barrier
        self._node_list = None
        self._pmpi_ctl = 'process'
        self._default_handler = signal.getsignal(signal.SIGINT)
        self.set_num_rank(num_rank)
        self.set_num_node(num_node)

    def __repr__(self):
        output = ''
        for k,v in self._environ().iteritems():
            output += '{k}={v} '.format(k=k, v=v)
        output += self._exec_str()
        return output

    def __str__(self):
        return self.__repr__()

    def _int_handler(self, signum, frame):
        """
        This is necessary to prevent the script from dying on the first CTRL-C press.  SLURM requires 2
        SIGINT signals to abort the job.
        """
        if type(self) == SrunLauncher:
            print "srun: interrupt (one more within 1 sec to abort)"
        else:
            self._default_handler(signum, frame)

    def set_node_list(self, node_list):
        self._node_list = node_list

    def set_num_node(self, num_node):
        self._num_node = num_node
        self._set_num_thread()

    def set_num_rank(self, num_rank):
        self._num_rank = num_rank
        self._set_num_thread()

    def set_pmpi_ctl(self, pmpi_ctl):
        self._pmpi_ctl = pmpi_ctl

    def check_call(self, exec_str, env_ext={}, stdout=sys.stdout, stderr=sys.stderr):
        env = dict(os.environ)
        env.update(env_ext)
        stdout.write(str(datetime.datetime.now()) + '\n\n' )
        stdout.write(exec_str + '\n\n')
        stdout.flush()
        signal.signal(signal.SIGINT, self._int_handler)
        subprocess.check_call(exec_str, shell=True, env=env, stdout=stdout, stderr=stderr)
        signal.signal(signal.SIGINT, self._default_handler)

    def get_report(self):
        return Report(self._report_path)

    def get_trace(self):
        return Trace(self._trace_path)

    def get_idle_nodes(self):
        return ''

    def get_alloc_nodes(self):
        return ''

    def _set_num_thread(self):
        # Figure out the number of CPUs per rank leaving one for the
        # OS and one (potentially, may/may not be use depending on pmpi_ctl)
        # for the controller.
        cmd = ' '.join((self._exec_option(), 'lscpu'))
        signal.signal(signal.SIGINT, self._int_handler)
        pid = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (out, err) = pid.communicate()
        signal.signal(signal.SIGINT, self._default_handler)
        if pid.returncode:
            raise subprocess.CalledProcessError(pid.returncode, cmd, err)
        core_socket = [int(line.split(':')[1])
                       for line in out.splitlines()
                       if line.find('Core(s) per socket:') == 0 or
                          line.find('Socket(s):') == 0]
        # Mulitply num core per socket by num socket and remove one
        # CPU for BSP and one for the controller to calculate number
        # of CPU for application.  Don't use hyper-threads.
        num_cpu = core_socket[0] * core_socket[1] - 2
        try:
            rank_per_node = self._num_rank / self._num_node
            # Fixes situations when rank_per_node may be 0 which results in a divide by 0 below.
            rank_per_node = rank_per_node if rank_per_node != 0 else 1
            self._num_thread = num_cpu / rank_per_node
        except AttributeError:
            pass

    def _environ(self):
        result = {'LD_DYNAMIC_WEAK': 'true',
                  'OMP_NUM_THREADS' : str(self._num_thread),
                  'GEOPM_PMPI_CTL' : self._pmpi_ctl,
                  'GEOPM_REPORT' : self._report_path,
                  'GEOPM_POLICY' : self._ctl_conf.get_path()}
        if self._trace_path:
            result['GEOPM_TRACE'] = self._trace_path
        if self._region_barrier:
            result['GEOPM_REGION_BARRIER'] = 'true'
        return result

    def _exec_str(self, argv):
        # Save the pmpi_ctl state and unset the value for non-geopom execution
        tmp_pmpi_ctl = self._pmpi_ctl
        self._pmpi_ctl = None
        result = self._geopm_exec_str(argv)
        self._pmpi_ctl = tmp_pmpi_ctl
        return result

    def _geopm_exec_str(self, argv):
        argv_mod = [self._mpiexec_option(),
                    self._num_node_option(),
                    self._num_rank_option(),
                    self._affinity_option(),
                    self._host_option(),
                    self._membind_option()]
        argv_mod.extend(argv)
        return ' '.join(argv_mod)

    def _num_rank_option(self):
        num_rank = self._num_rank
        if self._pmpi_ctl == 'process':
            num_rank += self._num_node
        return '-n {num_rank}'.format(num_rank=num_rank)

    def _affinity_option(self):
        return ''

    def _membind_option(self):
        return ''

    def _host_option(self):
        if self._host_file:
            raise NotImplementedError
        return ''


class SrunLauncher(Launcher):
    def __init__(self, app_conf, ctl_conf, report_path,
                 trace_path=None, host_file=None, time_limit=1):
        self._queuing_timeout = 30
        self._job_name = 'int_test'
        super(SrunLauncher, self).__init__(app_conf, ctl_conf, report_path,
                                           trace_path=trace_path, host_file=host_file,
                                           time_limit=time_limit)

    def _mpiexec_option(self):
        mpiexec = 'srun -K -I{timeout} -J {name}'.format(timeout=self._queuing_timeout, name=self._job_name)
        if self._time_limit is not None:
            mpiexec += ' -t {time_limit}'.format(time_limit=self._time_limit)
        if self._node_list is not None:
            mpiexec += ' -w ' + ','.join(self._node_list)
        return mpiexec

    def _exec_option(self):
        return 'srun -I{timeout} -J {name} -n 1'.format(timeout=self._queuing_timeout, name=self._job_name)

    def _num_node_option(self):
        return '-N {num_node}'.format(num_node=self._num_node)

    def _affinity_option(self):
        proc_mask = self._num_thread * '1' + '00'
        result_base = '--cpu_bind=v,mask_cpu:'
        mask_list = []
        if (self._pmpi_ctl == 'process'):
            mask_list.append('0x2')
        for ii in range(self._num_rank / self._num_node):
            mask_list.append('0x{:x}'.format(int(proc_mask, 2)))
            proc_mask = proc_mask + self._num_thread * '0'
        return result_base + ','.join(mask_list)

    def _host_option(self):
        result = ''
        if self._host_file:
            result = '-w {host_file}'.format(self._host_file)
        return result

    def get_idle_nodes(self):
        return subprocess.check_output('sinfo -t idle -hNo %N', shell=True).splitlines()

    def get_alloc_nodes(self):
        return subprocess.check_output('sinfo -t alloc -hNo %N', shell=True).splitlines()

class AlpsLauncher(Launcher):
    def __init__(self, app_conf, ctl_conf, report_path,
                 trace_path=None, host_file=None, time_limit=1):
        self._queuing_timeout = 30
        self._job_name = 'int_test'
        super(AlpsLauncher, self).__init__(app_conf, ctl_conf, report_path,
                                           trace_path=trace_path, host_file=host_file,
                                           time_limit=time_limit)

    def _environ(self):
        result = super(AlpsLauncher, self)._environ()
        result['KMP_AFFINITY'] = 'disabled'
        return result

    def _mpiexec_option(self):
        mpiexec = 'aprun'
        if self._time_limit is not None:
            mpiexec += ' -t {time_limit}'.format(time_limit=self._time_limit * 60)
        if self._node_list is not None:
            mpiexec += ' -L ' + ','.join(self._node_list)
        return mpiexec

    def _exec_option(self):
        return 'aprun -n 1'

    def _num_node_option(self):
        rank_per_node = self._num_rank / self._num_node
        if self._pmpi_ctl == 'process':
            rank_per_node += 1
        return '-N {rank_per_node}'.format(rank_per_node=rank_per_node)

    def _affinity_option(self):
        result_base = '-cc '
        mask_list = []
        off_start = 1
        if (self._pmpi_ctl == 'process'):
            mask_list.append('1')
            off_start = 2
        rank_per_node = self._num_rank / self._num_node
        thread_per_node = rank_per_node * self._num_thread
        mask_list.extend(['{0}-{1}'.format(off, off + self._num_thread - 1)
                          for off in range(off_start,thread_per_node + off_start, self._num_thread)])
        return result_base + ':'.join(mask_list)

    def _host_option(self):
        result = ''
        if self._host_file:
            result = '--node-list-file {host_file}'.format(self._host_file)
        return result

    def get_idle_nodes(self):
        raise NotImplementedError;

    def get_alloc_nodes(self):
        raise NotImplementedError;


def geopm_srun_affinity(mode, num_rank, num_thread):
    """
    geopm_srun_affinty(mode, num_rank, num_thread)
        mode: One of the following strings
              "process" - geopm controller as an mpi process.
              "pthread" - geopm controller as a posix thread.
              "geopmctl" - for geopmctl application.
              "application" - for main application when launching with geopmctl.
        num_rank: Number of ranks per node used by main application.
        num_thread: Number of threads per rank used by main application.
    """
    result_base = '--cpu_bind=v,mask_cpu:'
    if mode == 'geopmctl':
        result = result_base + '0x1'
    else:
        mask_list = []
        if mode == 'process':
            mask_list.append('0x1')
            binary_mask = num_thread * '1' + '0'
        elif mode == 'pthread':
            binary_mask = (num_thread + 1) * '1'
        elif mode == 'application':
            binary_mask = num_thread * '1' + '0'
        else:
            raise NameError('Unknown mode: "{mode}", valid options are "process", "pthread", "geopmctl", or "application"'.format(mode=mode))
        for ii in range(num_rank):
            hex_mask = '0x{:x}'.format(int(binary_mask, 2))
            mask_list.append(hex_mask)
            if ii == 0 and mode == 'pthread':
                binary_mask = num_thread * '1' + '0'
            binary_mask = binary_mask + num_thread * '0'

        result = result_base + ','.join(mask_list)
    return result

class SubsetOptionParser(optparse.OptionParser):
    def _process_args(self, largs, rargs, values):
        while rargs:
            try:
                optparse.OptionParser._process_args(self, largs, rargs, values)
            except (optparse.BadOptionError, optparse.AmbiguousOptionError) as e:
                largs.append(e.opt_str)

def swap_args(in_args):
    # Parse the subset of arguements used by geopm
    parser = SubsetOptionParser()
    parser.add_option('-n', '--ntasks', dest='num_rank', nargs=1, type='int')
    parser.add_option('-N', '--nodes', dest='num_node', nargs=1, type='int')
    parser.add_option('-c', '--cpus-per-task', dest='cpu_per_rank', nargs=1, type='int')
    parser.add_option('--geopm-ctl', dest='ctl', nargs=1, type='string')
    opts, args = parser.parse_args(in_args)

    if opts.ctl is None:
        args = in_args
        ctl = None
    else:
        # Check required arguements and add a rank per node for the controller
        if opts.num_rank is None:
            raise SyntaxError('Number of tasks must be specified with -n.')
        if opts.num_node is None:
            raise SyntaxError('Number of nodes must be specified with -N.')
        if '--cpu_bind' in args:
            raise SyntaxError('The option --cpu_bind must not be specified, this is controlled by geopm_srun.')
        ctl = opts.ctl
        num_rank = opts.num_rank
        num_node = opts.num_node
        if ctl not in ('process', 'pthread', 'application'):
            raise SyntaxError('--geopm-ctl must be one of: "process", "pthread", or "application"')
        if opts.cpu_per_rank is None:
            cpu_per_rank = int(os.environ.get('OMP_NUM_THREADS', '1'))
        else:
            cpu_per_rank = opts.cpu_per_rank
        if ctl == 'process':
            num_rank += num_node

        # Put back modified -n and -N
        geopm_args = ['-n', str(num_rank), '-N', str(num_node)]
        # Add affinity mask for geopm
        geopm_args.extend(geopm_srun_affinity(ctl, num_rank / num_node, cpu_per_rank).split())
        geopm_args.extend(args)
        args = geopm_args
    # Put srun at head of the arguments
    args.insert(0, 'srun')
    return ctl, args

def main():
    help_msg = """\
GEOPM options:
      --geopm-ctl=ctl         use geopm runtime and launch geopm with the
                              "ctl" method, one of "process", "pthread" or
                              "application"
      --geopm-report=rpt      create geopm report files at path "rpt"
      --geopm-trace=trc       create geopm trace files at path "trc"
      --geopm-region-barrier  enable node local barriers at geopm region entry/exit
"""
    rm = get_resource_manager()
    if rm == 'SLURM':
        arg0 = 'srun'
    elif rm == 'ALPS'
        arg0 = 'aprun'
    
    err = 0
    args = sys.argv[1:]
    # Print launcher help if requested
    if '--help' in args or '-h' in args:
        pid = subprocess.Popen([arg0, '--help'])
        pid.wait()
        sys.stdout.write(help_msg)
        err = pid.returncode
    else:
        launcher = launcher_factory():
  
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
        sys.stderr.write("Error: <geopm> {err}\n".format(err=e))
        err = -1
    sys.exit(err)
