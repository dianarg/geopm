#!/usr/bin/python

# For adaptive, parse output from
# for f in $(find . -name "*adaptive*.log"); do echo "$f"; grep '11396693813.*is_new' $f | tail -1; grep '20779751936.*is_new' $f | tail -1; done > adaptive_last_freq.txt
# format:
# ./31099.simple_freq_test_multi_node-1/test_plugin_simple_freq_multi_node_adaptive_6.log
# Region ID: 11396693813 Freq: 1.4e+09 is_new_region: 1
# Region ID: 20779751936 Freq: 1.2e+09 is_new_region: 1
#
# For per-region map, parse output from
# for f in $(find . -name "*freq_multi_node.log"); do grep 'Mix ratio\|Frequency map' $f | uniq; done > dynamic_freq.txt
# format:
# Mix ratio index 5
# Frequency map: stream:1200000000.0,dgemm:2100000000.0
# Mix ratio index 6
# Frequency map: stream:1200000000.0,dgemm:2100000000.0


from collections import defaultdict
import sys


def print_csv(dgemm_freq, stream_freq):
    # ratios obtained from top level parsing script
    pct_ratios = {0.55500543152056236: 6,
                  0.74201238229007493: 4,
                  0.31292686661286107: 1,
                  0.48814404837075598: 3,
                  0.40356655606121739: 2,
                  0.18675584827266081: 0,
                  0.64043599149308772: 5}

    pcts = sorted(pct_ratios.keys())
    rv = 'dgemm\n'
    for p in pcts:
        rv += str(p)+','+','.join(dgemm_freq[pct_ratios[p]])+'\n'

    rv += 'stream\n'
    for p in pcts:
        rv += str(p)+','+','.join(stream_freq[pct_ratios[p]])+'\n'
    return rv


def adaptive_freq(data_folder):
    dgemm_freq = defaultdict(list)
    stream_freq = defaultdict(list)

    filename = data_folder + "/adaptive_last_freq.txt"
    with open(filename) as f:
        for line in f:
            if '.log' in line:
                mix = int(line[-6])
            elif '11396693813' in line:
                freq = line.split()[6]
                dgemm_freq[mix].append(freq)
            elif '20779751936' in line:
                freq = line.split()[6]
                stream_freq[mix].append(freq)

    rv = 'Adaptive\n'
    rv += print_csv(dgemm_freq, stream_freq)
    return rv


def per_region_map(data_folder):
    dgemm_freq = defaultdict(list)
    stream_freq = defaultdict(list)

    filename = data_folder + "/dynamic_freq.txt"
    with open(filename) as f:
        for line in f:
            line = line.strip()
            if 'Mix ratio' in line:
                mix = int(line[-1])
            elif 'Frequency map' in line:
                line = line.split()[-1]
                line = line.split(',')
                stream = line[0].split(':')[1]
                dgemm = line[1].split(':')[1]
                dgemm_freq[mix].append(dgemm)
                stream_freq[mix].append(stream)

    rv = 'Per-region freq map\n'
    rv += print_csv(dgemm_freq, stream_freq)
    return rv


if __name__ == '__main__':
    print adaptive_freq(sys.argv[1])
    print per_region_map(sys.argv[1])
