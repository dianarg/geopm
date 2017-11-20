#!/bin/bash

if [ $# -ne 1 ]; then
    echo 'Provide data subfolder path'
    exit 1
fi

cd "$1" && \
for f in $(find . -maxdepth 2 -name "*adaptive*.log"); do echo "$f"; grep '11396693813.*is_new' $f | tail -1; grep '20779751936.*is_new' $f | tail -1; done > adaptive_last_freq.txt && \
for f in $(find . -maxdepth 2 -name "*freq_multi_node.log"); do grep 'Mix ratio\|Frequency map' $f | uniq; done > dynamic_freq.txt && \
cd ..
