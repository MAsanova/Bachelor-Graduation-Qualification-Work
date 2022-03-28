#!/bin/sh

./waf --run "scratch/control-network-example \
 --nWifis=10 \
 --timeStep=0.01 \
 --distStep=50 \
 --totalTime=50 \
 --protocol=2 \
" 2>&1 | tee log_function

