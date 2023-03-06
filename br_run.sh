#!/bin/bash

# Number of nodes (Number 0 is reserved for BR)
let NODES=2
params="255_255_1020_2_1_line_24_5_1"
DIR=$(pwd)
DIR_G="/home/wsbr_mounted"
mkdir -p $DIR/docker_res/$params
mkdir -p /tmp/wsbrd
BRN=0
./wsbrd -F examples/wsbrd.conf -u $(readlink /tmp/uart$BRN)