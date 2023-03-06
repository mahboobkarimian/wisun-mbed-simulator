#!/bin/bash

# Number of nodes (Number 0 is reserved for BR)
let NODES=2
DIR=$(pwd)
DIR_G="/home/wsbr_mounted"


ihex=$(printf '0x%02X' $1) # we get the value of i in hex and we will use the 2 chars after 0x
./wshwsim -m 01:02:03:04:05:06:00:${ihex: -2} "/tmp/uart$1" /tmp/sim_socket
#for (( i=0; i<=$NODES; i++ ))
#do
#	ihex=$(printf '0x%02X' $i) # we get the value of i in hex and we will use the 2 chars after 0x
#	./wshwsim -m 01:02:03:04:05:06:00:${ihex: -2} "/tmp/uart$i" /tmp/sim_socket
#	sleep 0.05
#done