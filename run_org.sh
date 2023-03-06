#!/bin/bash

# Number of nodes (Number 0 is reserved for BR)
let NODES=1

DIR=$(pwd)

# Clean up temp files
rm -rf /tmp/wsbrd/
rm -f /tmp/sim_socket /tmp/*_pae_*
mkdir -p /tmp/wsbrd/

# Compile with latest modifications
ninja

# Create a line topology (-g 0-1 -g 1-2 -g 2-3 -g ......)
TPG=""
for (( c=1; c<=$NODES; c++ ))
do
	TPG+="-g $(($c-1))-$c "
done

# Run server and apply the given topology $TPG
gnome-terminal --tab -- $DIR/wssimserver $TPG /tmp/sim_socket --dump
sleep 0.5

# Create 10 mac/phy (node 0 is BR, 1-9 are Router nodes)
for (( i=0; i<=$NODES; i++ ))
do
	ihex=$(printf '0x%02X' $i) # we get the value of i in hex string format and we will use the 2 chars after 0x
	# Create a new terminal for each node, set core limit to unlimited and run the node
	gnome-terminal --tab --title "MAC_N $i" --  bash -c "ulimit -c unlimited; $DIR/wshwsim -m 01:02:03:04:05:06:00:${ihex: -2} "/tmp/uart$i" /tmp/sim_socket; exec bash"
	sleep 0.1
done

# Get the PID of the running wshwsim processes
PID=$(ps -ef | grep wshwsim | grep -v grep | awk '{print $2}')
# Iterate over the PIDs and start GDB
for i in $PID
do
	#gnome-terminal --tab --title "GDB" --  bash -c "sudo gdb -p $i; exec bash"
	sleep 0.1
done

# Create Router nodes 1-9, for BR we need sudo, so it comes later with sudo
for (( i=1; i<=$NODES; i++ ))
do
	gnome-terminal --window --title "N $i"  -- $DIR/wsnode -F $DIR/examples/wsnode.conf -u $(readlink "/tmp/uart$i") -o storage_prefix=/tmp/n${i}_
done

# Run BR node
gnome-terminal --window --title "BR N0" -- $DIR/wsbrd -F $DIR/examples/wsbrd.conf -u $(readlink /tmp/uart0)




# Not C stayle for loop
#NODES=9
#NS=$(seq 0 $NODES)
#echo $NS
#
#for i in $NS
#do
#	echo $i
#done
