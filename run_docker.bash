#!/bin/bash
### Copyright HEIG-VD IICT 2022
### mahboob.karimian@heig-vd.ch

# Number of nodes (Number 0 is reserved for BR)
let NODES=24

#BROADCAST_INTV=(510 765 1020); UDI=255; BDI=255
BROADCAST_INTV=(200 400 800); UDI=100; BDI=120
MUL_IMIN=2
# Window mode "it", Silent mode "td":
MODE="td"

DIR=$(pwd)
DIR_G="/home/wsbr_mounted"

for idx_bi in ${BROADCAST_INTV[@]}; do

	sed -i "s/broadcast_interval =.*/broadcast_interval =$idx_bi/g" $DIR/examples/wsbrd.conf
	sed -i "s/unicast_dwell_interval =.*/unicast_dwell_interval =$UDI/g" $DIR/examples/wsbrd.conf
	sed -i "s/broadcast_dwell_interval =.*/broadcast_dwell_interval =$BDI/g" $DIR/examples/wsbrd.conf
	sed -i "s/multicast_set_parameters(.*/multicast_set_parameters($MUL_IMIN,0,1,1,75);/g" $DIR/app_wsbrd/wsrouter.c
	sed -i "s/multicast_set_parameters(.*/multicast_set_parameters($MUL_IMIN,0,1,1,75);/g" $DIR/app_wsbrd/wsbr.c
	
	ninja

	params="${UDI}_${BDI}_${idx_bi}_${MUL_IMIN}_1_line_24_5_1"

	DIR_S="$DIR_G/docker_res/$params"
	mkdir -p $DIR/docker_res/$params
	chmod -R 777 $DIR/docker_res/$params

	# Delete the old container if running with the same params
	docker container rm -f "sim_${params}"
	# Create a docker container and mount the pwd
	docker run -d --entrypoint bash --cap-add=NET_ADMIN --privileged -v $DIR:$DIR_G --name "sim_${params}" -it mbedsim:latest

	# Cleanup the old temp files
	docker exec -td "sim_${params}" bash -c "rm -f $DIR_S/n*_*"

	# Create a line topology (-g 0-1 -g 1-2 -g 2-3 -g ......)
	TPG=""
	for ((c = 1; c <= $NODES; c++)); do
		TPG+="-g $(($c - 1))-$c "
	done

	# Run server and apply the given topology $TPG
	gnome-terminal --window --title "SIMSVR $params" -- docker exec -$MODE "sim_${params}" bash -c "$DIR_G/wssimserver $TPG /tmp/sim_socket --dump"
	sleep 0.1

	# Create 10 mac/phy (node 0 is BR, 1-9 are Router nodes)
	for ((i = 0; i <= $NODES; i++)); do
		ihex=$(printf '0x%02X' $i) # we get the value of i in hex and we will use the 2 chars after 0x
		gnome-terminal --window --title "MAC_N $i $params" -- docker exec -$MODE -w $DIR_G "sim_${params}" bash -i -c "bash mac_run.sh $i"
		sleep 0.05
	done

	# Create Router nodes 1-9, for BR we need sudo, so it comes later with sudo
	for ((i = 1; i <= $NODES; i++)); do
		gnome-terminal --window --title "N $i $params" -- docker exec -$MODE -w $DIR_G "sim_${params}" bash -i -c "bash node_run.sh $i $params"
	done

	# Run BR node
	gnome-terminal --window --title "BR N0 $params" -- docker exec -$MODE -w $DIR_G "sim_${params}" bash -i -c 'bash br_run.sh'

done
