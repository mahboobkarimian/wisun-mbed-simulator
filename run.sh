#!/bin/bash

# Number of nodes (Number 0 is reserved for BR)
#TYPE="debug" # Debug with few node
#TYPE="test" # Testbench topology
TYPE="dag_simple" # Run full topology
if [ "$TYPE" = "debug" ];then
	let NODES=1
elif [ "$TYPE" = "single" ]; then
	let NODES=1
elif [ "$TYPE" = "dag_simple" ]; then
	let NODES=14
elif [ "$TYPE" = "normal" ]; then
	let NODES=15
else
	echo "Unsupported"
	sleep 2
	exit -1
fi

DIR=$(pwd)

# Clean up temp files
sudo rm -rf /tmp/wsbrd/
rm -f /tmp/sim_socket /tmp/*_pae_*
mkdir -p /tmp/wsbrd/

# Compile with latest modifications
ninja

# Creating D-Bus rule file for wsbrd which is necessary when it is executed as root
WSBRD_DBUS_CONF_FILE=/etc/dbus-1/system.d/com.silabs.Wisun.BorderRouter.conf

WSBRD_DBUS_CONF_FILE_CONTENT=$(cat <<EOF
<!DOCTYPE busconfig PUBLIC "-//freedesktop//DTD D-Bus Bus Configuration 1.0//EN"
"http://www.freedesktop.org/standards/dbus/1.0/busconfig.dtd">
<busconfig>
  <policy context="default">
    <allow own="com.silabs.Wisun.BorderRouter"/>
    <allow send_destination="com.silabs.Wisun.BorderRouter"/>
    <allow receive_sender="com.silabs.Wisun.BorderRouter"/>
    <allow send_interface="com.silabs.Wisun.BorderRouter"/>
    <allow receive_interface="com.silabs.Wisun.BorderRouter"/>
    <allow send_interface="org.freedesktop.DBus.Introspectable"/>
    <allow send_interface="org.freedesktop.DBus.Properties"/>
    <allow receive_interface="org.freedesktop.DBus.Introspectable"/>
    <allow receive_interface="org.freedesktop.DBus.Properties"/>
  </policy>
</busconfig>
EOF
)

if [ ! -f "$WSBRD_DBUS_CONF_FILE" ]; then
	echo "Writing WSBRD D-Bus configuration file"
	echo $WSBRD_DBUS_CONF_FILE_CONTENT | sudo tee -a $WSBRD_DBUS_CONF_FILE
fi

TPG=""

if [ "$TYPE" = "debug" ];then
	# Create a line topology (-g 0-1 -g 1-2 -g 2-3 -g ......)
	for (( c=1; c<=$NODES; c++ ))
	do
		TPG+="-g $(($c-1))-$c "
	done
elif [ "$TYPE" = "single" ]; then
	TPG+="-g 0,1:-76.0"
elif [ "$TYPE" = "dag_simple" ]; then
	TPG+="-g 0,1:-34.0 -g 0,2:-36.0 -g 0,3:-71.0 -g 2,4:-21.0 -g 2,5:-45.0 -g 4,6:-2.0 -g 0,7:-80.0 -g 1,7:-39.0 -g 4,7:-67.0 -g 0,8:-16.0 -g 3,8:-41.0 -g 5,8:-80.0 -g 7,8:-12.0 -g 3,9:-56.0 -g 4,9:-20.0 -g 7,9:-33.0 -g 8,9:-21.0 -g 7,10:-29.0 -g 8,11:-22.0 -g 9,12:-30.0 -g 9,13:-74.0 -g 10,14:-59.0"
elif [ "$TYPE" = "normal" ]; then
	TPG+="-g 0,1 -g 0,2 -g 2,3 -g 3,4 -g 4,5 -g 5,6 -g 4,7 -g 7,8 -g 0,9 -g 9,10 -g 10,11 -g 11,12 -g 12,13 -g 11,14 -g 14,15"
else
	#TPG+="-g 0-1 -g 0-2 -g 2-3 -g 3-4 -g 4-5 -g 5-6 -g 4-7 -g 7-8 -g 0-9 -g 9-10 -g 10-11 -g 11-12 -g 12-13 -g 11-14 -g 14-15"

	echo "Unsupported"
	sleep 2
	exit -1
fi

# Run server and apply the given topology $TPG
gnome-terminal --tab -- $DIR/wssimserver $TPG /tmp/sim_socket --dump
sleep 0.5

# Create 10 mac/phy (node 0 is BR, 1-9 are Router nodes)
for (( i=0; i<=$NODES; i++ ))
do
	ihex=$(printf '0x%02X' $i) # we get the value of i in hex string format and we will use the 2 chars after 0x
	gnome-terminal --tab --title "MAC_N $i" -- $DIR/wshwsim -m 01:02:03:04:05:06:00:${ihex: -2} "/tmp/uart$i" /tmp/sim_socket
	#sleep 0.1
done

# Create Router nodes 1-9, for BR we need sudo, so it comes later with sudo
for (( i=1; i<=$NODES; i++ ))
do
	gnome-terminal --window --title "N $i"  -- $DIR/wsnode -F $DIR/examples/wsnode.conf -u $(readlink "/tmp/uart$i") -o storage_prefix=/tmp/n${i}_
done

# Run BR node
gnome-terminal --window --title "BR N0" -- sudo $DIR/wsbrd -F $DIR/examples/wsbrd.conf -u $(readlink /tmp/uart0)




# Not C stayle for loop
#NODES=9
#NS=$(seq 0 $NODES)
#echo $NS
#
#for i in $NS
#do
#	echo $i
#done
