#!/bin/bash

############################
# 
# Script to initialize all programs necessary to work with multiple UAVs with AirSim, PX4 and QGroundControl

#  A MEDIASSSSSSSSSS
# Specifically, it opens, per UAV:
# - a PX4 instance with a specific UDP port
# it saves logs in a custom folder
# 
# author: damigo@inf.uc3m.es
############################

# number of UAVs to generate
if [ "$#" -eq  "0" ]
then
    echo "No arguments supplied"
    echo "No args, default 2 UAVs"
    sitl_num=2
else
    echo "Argument: $1"
    sitl_num=$1
fi
echo "Initializing $sitl_num PX4s"

# base ports to use
udpPortBase=14550
otherPortBase=50050
export PX4_SIM_MODEL=iris # without this line it crashes

# create path variables
src_path="$HOME/PX4-Autopilot"							# PX4 location
build_path=${src_path}/build/px4_sitl_default			# build/px4_sitl_default location

# make sure there are no existing instances
echo "killing running instances"
pkill -x px4 || true										# Close other PX4's (if any)
killall mavsdk_server										# Close all MavSDK servers (if any)
rm -fr /tmp/px4*											# Remove temporal PX4 locks (if any)
rm -fr ~/PX4-Autopilot/build/px4_sitl_default/instance_*/	# Remove all temporal log folders (one per UAV, if any)

# create 1 UAV per loop
n=1
sitl_num=$(($n + $sitl_num))	# plus one so it loops 2 times: 1, 2
while [ $n -lt $sitl_num ]; do
	echo "*******************************************************************************"

	# Create a temporal log folder (one per UAV)
	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	# create the port variables to use
	udpPort=$(($udpPortBase))
	otherPort=$(($otherPortBase))

	# Modify Mavlink default UDP port
	echo "Modify Mavlink default UDP port to $udpPort"
	python3 $HOME/AirSim/run/wsl2ModifyUDPport.py "$udpPort"

	# Recompile PX4 (with modified Mavlink)
	echo "Recompile PX4 (with modified Mavlink)"
	cd ~/PX4-Autopilot/
	make px4_sitl_default # none_iris  # Compile PX4 but doesn't run it

	# Run PX4
	echo "Run PX4 number $n"
	pushd "$working_dir" &>/dev/null
	../bin/px4 -i $n -d "$build_path/etc" -s etc/init.d-posix/rcS >out.log 2>err.log &
	popd &>/dev/null

	# loops for the next iteration
	n=$(($n + 1))
done