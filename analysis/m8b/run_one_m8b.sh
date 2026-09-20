#!/bin/bash
# M8b replay, one arm. Everything it writes stays under $P2_WORK (default $TMPDIR/p2_m8).
#   $1 ws (ws_stock|ws_gate)  $2 tag  $3 out_dir  $4 bag  $5 start  $6 dur
#   $7 inject(true|false)  $8 gate(true|false)  $9 max_yaw_rate  $10 mahal  $11 port
#   $12 yaw_pnc  $13 rate  $14 meas_gate(true|false)
set +u
# Scratch workspace for this replay (workspaces, slimmed bags, logs, run outputs).
P2_WORK="${P2_WORK:-${TMPDIR:-/tmp}/p2_m8}"
# ENU origin the driver wrote on the vehicle; never committed to this repository.
P2_ORIGIN_FILE="${P2_ORIGIN_FILE:-$HOME/.ros/rtk_origin.yaml}"
WS=$1; TAG=$2; OUT=$3; BAG=$4; START=$5; DUR=$6; INJ=$7; GATE=$8; MYR=$9; MAHAL=${10}
PORT=${11}; YPNC=${12:-0.06}; RATE=${13:-6.0}; MEAS=${14:-false}

source /opt/ros/noetic/setup.bash
source ${P2_WORK}/$WS/devel/setup.bash
export LD_LIBRARY_PATH=${P2_WORK}/install/lib:${LD_LIBRARY_PATH:-}
export ROS_MASTER_URI=http://127.0.0.1:$PORT
export ROS_HOME=${P2_WORK}/roshome/$TAG          # keep every ROS log out of $HOME
export ROS_LOG_DIR=$ROS_HOME/log
export M8B_GATE_DIR=${P2_WORK}                   # where the verbatim fig/gate.py sits
mkdir -p "$OUT" "$ROS_LOG_DIR"

IMU0=/rtk_heading_imu
if [ "$MEAS" = "true" ]; then IMU0=/rtk_heading_gated; fi

EV=${P2_WORK}/events.csv
# Fixed datum = the ENU origin the driver used. Read from $P2_ORIGIN_FILE at run
# time; the coordinates are not part of this repository.
DLAT=$(awk -F: '/lat/{gsub(/ /,"",$2);print $2}' ${P2_ORIGIN_FILE})
DLON=$(awk -F: '/lon/{gsub(/ /,"",$2);print $2}' ${P2_ORIGIN_FILE})
roslaunch --port $PORT ${P2_WORK}/m8b.launch \
  bag:=$BAG out_dir:=$OUT events_file:=$EV inject:=$INJ \
  rate:=$RATE start:=$START duration:=$DUR \
  gate:=$GATE max_yaw_rate:=$MYR mahal:=$MAHAL yaw_pnc:=$YPNC \
  meas_gate:=$MEAS imu0_topic:=$IMU0 \
  datum_lat:=$DLAT datum_lon:=$DLON \
  > $OUT/roslaunch.log 2>&1
echo "RUN_DONE $TAG rc=$?" >> $OUT/roslaunch.log
