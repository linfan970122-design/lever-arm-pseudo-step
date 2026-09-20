#!/bin/bash
# M8b run matrix. Everything stays under $P2_WORK (default $TMPDIR/p2_m8). See data/m8b_measurement_gate/design.md.
# MEAS = ws_stock + measurement-side gate (node-side gate absent, Mahalanobis off)
# BOTH = ws_gate (node-side gate on) + measurement-side gate
set +u
# Scratch workspace for this replay (workspaces, slimmed bags, logs, run outputs).
P2_WORK="${P2_WORK:-${TMPDIR:-/tmp}/p2_m8}"
# ENU origin the driver wrote on the vehicle; never committed to this repository.
P2_ORIGIN_FILE="${P2_ORIGIN_FILE:-$HOME/.ros/rtk_origin.yaml}"
BAG9=${P2_WORK}/bag9_slim.bag
BAG6=${P2_WORK}/bag6_slim.bag
R=${P2_WORK}/runs_m8b
RATE=${RATE:-6.0}
mkdir -p $R

# The port column below is an arbitrary free local port: each arm gets its own
# ROS master on loopback so the arms can run in parallel.
go(){ # ws tag inject gate myr mahal port pnc bag start dur meas
  nohup ${MEMGUARD-systemd-run --user --scope -q -p MemoryMax=4G --} \
    bash ${P2_WORK}/run_one_m8b.sh $1 $2 $R/$2 $9 ${10} ${11} $3 $4 $5 $6 $7 $8 $RATE ${12} \
    >/dev/null 2>&1 &
}

case "$1" in
smoke)    # 120 s window, one MEAS run, to prove the node and the topic wiring
  go ws_stock smk_meas_q060 true false 1.0e9 1.0e12 11499 0.06 $BAG9 1725 45 true
  wait; echo SMOKE_DONE ;;
b1)       # MEAS-GATE, four q, injected + baseline
  go ws_stock inj_meas_q060 true  false 1.0e9 1.0e12 11450 0.06    $BAG9 0 100000 true
  go ws_stock inj_meas_q1e3 true  false 1.0e9 1.0e12 11451 0.001   $BAG9 0 100000 true
  go ws_stock inj_meas_q1e4 true  false 1.0e9 1.0e12 11452 0.0001  $BAG9 0 100000 true
  go ws_stock inj_meas_q1e5 true  false 1.0e9 1.0e12 11453 0.00001 $BAG9 0 100000 true
  go ws_stock bas_meas_q060 false false 1.0e9 1.0e12 11454 0.06    $BAG9 0 100000 true
  go ws_stock bas_meas_q1e3 false false 1.0e9 1.0e12 11455 0.001   $BAG9 0 100000 true
  go ws_stock bas_meas_q1e4 false false 1.0e9 1.0e12 11456 0.0001  $BAG9 0 100000 true
  go ws_stock bas_meas_q1e5 false false 1.0e9 1.0e12 11457 0.00001 $BAG9 0 100000 true
  wait; echo B1_DONE ;;
b2)       # MEAS + node-side gate, four q, injected + baseline
  go ws_gate  inj_both_q060 true  true 1.234 1.0e12 11460 0.06    $BAG9 0 100000 true
  go ws_gate  inj_both_q1e3 true  true 1.234 1.0e12 11461 0.001   $BAG9 0 100000 true
  go ws_gate  inj_both_q1e4 true  true 1.234 1.0e12 11462 0.0001  $BAG9 0 100000 true
  go ws_gate  inj_both_q1e5 true  true 1.234 1.0e12 11463 0.00001 $BAG9 0 100000 true
  go ws_gate  bas_both_q060 false true 1.234 1.0e12 11464 0.06    $BAG9 0 100000 true
  go ws_gate  bas_both_q1e3 false true 1.234 1.0e12 11465 0.001   $BAG9 0 100000 true
  go ws_gate  bas_both_q1e4 false true 1.234 1.0e12 11466 0.0001  $BAG9 0 100000 true
  go ws_gate  bas_both_q1e5 false true 1.234 1.0e12 11467 0.00001 $BAG9 0 100000 true
  wait; echo B2_DONE ;;
b3)       # the two recorded 0906 events + the 6x replay-rate verification
  go ws_stock nat1_meas_q060 false false 1.0e9 1.0e12 11470 0.06   $BAG6 1790 110 true
  go ws_stock nat2_meas_q060 false false 1.0e9 1.0e12 11471 0.06   $BAG6 460  110 true
  go ws_stock nat1_meas_q1e4 false false 1.0e9 1.0e12 11472 0.0001 $BAG6 1790 110 true
  go ws_gate  nat1_both_q060 false true  1.234 1.0e12 11473 0.06   $BAG6 1790 110 true
  go ws_gate  nat2_both_q060 false true  1.234 1.0e12 11474 0.06   $BAG6 460  110 true
  go ws_stock ver_meas_6x_a  false false 1.0e9 1.0e12 11475 0.06   $BAG9 1100 120 true
  go ws_stock ver_meas_6x_b  false false 1.0e9 1.0e12 11476 0.06   $BAG9 1100 120 true
  wait; echo B3_DONE ;;
b4)       # the 1x leg of the rate verification (runs alone: 6x of nothing else competing)
  RATE=1.0
  go ws_stock ver_meas_1x    false false 1.0e9 1.0e12 11477 0.06   $BAG9 1100 120 true
  wait; echo B4_DONE ;;
*) echo "usage: run_all_m8b.sh smoke|b1|b2|b3|b4"; exit 1 ;;
esac
