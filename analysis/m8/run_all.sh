#!/bin/bash
# M8 full run matrix. Everything stays under $P2_WORK (default $TMPDIR/p2_m8). See data/m8_replay/design.md.
set +u
# Scratch workspace for this replay (workspaces, slimmed bags, logs, run outputs).
P2_WORK="${P2_WORK:-${TMPDIR:-/tmp}/p2_m8}"
# ENU origin the driver wrote on the vehicle; never committed to this repository.
P2_ORIGIN_FILE="${P2_ORIGIN_FILE:-$HOME/.ros/rtk_origin.yaml}"
BAG9=${P2_WORK}/bag9_slim.bag
BAG6=${P2_WORK}/bag6_slim.bag
R=${P2_WORK}/runs
RATE=${RATE:-6.0}
mkdir -p $R

# The port column below is an arbitrary free local port: each arm gets its own
# ROS master on loopback so the arms can run in parallel.
go(){ # ws tag inject gate myr mahal port pnc bag start dur
  nohup ${MEMGUARD-systemd-run --user --scope -q -p MemoryMax=4G --} \
    bash ${P2_WORK}/run_one.sh $1 $2 $R/$2 $9 ${10} ${11} $3 $4 $5 $6 $7 $8 $RATE \
    >/dev/null 2>&1 &
}

case "$1" in
batch1)   # 0909, injected, 8 arms
  go ws_stock inj_stock_q060  true  false 1.0e9 1.0e12 11410 0.06    $BAG9 0 100000
  go ws_gate  inj_gate_q060   true  true  1.234 1.0e12 11411 0.06    $BAG9 0 100000
  go ws_stock inj_mahal_q060  true  false 1.0e9 3.0    11412 0.06    $BAG9 0 100000
  go ws_stock inj_stock_q1e3  true  false 1.0e9 1.0e12 11413 0.001   $BAG9 0 100000
  go ws_gate  inj_gate_q1e3   true  true  1.234 1.0e12 11414 0.001   $BAG9 0 100000
  go ws_stock inj_stock_q1e4  true  false 1.0e9 1.0e12 11415 0.0001  $BAG9 0 100000
  go ws_gate  inj_gate_q1e4   true  true  1.234 1.0e12 11416 0.0001  $BAG9 0 100000
  go ws_stock inj_mahal_q1e4  true  false 1.0e9 3.0    11417 0.0001  $BAG9 0 100000
  wait; echo BATCH1_DONE ;;
batch2)   # 0909, injected q1e5 + baselines (inject off)
  go ws_stock inj_stock_q1e5  true  false 1.0e9 1.0e12 11420 0.00001 $BAG9 0 100000
  go ws_gate  inj_gate_q1e5   true  true  1.234 1.0e12 11421 0.00001 $BAG9 0 100000
  go ws_stock bas_stock_q060  false false 1.0e9 1.0e12 11422 0.06    $BAG9 0 100000
  go ws_gate  bas_gate_q060   false true  1.234 1.0e12 11423 0.06    $BAG9 0 100000
  go ws_stock bas_mahal_q060  false false 1.0e9 3.0    11424 0.06    $BAG9 0 100000
  go ws_stock bas_stock_q1e4  false false 1.0e9 1.0e12 11425 0.0001  $BAG9 0 100000
  go ws_gate  bas_gate_q1e4   false true  1.234 1.0e12 11426 0.0001  $BAG9 0 100000
  go ws_stock bas_stock_q1e5  false false 1.0e9 1.0e12 11427 0.00001 $BAG9 0 100000
  wait; echo BATCH2_DONE ;;
batch3)   # 0909 remaining baselines + 0906 natural events (no injection)
  go ws_gate  bas_gate_q1e5   false true  1.234 1.0e12 11430 0.00001 $BAG9 0 100000
  go ws_stock bas_stock_q1e3  false false 1.0e9 1.0e12 11431 0.001   $BAG9 0 100000
  go ws_gate  bas_gate_q1e3   false true  1.234 1.0e12 11432 0.001   $BAG9 0 100000
  go ws_stock bas_mahal_q1e4  false false 1.0e9 3.0    11433 0.0001  $BAG9 0 100000
  go ws_stock nat1_stock_q060 false false 1.0e9 1.0e12 11434 0.06    $BAG6 1790 110
  go ws_gate  nat1_gate_q060  false true  1.234 1.0e12 11435 0.06    $BAG6 1790 110
  go ws_stock nat1_mahal_q060 false false 1.0e9 3.0    11436 0.06    $BAG6 1790 110
  go ws_gate  nat1_gate_q1e4  false true  1.234 1.0e12 11437 0.0001  $BAG6 1790 110
  wait; echo BATCH3_DONE ;;
batch4)   # 0906 second window (the -178.9 deg event, bag offset ~510 s) + gate-equivalence reruns
  go ws_stock nat2_stock_q060 false false 1.0e9 1.0e12 11440 0.06    $BAG6 460 110
  go ws_gate  nat2_gate_q060  false true  1.234 1.0e12 11441 0.06    $BAG6 460 110
  go ws_stock nat2_mahal_q060 false false 1.0e9 3.0    11442 0.06    $BAG6 460 110
  go ws_stock nat1_stock_q1e4 false false 1.0e9 1.0e12 11443 0.0001  $BAG6 1790 110
  go ws_gate  inj_gate_q060_eq true true  1.234 1.0e12 11444 0.06    $BAG9 0 100000
  go ws_gate  inj_gate_q1e5_eq true true  1.234 1.0e12 11445 0.00001 $BAG9 0 100000
  wait; echo BATCH4_DONE ;;
*) echo "usage: run_all.sh batch1|batch2|batch3|batch4"; exit 1 ;;
esac
