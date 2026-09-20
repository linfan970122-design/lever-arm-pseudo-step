# lever-arm-pseudo-step

Code and derived results for the manuscript

> **Heading outliers become position steps: the lever-arm pseudo-step in dual-antenna RTK
> and where to guard against it**
> Ke Sheng*, Jianxi Zhu, Tao Zheng*, Zhenlei Chen — Jinhua Academy of Agricultural Sciences, Jinhua, Zhejiang, China (*corresponding authors)

A dual-antenna RTK receiver reports the position of one antenna and the heading of the
baseline between the two. The point a vehicle controller steers is neither of those: it is
the antenna position minus a lever arm **L** rotated by the heading. Every position quality
field — solution type, satellite count, correction age, covariance — describes the antenna
alone, so a heading that is present, fresh and wrong enters that conversion with all fields
nominal and moves the published position by `2|L| sin(|δ|/2)` while the antenna position and
its covariance do not change. This repository holds the reference implementation of the
heading-continuity gate proposed against that fault (`fig/gate.py`), the patch that releases
it into `robot_localization`, the replay harnesses that measure what it costs in that stack
(`analysis/m8/`, `analysis/m8b/`, `analysis/m8b_post/`), the lidar-ICP independent rotation
reference (`analysis/lidar_icp_yaw*.py`, `analysis/align_lidar_events.py`), and the scripts
that produce every number and figure in the paper.

**The recordings are not in this repository** — see *Data not included* below.

## Layout

```
fig/        gate.py           reference implementation of the gate (Python twin of the patch)
            motion_class.py   IMU-only motion labelling, never uses the RTK heading
            theory_numbers.py every number quoted in the theory sections
            results_numbers.py every number quoted in the results sections
            gate_mahal_sigma.py  1-D Mahalanobis comparison sweep (declared vs measured variance)
            gate_mahal_q_sweep.py  1-D Mahalanobis sweep over the filter's yaw process noise
            fill_tables.py    fills the manuscript's Tables 3 and 4 from results_numbers.csv
            fig1..fig8_*.py   Figures 1-8
            roc_position_tests.py  ROC of the two deployed position-domain tests.
                              Written but NOT run: it needs the command topics and the
                              deployed guard source, neither of which is distributed.
            *.csv, *.md       the derived results those scripts wrote (committed)

data/       extract_p2.py     per-frame extraction from a rosbag (the only script that
                              reads a recording)
            post_summary.py   per-bag summary + pseudo-step counts
            sanity.py         spot checks on the extracted frames
            summary.csv       per-bag summary for all recordings (committed)
            lidar_icp/        lidar ICP results (committed): summary.md, events_lidar.csv,
                              above_floor.md
            m8_replay/        M8 design note and results (committed)
            m8b_measurement_gate/  M8b design note and results (committed)

analysis/   lidar_icp_yaw.py         frame-to-frame lidar ICP yaw increment (whole bag)
            lidar_icp_yaw_windows.py the same, restricted to event windows (what was run)
            lidar_heading_ref.py     tree-row heading reference (orchard only; superseded by ICP)
            align_lidar_events.py    aligns ICP yaw with the RTK events; writes Fig. 9
            frame_intervals.py       published-frame interval statistics (Section 4.6) -> data/frame_intervals.md
            sign_check_0906.py       sign convention of eq. (2)-(3) checked on the 6 September events -> data/sign_check_0906.md
            above_floor_check.py     per-event table of every event above the ICP noise floor
            m8/                      robot_localization replay, node-side gate
            m8b/                     the same rule moved to the measurement side
            m8b_post/                whole-run cost analysis and Figure 10

patch/      heading_gate.patch      the patch to robot_localization
            heading_gate_note.md    what the patch does, rule by rule
```

## Requirements

* Python 3.8+ (developed on 3.12) with `numpy`, `pandas`, `matplotlib` — see
  `requirements.txt`. This is enough for every stage that does **not** read a recording.
* `open3d` 0.13 — only for `analysis/lidar_icp_yaw*.py`.
* ROS Noetic (`rospy`, `rosbag`) — only for `data/extract_p2.py`, the lidar scripts, and the
  replay harnesses.
* `robot_localization`, branch `noetic-devel`, commit **040dd17** — only for M8 / M8b.
* `livox_ros_driver2` message definitions — only to read `/livox/lidar` from a recording.

No script in this repository writes outside its own tree except where an environment
variable says so (`P2_OUT`, `P2_WORK`). None of them needs the network.

## Running each stage

Paths below are relative to the repository root. Every script locates its inputs from its
own location, so run them from anywhere.

### 1. Extraction → numbers → figures

```bash
# (a) one CSV per recording, one row per 5 Hz /rtk_odom frame. Needs ROS + the bags.
P2_OUT=out python3 data/extract_p2.py /path/to/run_*.bag
P2_OUT=out python3 data/post_summary.py        # adds gyro bounds and pseudo-step counts
P2_OUT=out python3 data/sanity.py              # spot checks

# (b) every number in the paper. Reads data/csv/*.csv (from step a) and data/summary.csv.
python3 fig/theory_numbers.py                  # -> fig/theory_numbers.csv
python3 fig/results_numbers.py                 # -> fig/results_numbers.{csv,md} + the
                                               #    gate_sweep / gate_mahal / pseudo_steps /
                                               #    gated_residuals / hold_errors CSVs
python3 fig/gate_mahal_sigma.py                # -> fig/gate_mahal_sigma*.csv
python3 fig/fill_tables.py                     # fills the manuscript tables

# (c) figures 1-8 (each writes its own PDF + PNG into fig/)
for f in fig/fig[1-8]_*.py; do python3 "$f"; done
```

`data/extract_p2.py` reads the ENU origin the driver used from `$P2_ORIGIN_FILE`
(default `~/.ros/rtk_origin.yaml`). That file is **not** in this repository.

`fig/gate.py` is importable on its own and runs its self-tests with `python3 fig/gate.py`.

### 2. Gate replay (heading stream only, no ROS)

`fig/gate.py` is the reference implementation: one function per rule version (v1, v2, v3),
fed the recorded heading, the gyro increment and the frame time. `fig/results_numbers.py`
sweeps it over the whole corpus and writes `fig/gate_sweep*.csv` (cost against budget) and
`fig/gate_split.csv` / `fig/gated_residuals.csv` (what each rule leaves behind). Figures 6
and 7 plot those files.

### 3. `robot_localization` replay — M8 (node-side gate)

Two catkin workspaces built from the same upstream commit:

```bash
# ws_stock : robot_localization @ 040dd17, unmodified
# ws_gate  : robot_localization @ 040dd17 + patch/heading_gate.patch
git clone https://github.com/cra-ros-pkg/robot_localization
git -C robot_localization checkout 040dd17
git -C robot_localization apply /path/to/patch/heading_gate.patch    # ws_gate only
```

Then, with `$P2_WORK` pointing at a scratch directory that holds both workspaces, the
slimmed bags, `m8.launch`, `run_one.sh`, `gate.py` and the event list:

```bash
export P2_WORK=/somewhere/p2_m8
python3 analysis/m8/gen_events.py               # -> data/m8_replay/events.csv (45 injections)
bash    analysis/m8/run_all.sh batch1           # the run matrix, one ROS master per arm
python3 analysis/m8/analyze_m8.py               # -> data/m8_replay/per_event.csv
python3 analysis/m8/natural_events.py           # -> data/m8_replay/natural_events.csv
python3 analysis/m8/gate_equiv.py               # C++/Python equivalence check
python3 analysis/m8/summarize_m8.py             # -> data/m8_replay/summary.md
```

`analysis/m8/republish.py` turns the recording back into the three streams the stack
expects, `recorder.py` subscribes to raw messages (never parsed text), and `m8.launch`
wires `ekf_localization_node` + `navsat_transform_node`. `data/m8_replay/design.md` is the
design note, written before the runs, including the replay-rate verification.

### 4. `robot_localization` replay — M8b (measurement-side gate)

Same workspaces, same bags, same event list; `analysis/m8b/m8b_gate_node.py` loads
`fig/gate.py` **verbatim** (from `$M8B_GATE_DIR`, default `$P2_WORK`) and gates the heading
stream before it reaches the EKF.

```bash
bash    analysis/m8b/run_all_m8b.sh batch1
python3 analysis/m8b/analyze_m8b.py
python3 analysis/m8b/natural_events_m8b.py
python3 analysis/m8b/equiv_m8b.py
python3 analysis/m8b/baseline_cost_m8b.py
python3 analysis/m8b/verify_rate_m8b.py
python3 analysis/m8b/summarize_m8b.py           # -> data/m8b_measurement_gate/summary.md
```

### 5. Whole-run cost (M8b post-review)

Reads the per-run CSVs left in `data/m8*/runs/` by stages 3 and 4 (not distributed):

```bash
python3 analysis/m8b_post/full_cost.py          # every >0.10 m excursion, no exclusions
python3 analysis/m8b_post/episode_1527_1550.py  # frame-by-frame dissection of the worst hold
python3 analysis/m8b_post/write_full_cost_md.py # -> data/m8b_measurement_gate/full_cost.md
python3 analysis/m8b_post/fig10_hold_in_turn.py # -> Figure 10
```

### 6. Lidar ICP independent reference

```bash
# on a machine with ROS Noetic, livox_ros_driver2 and open3d, one bag at a time
OMP_NUM_THREADS=4 python3 analysis/lidar_icp_yaw_windows.py \
    --bag /path/to/run_*.bag --out OUTDIR/run_xxx --events fig/pseudo_steps.csv
# -> OUTDIR/run_xxx_{lidar,rtk,imu}.csv ; copy these into data/lidar_icp/
python3 analysis/align_lidar_events.py          # -> data/lidar_icp/events_lidar.csv,
                                                #    summary.md and Figure 9
python3 analysis/above_floor_check.py           # -> data/lidar_icp/above_floor.md
```

`analysis/lidar_icp_yaw.py` is the whole-bag version (~16 h of wall clock for the corpus);
the windowed version is the one that produced the published numbers. The per-bag
`*_lidar.csv` / `*_rtk.csv` / `*_imu.csv` series are **not** distributed; the aligned result
`data/lidar_icp/events_lidar.csv` is.

## Reproducing the paper's numbers without the recordings

The derived result files are committed, so every number and most figures can be checked
without any bag:

| To check | Read |
|---|---|
| every number quoted in the results sections, with its definition | `fig/results_numbers.csv`, `fig/results_numbers.md` |
| every number quoted in the theory sections | `fig/theory_numbers.csv` |
| the pseudo-step event list (99 frames, per-frame) and its per-bag roll-up | `fig/pseudo_steps.csv`, `fig/pstep_by_bag.csv` |
| per-recording corpus summary (48 recordings) | `data/summary.csv` |
| gate cost against budget, and what each rule leaves behind | `fig/gate_sweep*.csv`, `fig/gate_split.csv`, `fig/gated_residuals.csv`, `fig/gate_misses.csv`, `fig/hold_errors.csv`, `fig/v3a_outages.csv` |
| the 1-D Mahalanobis comparison | `fig/gate_mahal.csv`, `fig/gate_mahal_sigma*.csv`, `fig/gate_mahal_sigma_note.md` |
| lidar ICP vs RTK, per event | `data/lidar_icp/events_lidar.csv`, `data/lidar_icp/summary.md`, `data/lidar_icp/above_floor.md` |
| M8 replay (node-side gate) | `data/m8_replay/design.md`, `summary.md`, `per_event.csv`, `natural_events.csv`, `gate_equivalence.csv`, `events.csv`, `exclude.csv` |
| M8b replay (measurement-side gate) | `data/m8b_measurement_gate/design.md`, `summary.md`, `summary_tables.md`, `per_event.csv`, `natural_events.csv`, `gate_equivalence.csv` |
| whole-run cost of the measurement-side gate | `data/m8b_measurement_gate/full_cost.md`, `full_cost_table.csv`, `full_cost_excursions.csv`, `full_cost_injection.csv`, `episode_1527_1550.csv` |

Figures 1 and 3 are analytic and redraw from `fig/theory_numbers.py` alone. Figures 6 and 7
redraw from the committed gate-sweep CSVs. Figures 2, 4, 5 and 8 also read the per-frame
extracts `data/csv/*.csv`, which are not distributed; Figures 9 and 10 need the lidar and
replay run outputs.

## Data not included

Not in this repository, by design:

* the **recordings** (rosbags): 48 runs, about 20 h of fixed-solution operation, plus the
  point clouds — hundreds of gigabytes;
* the **per-frame extracts** `data/csv/*.csv` written by `data/extract_p2.py`;
* the **per-run replay outputs** `data/m8*/runs/` and the per-bag lidar series
  `data/lidar_icp/*_{lidar,rtk,imu}.csv`;
* the **ENU origin / geodetic datum** of the field sites. No latitude, longitude, easting or
  northing appears anywhere in this repository. All positions in the committed files are
  local ENU offsets in metres, and all times are either seconds from the start of a
  recording or POSIX timestamps.

The recordings are available from the corresponding author on reasonable request.

## License

MIT — see `LICENSE`. `patch/heading_gate.patch` is a diff against `robot_localization`; its
context lines remain under that project's BSD-3-Clause license.
