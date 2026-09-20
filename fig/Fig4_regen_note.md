# Fig. 4 regenerated with the v2 motion class — 2026-09-20

## What was actually wrong

Script: `fig/fig4_increments.py` (the only producer of `Fig4_increments.pdf/.png`).
Class source: **not** the `cls_imu` column of `data/csv/*.csv` — that column is the **v1**
label written by `data/extract_p2.py`. Both `fig4_increments.py` and `fig/results_numbers.py`
ignore it and recompute the label with `motion_class.py` (v2), which is the single definition
of the rule.

🔴 The reviewer's diagnosis is only half right, and the half that is wrong matters:

- The **caption** in `fig/captions.md` was v1 (straight RMS 1.58°, turning 4.03°,
  P(>5°) 0.51 % / 15.95 %, no slow class). That was genuinely stale.
- The **figure** was **not** v1. `pdftotext Fig4_increments.pdf` on the file as it stood
  (now `Fig4_increments_v1cls.pdf`) shows three v2-style classes with legend counts
  `straight 140,175 / turn 29,881 / slow 172,262` — a slow class exists, so it cannot be v1
  (v1 has no slow class and its straight/turn counts are 269,063 / 62,056). Its two real
  defects were (a) the `na` class was never drawn, so the legend summed to 342,318 of
  364,799, and (b) straight and slow were off by 12 and 26 frames from
  `results_numbers.md` (140,175 vs 140,163; 172,262 vs 172,236).

Cause of (b), found by running both orderings on the same data: `results_numbers.py`
applies the usable-increment filter (`hd_ok` on both frames, `dpsi_deg` present, `dt ≤ 0.5 s`)
**first** and classifies the surviving frames; the old `fig4_increments.py` classified **all**
frames and filtered afterwards, so unusable frames still entered the 3 s trailing-window sums
`d_ant_win` / `turn_win` and moved a few frames across the 0.50 m slow boundary. Filename
suffix `_v1cls` was used for the archived files as instructed, but "v1cls" is a misnomer:
those files are v2 classes minus `na`, with the pre-filter window bug.

## What changed in the script

1. Filter first, then classify, then `sort_values(["bag","t"], kind="stable")` — byte-for-byte
   the same order of operations as `results_numbers.py`, so the figure and
   `results_numbers.md` §2 / §9.4 are now the same numbers by construction.
2. Fourth class `na` added to `SETS`, drawn in `#CC79A7` (Okabe–Ito reddish purple, the
   palette the other figures already use), labelled **unclassified** in the legend.
3. Nothing else: same 174 mm double column, same two panels, same style block, same
   pdf + png output names.

## Old vs new (all numbers printed by `fig4_increments.py` on this run)

| class | old legend n | new legend n | new RMS (°) | new P(>5°) % | new max (°) | caption v1 RMS (°) | caption v1 P(>5°) % |
|---|---|---|---|---|---|---|---|
| straight | 140,175 | **140,163** | 2.016 | 0.7898 | 178.87 | 1.58 | 0.51 |
| turn | 29,881 | **29,881** | 4.629 | 19.1326 | 168.29 | 4.03 | 15.95 |
| slow | 172,262 | **172,236** | 1.741 | 2.6330 | 167.51 | — | — |
| unclassified (`na`) | not drawn | **22,519** | 1.216 | 1.1945 | 13.82 | — | — |
| all | 364,799 | **364,799** | 2.200 | 3.1875 | 178.87 | — | — |

Exact legend strings now in the figure:
`all (n = 364,799)`, `straight (n = 140,163)`, `turn (n = 29,881)`, `slow (n = 172,236)`,
`unclassified (n = 22,519)`. 140,163 + 29,881 + 172,236 + 22,519 = **364,799**. ✅

## Verified against `results_numbers.md`

Every per-class number above matches §2 and §9.4 to the digits printed there
(straight 140163 / 2.0158 / 0.789795 %, turn 29881 / 4.6290 / 19.132559 %,
slow 172236 / 1.7406 / 2.633 %, na 22519 / 1.2157 / 1.194547 %; all 364799 / 2.1996 /
3.187509 %). The headline overall number is unchanged and re-verified by this run:
**P(|Δψ| > 16.66°) = 0.0606 % = 221 frames of 364,799** (the figure's own annotation is
computed from the plotted array, not typed in). Gyro bounds p99 = 7.57° and max = 16.66°
and Δ_min = 48.5° / 86.3° are read from `results_numbers.csv`, unchanged.

Caption claim "all three tails reach past 100°" checked against the maxima above: straight,
turn and slow all exceed 100°; **unclassified does not** (max 13.82°), which is why the
caption says *three*, not four.

## Not verified / out of scope

- 🔴 `results_numbers.md` §2 is internally inconsistent and was **not** touched: its table
  carries the v2 counts but has **no slow row** (140,163 + 29,881 + 22,519 = 192,563, not
  364,799) and the prose above it still says "`cls_imu` has no *slow* class". The slow row
  only appears in the §9.4 comparison table. Fixing that is a `results_numbers.py` change
  and was outside this task.
- `manuscript_p2_v0.md` was not edited (instructed). Its line ~127 numbers
  (2.02 / 4.63 / 1.74 / 2.20) already agree with what is now plotted.
- Only the Fig. 4 caption was edited in `captions.md`; no other caption touched.
- Nothing under `analysis/m8`, `analysis/m8b`, `data/m8_replay`, `data/m8b_measurement_gate`
  was read or written.
- The corpus header numbers quoted in the caption (47 recordings, 20.3 h) were not
  recomputed here; they come from `results_numbers.md` §1 and are unchanged from the
  previous caption.
