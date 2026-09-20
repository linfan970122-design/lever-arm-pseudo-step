# Mahalanobis comparison re-run with the measured heading noise (σ_ψ sensitivity)

Produced by `fig/gate_mahal_sigma.py` → `fig/gate_mahal_sigma.csv`,
`fig/gate_mahal_sigma_event1.csv`. The script re-uses `results_numbers.py` verbatim up to
(not including) its §9.5 block, so every label, denominator and helper is the same object the
manuscript numbers came from; all `to_csv` calls of the prelude are neutralised, so
`gate_mahal.csv` and every other existing output is untouched. Baseline reproduced exactly:
σ = 0.50°, k = 3 → 19.31 % all / 17.5634 % operational, identical to `gate_mahal.csv`.

Only `r_yaw` changes between rows: `r_yaw = (σ_ψ in rad)²`. σ = 0.50° is the driver's declared
cov[35] (`gate.DEFAULT_R_YAW` = 7.61544e-05 rad²); 0.74° is the measured overall σ_ψ and 1.09°
the greenhouse value, both from §4.5 of the manuscript. Genuine = the 2° gyro-agreement label,
denominators 321,238 (all) and 304,412 (operational). Onsets = the first frame of each ungated
pseudo-step episode ≥ 0.10 m: 15 operational, 21 including the 6 induced. **Frames withheld is
0 in every row by construction** — the filter always publishes its own estimate, it never
withholds — so "availability" cannot be matched on withheld frames; what is matched below is
the rejection (false-gate) rate.

## 1. Same thresholds as §9.5 (k = 3σ and 5σ), three variances

| σ_ψ (°) | k | subset | false-gate % | genuine rejected | onsets rejected | withheld | residual ep. ≥ 0.10 m | ≥ 0.05 m |
|---|---|---|---|---|---|---|---|---|
| 0.50 | 3 | all | 19.3100 | 62,031 | 14 / 21 | 0 | 0 | 10 |
| 0.50 | 3 | operational | 17.5634 | 53,465 | 10 / 15 | 0 | 0 | 10 |
| 0.50 | 5 | all | 14.5618 | 46,778 | 13 / 21 | 0 | 0 | 12 |
| 0.50 | 5 | operational | 13.5504 | 41,249 | 9 / 15 | 0 | 0 | 11 |
| 0.74 | 3 | all | 16.7496 | 53,806 | 13 / 21 | 0 | 0 | 10 |
| 0.74 | 3 | operational | 15.1880 | 46,234 | 9 / 15 | 0 | 0 | 10 |
| 0.74 | 5 | all | 12.0683 | 38,768 | 14 / 21 | 0 | 0 | 11 |
| 0.74 | 5 | operational | 11.1730 | 34,012 | 10 / 15 | 0 | 0 | 10 |
| 1.09 | 3 | all | 13.5840 | 43,637 | 14 / 21 | 0 | 0 | 9 |
| 1.09 | 3 | operational | 12.6956 | 38,647 | 9 / 15 | 0 | 0 | 9 |
| 1.09 | 5 | all | 11.5419 | 37,077 | 13 / 21 | 0 | 0 | 10 |
| 1.09 | 5 | operational | 9.4405 | 28,738 | 9 / 15 | 0 | 0 | 9 |

6 September showcase event (§9.5 code, unchanged): rejected for 209.74 s / 14.05° / 0.0894 m at
σ = 0.50, k = 3 (the manuscript's "210 s … 0.089 m"); 208.74 s / 0.0941 m at σ = 0.74, k = 3;
identical at σ = 1.09, k = 3.

**Reading.** Raising the variance to the measured value moves the rejection rate from 17.56 %
to 15.19 % (operational, 3σ), and to 12.70 % at the greenhouse σ of 1.09°. The variance
mismatch therefore accounts for about 2.4 of the 17.6 percentage points at σ = 0.74° and 4.9 of
them at σ = 1.09° — the rate stays two orders of magnitude above v3(b)'s 0.041 % and above
every point of the rule-(a) curve (0.20 %, 0.32 %, 0.43 %, 0.84 %, 3.8 %). Onset detection does
not improve with the larger variance (10 → 9 of 15 at 3σ). The reason is structural, not a
tuning error: q_yaw = (0.002 rad/s)² keeps P far below R, so the test is ≈ |ν| > k·σ_ψ, i.e.
2.2° at σ = 0.74°, k = 3, while the innovation carries the filter's own lag as well as the
measurement noise.

## 2. Matched-availability operating point

k swept (bisection, 22 steps) until the rejection rate on genuine frames equals v3(b)'s
0.044 % (all frames) or 0.041 % (operational). Residual episodes are measured in that rule's
own published output, as in §9.5.

| σ_ψ (°) | target | k matched | false-gate % (all / op) | onsets rejected, all (of 21) / op (of 15) | residual ep. ≥ 0.10 m (all / op) | ≥ 0.05 m (all / op) |
|---|---|---|---|---|---|---|
| 0.50 | 0.044 % all | 219.26 | 0.0439 / 0.0457 | 0 / 0 | 2 / 1 | 5 / 4 |
| 0.50 | 0.041 % op | 224.18 | 0.0395 / 0.0411 | 0 / 0 | 2 / 1 | 5 / 4 |
| 0.74 | 0.044 % all | 144.85 | 0.0439 / 0.0457 | 0 / 0 | 0 / 0 | 3 / 2 |
| 0.74 | 0.041 % op | 151.88 | 0.0408 / 0.0424 | 0 / 0 | 0 / 0 | 3 / 2 |
| 1.09 | 0.044 % all | 102.25 | 0.0448 / 0.0466 | 0 / 0 | 0 / 0 | 7 / 6 |
| 1.09 | 0.041 % op | 112.38 | 0.0395 / 0.0411 | 0 / 0 | 0 / 0 | 7 / 6 |

**At matched availability the Mahalanobis gate detects nothing: 0 of 15 operational onsets and
0 of 21 including the induced ones, at all three variances.** The threshold it needs is
k ≈ 102–224 σ, i.e. an effective 3σ allowance of 35.7–37.2° of heading — about fifty times the
measured σ_ψ. At that point nothing around the 6 September event is rejected (0 frames held): the frame after the event is accepted with a
heading error of 67.53° and a lever-arm position error of 0.4064 m, and nothing is flagged.
(The residual-episode count stays near zero not because the fault was caught but because the
low-gain filter publishes a smoothed estimate, so the fault enters the output as a ramp rather
than a step — the same detection-versus-correction distinction the paper makes elsewhere.)

## 3. Caveats

Unchanged from §9.5: this is a 1-D emulation of the `robot_localization` yaw channel — one
scalar state, no cross-covariance with position or velocity, no differential-drive process
model. Only R was varied here; q_yaw was left at the measured gyro drift, and the genuine
label is the same 2° gyro-agreement proxy, with its known limits (§5.1).
