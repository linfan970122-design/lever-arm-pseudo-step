# Mahalanobis (1-D innovation) gate: sweep over yaw process noise q (0920)

Producer `fig/gate_mahal_q_sweep.py` (wraps results_numbers.py §9.5 / gate.gate_series_mahal; gate.py untouched). Grid: q = default × {0.01, 0.1, 1, 10, 100, 1000} × σ_ψ {0.74°, 1.09°} × k {3, 5}, plus the two q values that match v3(b)'s 0.044 % false-gate rate at σ = 0.74°. Denominators as in results_numbers.md §7 (genuine 321,238 all / 304,412 operational). Full table: `gate_mahal_q_sweep.csv`.

Key rows (operational class, σ = 0.74°):
| q / default | k | false-gate % | onsets held /15 | residual episodes ≥0.10 m |
|---|---|---|---|---|
| 1 | 3 | 15.19 | 9 | 0 |
| 1 | 5 | 11.17 | 10 | 0 |
| 100 | 5 | 3.19 | 8 | 10 |
| 1000 | 3 | 1.03 | 8 | 21 |
| 1000 | 5 | 0.32 | 12 | 19 |
| 38 521 (matched 0.044 %) | 3 | 0.040 | 8 | 16 |
| 13 828 (matched 0.044 %) | 5 | 0.040 | 8 | 16 |

Conclusion: no (q, σ, k) reaches 14/15 operational onsets at ≤ 1 % false-gate rate; raising q buys availability but lets the outliers into the state (residual episodes rise from 0 to 16–21). At matched availability the Mahalanobis gate holds 8/15 onsets against rule (b)'s 14/15 with a comparable residual count (16 vs 15). The earlier sentence "not a tuning artefact" now rests on both σ and q having been swept.
