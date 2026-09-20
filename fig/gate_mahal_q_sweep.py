#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Process-noise (q_yaw) sweep of the §9.5 Mahalanobis comparison.

Same construction as fig/gate_mahal_sigma.py: results_numbers.py is re-used verbatim up to
(not including) its §9.5 block, so every label, denominator and helper is the same object the
manuscript numbers came from; all to_csv calls of the prelude are neutralised first, so no
existing CSV is touched.  gate.py is NOT modified -- q_yaw is already a keyword argument of
G.gate_series_mahal, so the sweep just passes it.

Writes only fig/gate_mahal_q_sweep.csv and fig/gate_mahal_q_sweep_event1.csv.
"""
import os
import sys
import time

import numpy as np
import pandas as pd

P2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # repository root
FIG = os.path.join(P2, "fig")
sys.path.insert(0, FIG)

RN = os.path.join(FIG, "results_numbers.py")
src = open(RN, encoding="utf-8").read()
MARK = "# ---- 9.5 Mahalanobis"
assert MARK in src, "marker not found in results_numbers.py"
head = src.split(MARK)[0]

_real_to_csv = pd.DataFrame.to_csv
pd.DataFrame.to_csv = lambda *a, **k: None      # guard: do not rewrite existing outputs

t0 = time.time()
ns = {"__file__": RN, "__name__": "rn_partial", "__builtins__": __builtins__}
exec(compile(head, RN, "exec"), ns)
print("[prelude done in %.1f s]" % (time.time() - t0), flush=True)

G = ns["G"]
S, N = ns["S"], ns["N"]
bag_groups = ns["bag_groups"]
cum_gyro_yaw = ns["cum_gyro_yaw"]
gen_np, OPS = ns["gen_np"], ns["OPS"]
OP_BAGS = ns["OP_BAGS"]
ONSET = ns["ONSET"]
gated_residuals = ns["gated_residuals"]
wrap180 = ns["wrap180"]
LNORM = ns["LNORM"]
EV_BAG, EV_J0, EV_HDG, EV_CG = ns["EV_BAG"], ns["EV_J0"], ns["EV_HDG"], ns["EV_CG"]

Q0 = G.DEFAULT_Q_YAW
ONS_ALL = np.concatenate([ONSET["operational"], ONSET["induced"]])
print("q0 = %.6g rad^2/s ; genuine all %d, op %d ; onsets op %d all %d"
      % (Q0, int(gen_np.sum()), int((gen_np & OPS).sum()),
         ONSET["operational"].size, ONS_ALL.size), flush=True)


def run(sigma_deg, k, q):
    """One full-corpus pass. Returns (rejected mask, used yaw, event-1 dict)."""
    r_yaw = float(np.radians(sigma_deg)) ** 2
    st = np.zeros(N, np.int8)
    uy = np.zeros(N)
    ev = None
    for b, idx, yaw, ts in bag_groups:
        r_ = G.gate_series_mahal(yaw, ts, cum_gyro_rad=cum_gyro_yaw[idx], k_sigma=k,
                                 q_yaw=q, r_yaw=r_yaw)
        st[idx] = r_["state"]
        uy[idx] = r_["used_yaw"]
        if b == EV_BAG:
            st2 = r_["state"]
            j0 = EV_J0
            k_acc = next((z for z in range(j0 + 1, st2.size) if st2[z] == G.ACCEPT), None)
            if k_acc is not None:
                true_hdg = EV_HDG[j0 - 1] + (EV_CG[k_acc] - EV_CG[j0 - 1])
                derr = float(abs(wrap180(EV_HDG[k_acc] - true_hdg)))
                ev = dict(sigma_deg=sigma_deg, k_sigma=k, q_yaw=q, q_over_q0=q / Q0,
                          rejected_s=round(float(ts[k_acc] - ts[j0]), 2),
                          rejected_frames=int(np.sum(st2[j0:k_acc] != G.ACCEPT)),
                          heading_err_deg=round(derr, 2),
                          position_err_m=round(float(2.0 * LNORM
                                                     * np.sin(np.radians(derr) / 2.0)), 4))
    return st != G.ACCEPT, uy, ev


def fg(rej, subset="all"):
    msk = OPS if subset == "operational" else np.ones(N, bool)
    gsub = gen_np & msk
    return 100.0 * int((rej & gsub).sum()) / int(gsub.sum())


def full_row(sigma_deg, k, q, rej, uy, note=""):
    rows = []
    pub = np.ones(N, bool)
    for lab, msk, bags_ in (("all", np.ones(N, bool), None),
                            ("operational", OPS, set(OP_BAGS))):
        c_, e_, _ = gated_residuals(pub, uy, ~rej, only_bags=bags_)
        ons = ONS_ALL if lab == "all" else ONSET["operational"]
        gsub = gen_np & msk
        rows.append(dict(q_yaw=q, q_over_q0=round(q / Q0, 6), sigma_deg=sigma_deg,
                         k_sigma=k, subset=lab,
                         false_gate_pct=round(100.0 * int((rej & gsub).sum())
                                              / int(gsub.sum()), 4),
                         genuine_frames=int(gsub.sum()),
                         genuine_rejected=int((rej & gsub).sum()),
                         rejected=int((rej & msk).sum()),
                         rejected_pct_all_frames=round(100.0 * int((rej & msk).sum())
                                                       / int(msk.sum()), 4),
                         withheld=0,
                         onsets_rejected=int(rej[ons].sum()), onsets=int(ons.size),
                         residual_episodes_010=e_[0.10], residual_frames_010=c_[0.10],
                         residual_episodes_005=e_[0.05],
                         note=note))
    return rows


ROWS, EV = [], []
QS = [Q0 / 100.0, Q0 / 10.0, Q0, Q0 * 10.0, Q0 * 100.0, Q0 * 1000.0]
SIGMAS = (0.74, 1.09)
for q in QS:
    for sg in SIGMAS:
        for k in (3.0, 5.0):
            t1 = time.time()
            rej, uy, ev = run(sg, k, q)
            ROWS += full_row(sg, k, q, rej, uy, note="q sweep, grid")
            if ev:
                EV.append(ev)
            print("q %.4g (x%.4g) sigma %.2f k %.1f  fg_all %.4f fg_op %.4f  (%.1f s)"
                  % (q, q / Q0, sg, k, fg(rej), fg(rej, "operational"), time.time() - t1),
                  flush=True)

# ---- matched availability in q, sigma = 0.74: which q reaches v3(b)'s 0.044 % -----
MATCH = []
for k in (3.0, 5.0):
    lo, hi = np.log10(Q0), np.log10(Q0 * 1e9)
    best = None
    for _ in range(26):
        mid = 0.5 * (lo + hi)
        q = 10.0 ** mid
        rej, uy, ev = run(0.74, k, q)
        v = fg(rej)
        if best is None or abs(v - 0.044) < abs(best[1] - 0.044):
            best = (q, v, rej, uy, ev)
        if v > 0.044:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-3:
            break
    q_b, v_b, rej_b, uy_b, ev_b = best
    MATCH.append((k, q_b, v_b))
    ROWS += full_row(0.74, k, q_b, rej_b, uy_b,
                     note="matched availability to v3(b) all 0.044 %% (q bisected)")
    if ev_b:
        ev_b = dict(ev_b); ev_b["note"] = "matched availability"
        EV.append(ev_b)
    print("MATCH sigma 0.74 k %.1f -> q %.4g (x%.4g), fg_all %.4f"
          % (k, q_b, q_b / Q0, v_b), flush=True)

df = pd.DataFrame(ROWS)
_real_to_csv(df, os.path.join(FIG, "gate_mahal_q_sweep.csv"), index=False)
_real_to_csv(pd.DataFrame(EV), os.path.join(FIG, "gate_mahal_q_sweep_event1.csv"), index=False)
print(df.to_string())
print(pd.DataFrame(EV).to_string())
print("[total %.1f s]" % (time.time() - t0))
