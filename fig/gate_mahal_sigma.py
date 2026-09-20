#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rerun the Mahalanobis (normalised-innovation) comparison of results_numbers.py §9.5
with the MEASURED heading noise instead of the driver's declared 0.50 deg.

Reuses results_numbers.py verbatim up to (but not including) its §9.5 block, so every
label, denominator and helper is the same object the manuscript numbers came from.
All to_csv calls are neutralised first: no existing CSV is touched.
Writes only fig/gate_mahal_sigma.csv.
"""
import os
import sys
import time

import numpy as np
import pandas as pd

P2 = os.environ.get("P2_ROOT",
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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
gen_np, OPS, IND = ns["gen_np"], ns["OPS"], ns["IND"]
OP_BAGS, INDUCED_BAGS = ns["OP_BAGS"], ns["INDUCED_BAGS"]
ONSET = ns["ONSET"]
gated_residuals = ns["gated_residuals"]
wrap180 = ns["wrap180"]
LNORM = ns["LNORM"]
EV_BAG, EV_J0, EV_HDG, EV_CG = ns["EV_BAG"], ns["EV_J0"], ns["EV_HDG"], ns["EV_CG"]

N_GEN_ALL = int(gen_np.sum())
N_GEN_OP = int((gen_np & OPS).sum())
print("genuine all %d, genuine operational %d, onsets op %d, onsets induced %d"
      % (N_GEN_ALL, N_GEN_OP, ONSET["operational"].size, ONSET["induced"].size), flush=True)

ONS_ALL = np.concatenate([ONSET["operational"], ONSET["induced"]])


def run(sigma_deg, k):
    """One full-corpus pass of the 1-D Mahalanobis filter. Returns (rejected mask, used yaw)."""
    r_yaw = float(np.radians(sigma_deg)) ** 2
    st = np.zeros(N, np.int8)
    uy = np.zeros(N)
    for b, idx, yaw, ts in bag_groups:
        r_ = G.gate_series_mahal(yaw, ts, cum_gyro_rad=cum_gyro_yaw[idx], k_sigma=k,
                                 r_yaw=r_yaw)
        st[idx] = r_["state"]
        uy[idx] = r_["used_yaw"]
    return st != G.ACCEPT, uy


def fg(rej, subset):
    msk = OPS if subset == "operational" else np.ones(N, bool)
    gsub = gen_np & msk
    return 100.0 * int((rej & gsub).sum()) / int(gsub.sum())


def full_row(sigma_deg, k, rej, uy, note=""):
    rows = []
    pub = np.ones(N, bool)
    for lab, msk, bags_ in (("all", np.ones(N, bool), None),
                            ("operational", OPS, set(OP_BAGS))):
        c_, e_, _ = gated_residuals(pub, uy, ~rej, only_bags=bags_)
        ons = ONS_ALL if lab == "all" else ONSET["operational"]
        gsub = gen_np & msk
        rows.append(dict(sigma_deg=sigma_deg, k_sigma=k, subset=lab,
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


def event1(sigma_deg, k):
    """What the filter does with the 6 September showcase event, same code as §9.5."""
    r_yaw = float(np.radians(sigma_deg)) ** 2
    for b, idx, yaw, ts in bag_groups:
        if b != EV_BAG:
            continue
        r_ = G.gate_series_mahal(yaw, ts, cum_gyro_rad=cum_gyro_yaw[idx], k_sigma=k,
                                 r_yaw=r_yaw)
        st2 = r_["state"]
        j0 = EV_J0
        k_acc = next((q for q in range(j0 + 1, st2.size) if st2[q] == G.ACCEPT), None)
        if k_acc is None:
            return None
        true_hdg = EV_HDG[j0 - 1] + (EV_CG[k_acc] - EV_CG[j0 - 1])
        derr = float(abs(wrap180(EV_HDG[k_acc] - true_hdg)))
        return dict(sigma_deg=sigma_deg, k_sigma=k,
                    rejected_s=round(float(ts[k_acc] - ts[j0]), 2),
                    rejected_frames=int(np.sum(st2[j0:k_acc] != G.ACCEPT)),
                    heading_err_deg=round(derr, 2),
                    position_err_m=round(float(2.0 * LNORM
                                               * np.sin(np.radians(derr) / 2.0)), 4))
    return None


ROWS, EV = [], []
SIGMAS = (0.50, 0.74, 1.09)
for sg in SIGMAS:
    for k in (3.0, 5.0):
        t1 = time.time()
        rej, uy = run(sg, k)
        ROWS += full_row(sg, k, rej, uy, note="declared/measured sigma, same k as §9.5")
        EV.append(event1(sg, k))
        print("sigma %.2f k %.1f  fg_all %.4f fg_op %.4f  (%.1f s)"
              % (sg, k, fg(rej, "all"), fg(rej, "operational"), time.time() - t1), flush=True)

# ---- matched availability: the k whose false-gate rate equals v3(b)'s ------------
TARGETS = (("all", 0.044), ("operational", 0.041))
MATCH = []
for sg in SIGMAS:
    for subset, target in TARGETS:
        lo, hi = 3.0, 400.0
        best = None
        for _ in range(22):
            mid = 0.5 * (lo + hi)
            rej, uy = run(sg, mid)
            v = fg(rej, subset)
            if best is None or abs(v - target) < abs(best[1] - target):
                best = (mid, v, rej, uy)
            if v > target:
                lo = mid
            else:
                hi = mid
            if hi - lo < 1e-3:
                break
        k_b, v_b, rej_b, uy_b = best
        MATCH.append((sg, subset, target, k_b, v_b))
        ROWS += full_row(sg, round(k_b, 4), rej_b, uy_b,
                         note="matched availability to v3(b) %s %.3f %%" % (subset, target))
        EV.append(event1(sg, round(k_b, 4)))
        print("MATCH sigma %.2f %s target %.3f -> k %.3f, fg %.4f"
              % (sg, subset, target, k_b, v_b), flush=True)

df = pd.DataFrame(ROWS)
_real_to_csv(df, os.path.join(FIG, "gate_mahal_sigma.csv"), index=False)
_real_to_csv(pd.DataFrame([e for e in EV if e]),
             os.path.join(FIG, "gate_mahal_sigma_event1.csv"), index=False)
print(df.to_string())
print(pd.DataFrame([e for e in EV if e]).to_string())
print("[total %.1f s]" % (time.time() - t0))
