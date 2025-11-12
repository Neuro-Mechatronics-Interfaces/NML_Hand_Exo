#!/usr/bin/env python3
"""
Conducts power analysis for one-sided binomial tests and futility checks.
Usage (from repo root):
    python tools/power_bernoulli.py
    
    # For help on any mode:
    python tools/power_bernoulli.py -h
    python tools/power_bernoulli.py plan -h
    python tools/power_bernoulli.py futility -h

    # Suggested usage:
    # Find minimal n to achieve 80% power at alpha=0.01 for p0=0.9 and default list of true p1 accuracy levels.
    python tools/power_bernoulli.py plan --alpha 0.05 --power 0.80 --p0 0.95

    # Example: We have person with true success rate of 95% and we have observed 25 trials with 20 successes due to a poor start. 
    # Can we still hope to reject H0: p <= 0.95 at alpha=0.05 by or before Nmax=200 with expected future performance?    
    python tools/power_bernoulli.py futility --alpha 0.05 --p0 0.95 --t 25 --x 20 --ncap 200 
"""

import argparse
import math
from math import lgamma
from functools import lru_cache
from statistics import NormalDist

# ------------------------
# Helpers for randomized mixing and power
# ------------------------
def randomized_mixing(n, p0, alpha, kcrit):
    """Return r in [0,1] such that size is exactly alpha by randomizing at kcrit-1."""
    tail_k   = binom_sf(n, p0, kcrit)       # P_{p0}(X >= kcrit)
    pmf_km1  = binom_sf(n, p0, kcrit-1) - tail_k  # = P_{p0}(X = kcrit-1)
    if pmf_km1 <= 0:
        return 0.0  # no state to randomize on
    r = (alpha - tail_k) / pmf_km1
    return max(0.0, min(1.0, r))

def randomized_power(n, p0, p1, alpha, kcrit):
    """Power of the randomized test that mixes at kcrit-1."""
    r = randomized_mixing(n, p0, alpha, kcrit)
    tail_k_p1  = binom_sf(n, p1, kcrit)            # P_{p1}(X >= kcrit)
    pmf_km1_p1 = binom_sf(n, p1, kcrit-1) - tail_k_p1  # P_{p1}(X = kcrit-1)
    return tail_k_p1 + r * pmf_km1_p1, r


# ------------------------
# Binomial tail (stable)
# ------------------------
def log_binom(n, k):  # log nCk
    return lgamma(n + 1) - lgamma(k + 1) - lgamma(n - k + 1)

@lru_cache(maxsize=None)
def binom_sf(n, p, k):
    """P[X >= k] for X~Binom(n,p); k can be 0..n. Uses log-sum-exp."""
    if k <= 0:
        return 1.0
    if k > n:
        return 0.0
    logs = [log_binom(n, i) + i * math.log(p) + (n - i) * math.log(1 - p)
            for i in range(k, n + 1)]
    m = max(logs)
    return math.exp(m) * sum(math.exp(L - m) for L in logs)

# ------------------------
# Critical k (exact)
# ------------------------
def critical_k(n, p0, alpha):
    """
    Smallest integer k s.t. P_{p0}(X >= k) <= alpha; None if impossible.
    Uses a short binary search, seeded near a normal quantile for speed.
    """
    # Quick impossible check: at k = n (all correct) tail = p0^n
    if (p0 ** n) > alpha:
        return None

    # Normal-approx seed (with a small continuity correction)
    nd = NormalDist()
    mu = n * p0
    sd = math.sqrt(n * p0 * (1 - p0))
    k_seed = max(0, min(n, int(math.ceil(mu + nd.inv_cdf(1 - alpha) * sd - 0.5))))

    # Binary search on k in [0, n]
    lo, hi = 0, n
    # Narrow bounds using seed if helpful
    if 0 <= k_seed <= n:
        # If tail at k_seed already <= alpha, raise lo
        if binom_sf(n, p0, k_seed) <= alpha:
            lo = k_seed
        else:
            hi = k_seed

    while lo < hi:
        mid = (lo + hi) // 2
        if binom_sf(n, p0, mid) <= alpha:
            hi = mid
        else:
            lo = mid + 1
    return lo

# ------------------------
# Power at n (exact)
# ------------------------
def power_at_n(n, p0, p1, alpha):
    kcrit = critical_k(n, p0, alpha)
    if kcrit is None:
        return None, None  # cannot reject at any k
        # kcrit = round(p1 * n)
    powr = binom_sf(n, p1, kcrit)
    return kcrit, powr

# ------------------------
# Fast search for min n
# ------------------------
def approx_n_normal(p0, p1, alpha, power):
    """
    Back-of-envelope n from a one-sided normal-approx test.
    Use as a starting point only.
    """
    nd = NormalDist()
    z1a = nd.inv_cdf(1 - alpha)
    z1b = nd.inv_cdf(power)
    num = (z1a * math.sqrt(p0 * (1 - p0)) + z1b * math.sqrt(p1 * (1 - p1))) ** 2
    den = (p1 - p0) ** 2
    return max(1, int(math.ceil(num / den)))

def min_n_binom(p0=0.95, p1=0.98, alpha=0.05, power=0.80, nmax=200000):
    """
    Find minimal n with exact binomial power >= target.
    Uses:
      1) normal approx to seed,
      2) exponential bracketing,
      3) binary search.
    """
    if not (0 < p0 < 1 and 0 < p1 < 1):
        return None, None, None

    # Seed from approximation
    n_seed = approx_n_normal(p0, p1, alpha, power)

    # Helper: does n achieve target power?
    def achieves(n):
        kcrit, powr = power_at_n(n, p0, p1, alpha)
        return (kcrit is not None) and (powr is not None) and (powr >= power)

    # If seed works, try to shrink with binary search below it
    if achieves(n_seed):
        lo, hi = 1, n_seed
    else:
        # Exponential growth until it works (or hit nmax)
        lo, hi = n_seed, n_seed
        while hi < nmax and not achieves(hi):
            lo = hi + 1
            hi = min(nmax, max(2 * hi, hi + 1))
        if hi == nmax and not achieves(hi):
            return None, None, None

    # Binary search in [lo, hi] for minimal n
    ans_n, ans_k, ans_pow = None, None, None
    while lo <= hi:
        mid = (lo + hi) // 2
        kcrit, powr = power_at_n(mid, p0, p1, alpha)
        if (kcrit is not None) and (powr is not None) and (powr >= power):
            ans_n, ans_k, ans_pow = mid, kcrit, powr
            hi = mid - 1
        else:
            lo = mid + 1
    return ans_n, ans_k, ans_pow

# ------------------------
# Futility checks
# ------------------------
def can_ever_reject_from(t, x, p0, alpha, Nmax):
    """
    Check if you could possibly reject H0: p <= p0 at level alpha
    by or before Nmax, assuming all remaining trials are successes.
    Returns (possible: bool, first_n: int|None, kcrit_at_first_n: int|None).
    """
    for n in range(t, Nmax + 1):
        kcrit = critical_k(n, p0, alpha)
        if kcrit is None:
            continue
        # Best-case future: all remaining successes
        max_successes_possible = x + (n - t)
        if max_successes_possible >= kcrit:
            return True, n, kcrit
    return False, None, None



def kcrit_sequence(p0, alpha, Nmax):
    """
    Precompute kcrit[n] for n=0..Nmax where kcrit[n] is the smallest k such that
    P_{p0}(X>=k) <= alpha (or None if impossible). Index 0 unused.
    """
    seq = [None] * (Nmax + 1)
    seq[0] = None
    for n in range(1, Nmax + 1):
        seq[n] = critical_k(n, p0, alpha)
    return seq

def futility_horizon_expected(p0, p1, alpha, Nmax, kcrit_seq=None, round_fn=round):
    """
    Find the earliest t (1..Nmax) such that, if X≈round_fn(p1*t) so far,
    then even with ALL remaining successes up to some n in [t..Nmax],
    you still cannot meet the exact test (i.e., for every n in [t..Nmax],
    x + (n - t) < kcrit[n]).
    Returns (t_stop, x_at_t_stop) or (None, None) if futility is never forced by Nmax.
    """
    if not (0 < p1 < 1):
        return None, None
    if kcrit_seq is None:
        kcrit_seq = kcrit_sequence(p0, alpha, Nmax)

    for t in range(1, Nmax + 1):
        x = int(round_fn(p1 * t))
        # Check if there exists ANY n in [t..Nmax] such that you could still pass:
        # condition for possible pass at n: x + (n - t) >= kcrit[n]
        possible = False
        for n in range(t, Nmax + 1):
            kc = kcrit_seq[n]
            if kc is None:
                continue
            if x + (n - t) >= kc:
                possible = True
                break
        if not possible:
            return t, x
    return None, None

def kcrit_sequence_fast(p0, alpha, Nmax):
    """
    O(Nmax) incremental sweep exploiting monotonicity:
    kcrit(n+1) is either kcrit(n) or kcrit(n)+1 (rarely +2).
    """
    seq = [None] * (Nmax + 1)
    k_prev = None
    for n in range(1, Nmax + 1):
        # quick impossibility check: even all-correct tail p0**n > alpha => None
        if p0**n > alpha:
            seq[n] = None
            continue
        # start from previous kcrit if available, else normal seed
        if k_prev is None:
            # normal seed (same as in critical_k)
            nd = NormalDist()
            mu = n * p0
            sd = math.sqrt(n * p0 * (1 - p0))
            k = max(0, min(n, int(math.ceil(mu + nd.inv_cdf(1 - alpha) * sd - 0.5))))
        else:
            # k cannot decrease; start from k_prev
            k = k_prev

        # find smallest k with tail <= alpha
        # try current k; if tail > alpha, bump k up until it passes
        while k <= n and binom_sf(n, p0, k) > alpha:
            k += 1
        # ensure minimality (drop if we overshot)
        while k > 0 and binom_sf(n, p0, k - 1) <= alpha:
            k -= 1

        seq[n] = k if k <= n else None
        k_prev = seq[n] if seq[n] is not None else k_prev
    return seq

def prepare_futility_tables(p0, alpha, Nmax):
    """
    Build kcrit[n], M[n]=n-kcrit[n] (with -inf for None), and SufMax[n].
    """
    kcrit = kcrit_sequence_fast(p0, alpha, Nmax)
    M = [float("-inf")] * (Nmax + 1)
    for n in range(1, Nmax + 1):
        kc = kcrit[n]
        if kc is not None:
            M[n] = n - kc
    # suffix maxima
    SufMax = [float("-inf")] * (Nmax + 2)
    for n in range(Nmax, 0, -1):
        SufMax[n] = max(M[n], SufMax[n + 1])
    return kcrit, M, SufMax

def futility_horizon_expected_fast(p0, p1, alpha, Nmax, SufMax, round_fn=round):
    """
    O(Nmax): earliest t where SufMax[t] < t - x  (with x≈round(p1*t)).
    Returns (t_stop, x_at_t_stop) or (None, None).
    """
    if not (0 < p1 < 1):
        return None, None
    for t in range(1, Nmax + 1):
        x = int(round_fn(p1 * t))
        if SufMax[t] < (t - x):
            return t, x
    return None, None


# ------------------------
# CLI
# ------------------------
def plan_mode(args):
    print(f"Binomial power analysis (one-sided): alpha={args.alpha}, power target={args.power}, p0={args.p0}")
    rtype = "NONRANDOMIZED" if args.non_randomized else "RANDOMIZED"
    kcrit_seq, M, SufMax = prepare_futility_tables(args.p0, args.alpha, args.nmax)

    for p1 in args.p1:
        if not (0 < p1 < 1):
            print(f"  [{rtype}]  p1={p1:.3f}: (skip; must be in (0,1)).")
            continue

        n_est = approx_n_normal(args.p0, p1, args.alpha, args.power)
        n, kcrit, powr = min_n_binom(args.p0, p1, args.alpha, args.power, args.nmax)

        # CASE A: We found a valid design n (power >= target) -------------------
        if (n is not None) and (kcrit is not None):
            if args.non_randomized:
                approx_str = f" [approx n0≈{n_est}]" if args.show_approx else ""
                print(f"  [{rtype}]  p1={p1:.3f}: n ≈ {n} (need ≥ {kcrit} correct), achieved power={powr:.3f}{approx_str}")
            else:
                rand_pow, r = randomized_power(n, args.p0, p1, args.alpha, kcrit)
                size_tail = binom_sf(n, args.p0, kcrit)
                print(f"  [{rtype}]  p1={p1:.3f}: n ≈ {n} (need ≥ {kcrit} correct), "
                      f"power: r={r:.3f}, power={rand_pow:.3f}, p={size_tail:.3g}")
            continue

        # CASE B: No such n up to nmax (or you want futility behavior when p1<=p0) ----
        # Compute earliest expected futility horizon under p1
        t_stop, x_stop = futility_horizon_expected_fast(args.p0, p1, args.alpha, args.nmax, SufMax)
        if t_stop is None:
            # Never forced to stop by Nmax (you might remain "mathematically possible"),
            # but you also didn't achieve target power by any n <= Nmax.
            print(f"  [{rtype}]  p1={p1:.3f}: no design with target power up to nmax={args.nmax}; "
                  f"futility not forced by Nmax under expected path.")
        else:
            # From t_stop onward, even with perfect future you cannot pass by or before Nmax
            # Report how many additional trials remain until Nmax as context
            remaining = args.nmax - t_stop
            print(f"  [{rtype}]  p1={p1:.3f}: no design with target power up to nmax={args.nmax}.")
            print(f"           → Expected futility by t≈{t_stop} (x≈{x_stop} correct; {x_stop}/{t_stop}≈{x_stop/t_stop:.3f}).")
            print(f"             From that point, even perfect future performance cannot reach significance by Nmax={args.nmax}.")

def futility_mode(args):
    # Basic validation
    if not (0 <= args.x <= args.t):
        raise ValueError("Require 0 <= x <= t.")
    print(f"Futility check (one-sided exact test): p0={args.p0}, alpha={args.alpha}, observed t={args.t}, x={args.x}, Nmax={args.ncap}")
    possible, n_hit, kcrit = can_ever_reject_from(args.t, args.x, args.p0, args.alpha, args.ncap)
    if not possible:
        # compute max possible successes at Nmax and the corresponding shortfall
        # and the current best-case pass probability at Nmax too (optional)
        # But more useful: show how many additional successes you would need by Nmax
        # to cross the threshold (if even defined)
        print("  RESULT: Stop for futility. Even with perfect future performance you cannot reach significance by or before Nmax.")
        return
    # Earliest point where passing is still possible
    addl_trials_needed = n_hit - args.t
    max_successes_future = n_hit - args.t  # because we assumed all future successes
    required_addl_successes = max(0, kcrit - args.x)  # successes needed by n_hit
    print("  RESULT: Continuing could still succeed under best-case performance.")
    print(f"          Earliest n where a pass is possible: n={n_hit} (need ≥ {kcrit} total correct by then).")
    print(f"          From now: need at least {required_addl_successes} additional successes over the next {addl_trials_needed} trials.")

def main():
    parser = argparse.ArgumentParser(description="Exact one-sided binomial tools (power planning and futility checks).")
    subparsers = parser.add_subparsers(dest="mode")

    # ---- plan (default) ----
    p_plan = subparsers.add_parser("plan", help="Power planning over a list of true accuracies p1.")
    p_plan.add_argument("--alpha", type=float, default=0.05, help="Significance (one-sided).")
    p_plan.add_argument("--power", type=float, default=0.80, help="Target power (1 - beta).")
    p_plan.add_argument("--p0", type=float, default=0.95, help="Threshold accuracy under H0.")
    p_plan.add_argument("--p1", nargs="+", type=float, default=[0.1, 0.5, 0.98, 0.995, 0.99999],
                        help="True accuracies to evaluate.")
    p_plan.add_argument("--nmax", type=int, default=500, help="Safety cap on n.")
    p_plan.add_argument("--show-approx", action="store_true", help="Also show normal-approx starting n.")
    p_plan.add_argument("--non-randomized", action="store_true", help="Report non-randomized-test size matching and power (mix at kcrit-1).")

    # ---- futility ----
    p_fut = subparsers.add_parser("futility", help="Simple early stop for futility given interim data (t, x) and Nmax.")
    p_fut.add_argument("--alpha", type=float, default=0.05, help="Significance (one-sided).")
    p_fut.add_argument("--p0", type=float, default=0.95, help="Threshold accuracy under H0.")
    p_fut.add_argument("--t", type=int, required=True, help="Trials observed so far.")
    p_fut.add_argument("--x", type=int, required=True, help="Successes observed so far.")
    p_fut.add_argument("--ncap", type=int, required=True, help="Maximum total sample size allowed (Nmax).")
    args = parser.parse_args()

    # 🔧 If no subcommand, inject "plan"
    if args.mode is None:
        # Reparse with "plan" injected
        args = parser.parse_args(["plan"] + vars(args).get("extra", []))

    if args.mode == "plan":
        plan_mode(args)
    elif args.mode == "futility":
        futility_mode(args)
    else:
        raise ValueError(f"Unknown mode: {args.mode}")

if __name__ == "__main__":
    main()
