#!/usr/bin/env python3
"""
Extracts A1 admittance parameters from PlotJuggler CSV exports
(layout: admittance_calibration_layout.xml).

    static   test a: stiffness from deflection under a known mass
    release  test c: natural frequency, damping ratio, overshoot, settling time
    delay    test b: UR position loop delay, reference vs feedback per joint
    press    test d: force vs deflection in contact, peak force
    columns  list the series the script recognises in a CSV

Only numpy is required; --plot needs matplotlib.
"""

import argparse
import re
import sys

import numpy as np

STATUS = "/admittance_controller/status"
JTC = "/admittance_jtc/controller_state"
G = 9.81

PATTERNS = {
    "offset": STATUS + r"/admittance_position/transform/translation/{axis}$",
    "force": STATUS + r"/wrench_base/wrench/force/{axis}$",
    "adm_vel": STATUS + r"/admittance_velocity/twist/linear/{axis}$",
    "ref": JTC + r"/reference/positions[\[./]?{joint}\]?$",
    "fb": JTC + r"/feedback/positions[\[./]?{joint}\]?$",
}


class Recording:
    """PlotJuggler CSV: `__time` (or CSV Exporter `time`) column plus one column per series, empty where a series has no sample."""

    def __init__(self, path: str):
        with open(path) as f:
            self.header = [h.strip().strip('"') for h in f.readline().split(",")]
        data = np.genfromtxt(path, delimiter=",", skip_header=1, filling_values=np.nan)
        self.data = np.atleast_2d(data)
        time_cols = [i for i, h in enumerate(self.header) if h in ("__time", "time")]
        if not time_cols:
            sys.exit(f"No `__time` / `time` column in {path}.")
        self.t_col = time_cols[0]

    def find(self, key: str, **fmt) -> int | None:
        regex = re.compile(PATTERNS[key].format(**fmt))
        hits = [i for i, h in enumerate(self.header) if regex.search(h)]
        return hits[0] if hits else None

    def series(self, key: str, **fmt) -> tuple[np.ndarray, np.ndarray]:
        col = self.find(key, **fmt)
        if col is None:
            sys.exit(
                f"Series '{PATTERNS[key].format(**fmt)}' not found. Run `columns` to see the header."
            )
        t, v = self.data[:, self.t_col], self.data[:, col]
        mask = ~np.isnan(v) & ~np.isnan(t)
        return t[mask], v[mask]


def resample(t: np.ndarray, v: np.ndarray, grid: np.ndarray) -> np.ndarray:
    return np.interp(grid, t, v)


def uniform_grid(t: np.ndarray, rate: float) -> np.ndarray:
    return np.arange(t[0], t[-1], 1.0 / rate)


def window(t, v, t_from, t_to):
    m = np.ones_like(t, dtype=bool)
    if t_from is not None:
        m &= t >= t[0] + t_from
    if t_to is not None:
        m &= t <= t[0] + t_to
    return t[m], v[m]


def maybe_plot(args, draw):
    if not args.plot:
        return
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = draw(plt)
    fig.tight_layout()
    fig.savefig(args.plot, dpi=150)
    print(f"plot: {args.plot}")


# ---------------------------------------------------------------------------- static (a)


def cmd_static(rec: Recording, args):
    t, x = window(*rec.series("offset", axis=args.axis), args.t_from, args.t_to)
    tf, f = window(*rec.series("force", axis=args.axis), args.t_from, args.t_to)
    f_on_x = resample(tf, f, t)

    k_fit, b_fit = np.polyfit(x, f_on_x, 1)
    # Loaded / unloaded plateaus: samples within 10% of the force range from either extreme.
    # Comparing plateau levels removes the baseline drift that a single deflection value includes.
    f_range = f_on_x.max() - f_on_x.min()
    loaded_low = abs(f_on_x.min()) > abs(f_on_x.max())
    f_loaded_end = f_on_x.min() if loaded_low else f_on_x.max()
    f_free_end = f_on_x.max() if loaded_low else f_on_x.min()
    loaded = np.abs(f_on_x - f_loaded_end) < 0.1 * f_range
    free = np.abs(f_on_x - f_free_end) < 0.1 * f_range
    d_f = np.median(f_on_x[loaded]) - np.median(f_on_x[free])
    d_x = np.median(x[loaded]) - np.median(x[free])
    print(
        f"axis {args.axis}: deflection range {x.min() * 1e3:+.2f} .. {x.max() * 1e3:+.2f} mm"
    )
    print(
        f"F/x linear fit (all samples, includes transients): K = {k_fit:.1f} N/m (offset {b_fit:+.2f} N)"
    )
    print(
        f"plateaus: dF = {d_f:+.3f} N, dx = {d_x * 1e3:+.3f} mm -> K = {d_f / d_x:.1f} N/m, "
        f"load = {abs(d_f) / G:.3f} kg"
    )
    print(
        f"unloaded baseline: F = {np.median(f_on_x[free]):+.3f} N, x = {np.median(x[free]) * 1e3:+.3f} mm"
    )
    if args.mass:
        print(
            f"mass {args.mass} kg: expected |dx| = {args.mass * G / args.k_nominal * 1e3:.2f} mm "
            f"at K = {args.k_nominal:.0f} N/m, measured {abs(d_x) * 1e3:.2f} mm"
        )
    if k_fit < 0:
        print(
            "WARNING: negative slope, the arm deflects against the force. Check the F/T frame and signs."
        )

    def draw(plt):
        fig, ax = plt.subplots(2, 1, figsize=(8, 6))
        ax[0].plot(t - t[0], x * 1e3, label="x [mm]")
        ax[0].plot(t - t[0], f_on_x, label="F [N]")
        ax[0].legend()
        ax[0].set_xlabel("t [s]")
        ax[1].plot(x * 1e3, f_on_x, ".", ms=2)
        xs = np.linspace(x.min(), x.max(), 2)
        ax[1].plot(xs * 1e3, k_fit * xs + b_fit, "r-", label=f"K = {k_fit:.0f} N/m")
        ax[1].set_xlabel("x [mm]")
        ax[1].set_ylabel("F [N]")
        ax[1].legend()
        return fig

    maybe_plot(args, draw)


# ---------------------------------------------------------------------------- release (c)


def second_order_free(t, x0, v0, wn, zeta):
    """Free response of x'' + 2*zeta*wn*x' + wn^2*x = 0 for all three damping regimes."""
    if abs(zeta - 1.0) < 1e-6:
        return (x0 + (v0 + wn * x0) * t) * np.exp(-wn * t)
    if zeta < 1.0:
        wd = wn * np.sqrt(1.0 - zeta**2)
        return np.exp(-zeta * wn * t) * (
            x0 * np.cos(wd * t) + (v0 + zeta * wn * x0) / wd * np.sin(wd * t)
        )
    s = wn * np.sqrt(zeta**2 - 1.0)
    r1, r2 = -zeta * wn + s, -zeta * wn - s
    c2 = (v0 - r1 * x0) / (r2 - r1)
    return (x0 - c2) * np.exp(r1 * t) + c2 * np.exp(r2 * t)


def cmd_release(rec: Recording, args):
    t, x = window(*rec.series("offset", axis=args.axis), args.t_from, args.t_to)
    grid = uniform_grid(t, 1000.0)
    x = resample(t, x, grid)
    t = grid

    # The hand holds the deflection, then lets go: the free response starts at the last sample
    # of the hold, taken as the last one still within 2% of the peak before the decay.
    i_peak = int(np.argmax(np.abs(x)))
    below = np.nonzero(np.abs(x[i_peak:]) < 0.98 * np.abs(x[i_peak]))[0]
    i0 = i_peak + (int(below[0]) - 1 if below.size else 0)
    tr, xr = t[i0:] - t[i0], x[i0:]
    x_final = np.median(xr[tr > tr[-1] - min(0.5, tr[-1] / 4)])
    xr = xr - x_final
    # The onset lies a few ms into the decay, so the initial velocity is measured, not assumed 0.
    n_v = max(3, int(0.01 * 1000.0))
    v0, x0 = np.polyfit(tr[:n_v], xr[:n_v], 1)
    fit_mask = tr < args.fit_window

    best = (np.inf, None, None)
    for wn in np.linspace(1.0, 40.0, 391):
        for zeta in np.linspace(0.2, 3.0, 141):
            err = np.sum(
                (second_order_free(tr[fit_mask], x0, v0, wn, zeta) - xr[fit_mask]) ** 2
            )
            if err < best[0]:
                best = (err, wn, zeta)
    _, wn, zeta = best

    overshoot = max(0.0, -np.min(xr * np.sign(x0))) / abs(x0) * 100.0
    outside = np.nonzero(np.abs(xr) > 0.02 * abs(x0))[0]
    settling = tr[outside[-1]] if outside.size else 0.0
    print(
        f"axis {args.axis}: initial deflection {x0 * 1e3:+.2f} mm, final {x_final * 1e3:+.2f} mm"
    )
    print(
        f"fit: wn = {wn:.2f} rad/s, zeta = {zeta:.2f} "
        f"(nominal wn = sqrt(K/M) = {np.sqrt(args.k_nominal / args.m_nominal):.2f}, zeta = 1)"
    )
    print(f"overshoot {overshoot:.1f} %, 2% settling time {settling:.3f} s")

    def draw(plt):
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(tr, xr * 1e3, label="measured")
        ax.plot(
            tr,
            second_order_free(tr, x0, v0, wn, zeta) * 1e3,
            "--",
            label=f"fit wn={wn:.1f}, ζ={zeta:.2f}",
        )
        ax.set_xlim(0, min(tr[-1], 3 * args.fit_window))
        ax.set_xlabel("t since release [s]")
        ax.set_ylabel("x [mm]")
        ax.legend()
        return fig

    maybe_plot(args, draw)


# ---------------------------------------------------------------------------- delay (b)


def estimate_delay(t_ref, ref, t_fb, fb, rate, max_lag):
    """
    Delay of feedback w.r.t. reference by cross-correlating their velocities on a uniform grid.
    Velocities (not positions) remove the large DC/ramp component that would bias the peak.
    Each lag uses the correlation coefficient of the overlapping parts, so shorter overlaps at
    larger lags are not penalised. The peak is refined with a parabola through its neighbours.
    """
    t0, t1 = max(t_ref[0], t_fb[0]), min(t_ref[-1], t_fb[-1])
    grid = np.arange(t0, t1, 1.0 / rate)
    r = np.gradient(resample(t_ref, ref, grid)) * rate
    f = np.gradient(resample(t_fb, fb, grid)) * rate
    n_lag = int(max_lag * rate)
    lags = np.arange(0, n_lag + 1)
    corr = np.array([np.corrcoef(r[: len(r) - lag], f[lag:])[0, 1] for lag in lags])
    k = int(np.argmax(corr))
    if 0 < k < len(corr) - 1:
        a, b, c = corr[k - 1], corr[k], corr[k + 1]
        denom = a - 2 * b + c
        k = k + (0.5 * (a - c) / denom if denom != 0 else 0.0)
    return k / rate, corr.max(), np.abs(r).max()


def cmd_delay(rec: Recording, args):
    delays, weights = [], []
    for j in range(6):
        if rec.find("ref", joint=j) is None or rec.find("fb", joint=j) is None:
            continue
        tr, r = window(*rec.series("ref", joint=j), args.t_from, args.t_to)
        tf, f = window(*rec.series("fb", joint=j), args.t_from, args.t_to)
        delay, quality, peak_vel = estimate_delay(tr, r, tf, f, 1000.0, args.max_lag)
        err = resample(tf, f, tr) - r
        print(
            f"joint {j}: delay {delay * 1e3:6.1f} ms  corr {quality:.2f}  peak |q'| {peak_vel:.3f} rad/s  "
            f"RMS err {np.sqrt(np.mean(err**2)) * 1e3:.2f} mrad"
        )
        if quality > 0.5 and peak_vel > 0.01:
            delays.append(delay)
            weights.append(peak_vel)
    if not delays:
        sys.exit("No joint moved enough for a reliable estimate.")
    print(
        f"weighted delay (joints with corr > 0.5): {np.average(delays, weights=weights) * 1e3:.1f} ms"
    )


# ---------------------------------------------------------------------------- press (d)


def cmd_press(rec: Recording, args):
    t, x = window(*rec.series("offset", axis=args.axis), args.t_from, args.t_to)
    tf, f = window(*rec.series("force", axis=args.axis), args.t_from, args.t_to)
    f_on_x = resample(tf, f, t)
    i_peak = int(np.argmax(np.abs(f_on_x)))
    contact = np.abs(f_on_x) > args.contact_threshold
    print(
        f"axis {args.axis}: peak |F| = {abs(f_on_x[i_peak]):.2f} N at x = {x[i_peak] * 1e3:+.2f} mm, "
        f"t = {t[i_peak] - t[0]:.2f} s"
    )
    if contact.sum() > 10:
        k_fit, _ = np.polyfit(x[contact], f_on_x[contact], 1)
        print(
            f"in contact (|F| > {args.contact_threshold} N): F/x slope {k_fit:.1f} N/m, "
            f"contact time {contact.sum() / contact.size * (t[-1] - t[0]):.2f} s"
        )
    print(f"force ceiling K * max_offset = {args.k_nominal * args.max_offset:.1f} N")

    def draw(plt):
        fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        ax[0].plot(t - t[0], f_on_x)
        ax[0].axhline(
            np.sign(f_on_x[i_peak]) * args.k_nominal * args.max_offset,
            color="r",
            ls="--",
            label="ceiling",
        )
        ax[0].set_ylabel("F [N]")
        ax[0].legend()
        ax[1].plot(t - t[0], x * 1e3)
        ax[1].set_ylabel("x [mm]")
        ax[1].set_xlabel("t [s]")
        return fig

    maybe_plot(args, draw)


def cmd_columns(rec: Recording, args):
    for key, pattern in PATTERNS.items():
        fmts = (
            [{"axis": a} for a in "xyz"]
            if "{axis}" in pattern
            else [{"joint": j} for j in range(6)]
        )
        found = [
            rec.header[c] for fmt in fmts if (c := rec.find(key, **fmt)) is not None
        ]
        print(f"{key:8s} {len(found)}/{len(fmts)}  {found[:1]}")
    print(f"\n{len(rec.header)} columns in total")


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument(
        "command", choices=["static", "release", "delay", "press", "columns"]
    )
    p.add_argument("csv")
    p.add_argument("--axis", choices="xyz", default="z", help="base_link axis")
    p.add_argument("--t-from", type=float, help="[s] from the start of the recording")
    p.add_argument("--t-to", type=float)
    p.add_argument("--plot", help="output image path")
    p.add_argument("--mass", type=float, help="[kg] hung mass (static)")
    p.add_argument("--k-nominal", type=float, default=500.0, help="[N/m]")
    p.add_argument("--m-nominal", type=float, default=5.0, help="[kg]")
    p.add_argument(
        "--max-offset", type=float, default=0.02, help="[m] admittance_max_offset_lin"
    )
    p.add_argument(
        "--fit-window",
        type=float,
        default=1.0,
        help="[s] after release used for the fit",
    )
    p.add_argument("--max-lag", type=float, default=0.15, help="[s] delay search range")
    p.add_argument("--contact-threshold", type=float, default=1.0, help="[N]")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rec = Recording(args.csv)
    {
        "static": cmd_static,
        "release": cmd_release,
        "delay": cmd_delay,
        "press": cmd_press,
        "columns": cmd_columns,
    }[args.command](rec, args)
