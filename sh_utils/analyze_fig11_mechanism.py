#!/usr/bin/env python3

import argparse
import csv
import math
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


ZT_ORDER = [2.0, 1.5, 1.0]
CASE_TAGS = {
    2.0: "zt_2_0",
    1.5: "zt_1_5",
    1.0: "zt_1_0",
}
PAIR_ORDER = [(2.0, 1.5), (2.0, 1.0), (1.5, 1.0)]
REQUIRED_SUMMARY_KEYS = [
    "vt1",
    "vt2",
    "vt_norm",
    "vt_signed",
    "duration",
    "winner_eps_level",
    "max_abs_omega2_int",
    "solve_mode",
]


@dataclass
class CaseSummary:
    zt: float
    case_tag: str
    vt1: float
    vt2: float
    vt_norm: float
    vt_signed: float
    duration: float
    winner_eps_level: str
    solve_mode: str
    max_abs_omega2_int: float
    max_abs_omega2_export: float


@dataclass
class CurveSeries:
    t: List[float]
    x: List[float]
    z: List[float]
    tau_norm: List[float]
    omega2: List[float]


@dataclass
class PairMetric:
    za: float
    zb: float
    traj_rmse_norm: float
    tau_rmse_norm: float
    omega_rmse_norm: float
    raw_span_x: float
    raw_span_z: float
    span_x: float
    span_z: float
    raw_mean_abs_union_tau: float
    raw_mean_abs_union_omega: float
    mean_abs_union_tau: float
    mean_abs_union_omega: float


def parse_kv(path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key.strip()] = value.strip()
    return data


def read_case_summary(input_dir: Path, zt: float) -> CaseSummary:
    case_tag = CASE_TAGS[zt]
    summary_path = input_dir / f"{case_tag}_summary.txt"
    if not summary_path.exists():
        raise RuntimeError(f"Missing summary: {summary_path}")

    raw = parse_kv(summary_path)
    for key in REQUIRED_SUMMARY_KEYS:
        if key not in raw:
            raise RuntimeError(f"Missing key '{key}' in {summary_path}")

    max_abs_omega2_export = float(raw.get("max_abs_omega2_export", "nan"))
    return CaseSummary(
        zt=zt,
        case_tag=case_tag,
        vt1=float(raw["vt1"]),
        vt2=float(raw["vt2"]),
        vt_norm=float(raw["vt_norm"]),
        vt_signed=float(raw["vt_signed"]),
        duration=float(raw["duration"]),
        winner_eps_level=raw["winner_eps_level"],
        solve_mode=raw["solve_mode"],
        max_abs_omega2_int=float(raw["max_abs_omega2_int"]),
        max_abs_omega2_export=max_abs_omega2_export,
    )


def read_case_curve(input_dir: Path, zt: float) -> CurveSeries:
    case_tag = CASE_TAGS[zt]
    csv_path = input_dir / f"{case_tag}.csv"
    if not csv_path.exists():
        raise RuntimeError(f"Missing curve csv: {csv_path}")

    t: List[float] = []
    x: List[float] = []
    z: List[float] = []
    tau_norm: List[float] = []
    omega2: List[float] = []

    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        required_cols = {"t", "x", "z", "omega2"}
        if reader.fieldnames is None:
            raise RuntimeError(f"Empty csv: {csv_path}")
        missing = required_cols.difference(reader.fieldnames)
        if missing:
            raise RuntimeError(f"Missing csv columns {sorted(missing)} in {csv_path}")

        tau_key = "tau_norm" if "tau_norm" in reader.fieldnames else "tau_norm2"
        for row in reader:
            try:
                ti = float(row["t"])
                xi = float(row["x"])
                zi = float(row["z"])
                taui = float(row[tau_key])
                omegai = float(row["omega2"])
            except (TypeError, ValueError, KeyError):
                continue
            if not all(math.isfinite(v) for v in (ti, xi, zi, taui, omegai)):
                continue
            t.append(ti)
            x.append(xi)
            z.append(zi)
            tau_norm.append(taui)
            omega2.append(omegai)

    if len(t) < 2:
        raise RuntimeError(f"Insufficient valid rows in {csv_path}")

    rows = sorted(zip(t, x, z, tau_norm, omega2), key=lambda p: p[0])
    dedup: List[Tuple[float, float, float, float, float]] = []
    for row in rows:
        if dedup and abs(row[0] - dedup[-1][0]) <= 1e-12:
            dedup[-1] = row
        else:
            dedup.append(row)

    if len(dedup) < 2:
        raise RuntimeError(f"Degenerate time axis in {csv_path}")

    return CurveSeries(
        t=[r[0] for r in dedup],
        x=[r[1] for r in dedup],
        z=[r[2] for r in dedup],
        tau_norm=[r[3] for r in dedup],
        omega2=[r[4] for r in dedup],
    )


def linear_interp(u_src: List[float], y_src: List[float], u_query: List[float]) -> List[float]:
    out: List[float] = []
    n = len(u_src)
    for uq in u_query:
        if uq <= u_src[0]:
            out.append(y_src[0])
            continue
        if uq >= u_src[-1]:
            out.append(y_src[-1])
            continue

        idx = bisect_left(u_src, uq)
        i0 = max(0, idx - 1)
        i1 = min(n - 1, idx)
        if i1 == i0:
            out.append(y_src[i0])
            continue

        u0 = u_src[i0]
        u1 = u_src[i1]
        if abs(u1 - u0) <= 1e-12:
            out.append(y_src[i0])
            continue

        alpha = (uq - u0) / (u1 - u0)
        out.append((1.0 - alpha) * y_src[i0] + alpha * y_src[i1])
    return out


def normalize_and_resample(curve: CurveSeries, sample_count: int, eps: float) -> CurveSeries:
    t0 = curve.t[0]
    t1 = curve.t[-1]
    duration = max(t1 - t0, eps)
    u_src = [(ti - t0) / duration for ti in curve.t]
    u_grid = [i / float(sample_count - 1) for i in range(sample_count)]

    return CurveSeries(
        t=u_grid,
        x=linear_interp(u_src, curve.x, u_grid),
        z=linear_interp(u_src, curve.z, u_grid),
        tau_norm=linear_interp(u_src, curve.tau_norm, u_grid),
        omega2=linear_interp(u_src, curve.omega2, u_grid),
    )


def mean_abs(values: List[float]) -> float:
    if not values:
        return 0.0
    return sum(abs(v) for v in values) / float(len(values))


def rmse(values: List[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / float(len(values)))


def pair_metrics(curve_a: CurveSeries, curve_b: CurveSeries, eps: float) -> PairMetric:
    raw_span_x = max(max(curve_a.x), max(curve_b.x)) - min(min(curve_a.x), min(curve_b.x))
    raw_span_z = max(max(curve_a.z), max(curve_b.z)) - min(min(curve_a.z), min(curve_b.z))
    span_x = max(raw_span_x, eps)
    span_z = max(raw_span_z, eps)

    dx_norm = [(xa - xb) / span_x for xa, xb in zip(curve_a.x, curve_b.x)]
    dz_norm = [(za - zb) / span_z for za, zb in zip(curve_a.z, curve_b.z)]
    traj_norm_sq = [dx * dx + dz * dz for dx, dz in zip(dx_norm, dz_norm)]
    traj_rmse_norm = math.sqrt(sum(traj_norm_sq) / float(len(traj_norm_sq)))

    tau_diff = [ta - tb for ta, tb in zip(curve_a.tau_norm, curve_b.tau_norm)]
    raw_mean_abs_union_tau = mean_abs(curve_a.tau_norm + curve_b.tau_norm)
    mean_abs_union_tau = max(raw_mean_abs_union_tau, eps)
    tau_rmse_norm = rmse(tau_diff) / mean_abs_union_tau

    omega_diff = [oa - ob for oa, ob in zip(curve_a.omega2, curve_b.omega2)]
    raw_mean_abs_union_omega = mean_abs(curve_a.omega2 + curve_b.omega2)
    mean_abs_union_omega = max(raw_mean_abs_union_omega, eps)
    omega_rmse_norm = rmse(omega_diff) / mean_abs_union_omega

    return PairMetric(
        za=0.0,
        zb=0.0,
        traj_rmse_norm=traj_rmse_norm,
        tau_rmse_norm=tau_rmse_norm,
        omega_rmse_norm=omega_rmse_norm,
        raw_span_x=raw_span_x,
        raw_span_z=raw_span_z,
        span_x=span_x,
        span_z=span_z,
        raw_mean_abs_union_tau=raw_mean_abs_union_tau,
        raw_mean_abs_union_omega=raw_mean_abs_union_omega,
        mean_abs_union_tau=mean_abs_union_tau,
        mean_abs_union_omega=mean_abs_union_omega,
    )


def pass_flag(ok: bool) -> str:
    return "PASS" if ok else "FAIL"


def write_metrics_tsv(out_path: Path, summaries: Dict[float, CaseSummary]) -> None:
    with out_path.open("w", newline="") as f:
        writer = csv.writer(f, delimiter="\t")
        writer.writerow(
            [
                "case_tag",
                "zt",
                "vt1",
                "vt2",
                "vt_norm",
                "neg_vt2",
                "vt_signed",
                "duration",
                "winner_eps_level",
                "solve_mode",
                "max_abs_omega2_int",
                "max_abs_omega2_export",
            ]
        )
        for zt in ZT_ORDER:
            s = summaries[zt]
            writer.writerow(
                [
                    s.case_tag,
                    f"{zt:.1f}",
                    f"{s.vt1:.9f}",
                    f"{s.vt2:.9f}",
                    f"{s.vt_norm:.9f}",
                    f"{-s.vt2:.9f}",
                    f"{s.vt_signed:.9f}",
                    f"{s.duration:.9f}",
                    s.winner_eps_level,
                    s.solve_mode,
                    f"{s.max_abs_omega2_int:.9f}",
                    f"{s.max_abs_omega2_export:.9f}",
                ]
            )


def write_pairwise_tsv(out_path: Path, pair_stats: Dict[Tuple[float, float], PairMetric]) -> None:
    with out_path.open("w", newline="") as f:
        writer = csv.writer(f, delimiter="\t")
        writer.writerow(
            [
                "pair",
                "traj_rmse_norm",
                "tau_rmse_norm",
                "omega_rmse_norm",
                "raw_span_x",
                "raw_span_z",
                "span_x",
                "span_z",
                "raw_mean_abs_union_tau",
                "raw_mean_abs_union_omega",
                "mean_abs_union_tau",
                "mean_abs_union_omega",
            ]
        )
        for za, zb in PAIR_ORDER:
            p = pair_stats[(za, zb)]
            writer.writerow(
                [
                    f"{za:.1f}_{zb:.1f}",
                    f"{p.traj_rmse_norm:.9f}",
                    f"{p.tau_rmse_norm:.9f}",
                    f"{p.omega_rmse_norm:.9f}",
                    f"{p.raw_span_x:.9f}",
                    f"{p.raw_span_z:.9f}",
                    f"{p.span_x:.9f}",
                    f"{p.span_z:.9f}",
                    f"{p.raw_mean_abs_union_tau:.9f}",
                    f"{p.raw_mean_abs_union_omega:.9f}",
                    f"{p.mean_abs_union_tau:.9f}",
                    f"{p.mean_abs_union_omega:.9f}",
                ]
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze Fig.11 mechanism validation outputs")
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--output-summary", required=True)
    parser.add_argument("--output-metrics", required=True)
    parser.add_argument("--output-pairwise", default="")

    parser.add_argument("--norm-epsilon", type=float, default=1e-6)
    parser.add_argument("--resample-count", type=int, default=201)

    parser.add_argument("--adapt-abs-threshold-vt2", type=float, default=0.02)
    parser.add_argument("--adapt-rel-threshold-vt2", type=float, default=0.2)
    parser.add_argument("--adapt-abs-threshold-vtnorm", type=float, default=0.02)
    parser.add_argument("--adapt-rel-threshold-vtnorm", type=float, default=0.2)

    parser.add_argument("--traj-change-threshold", type=float, default=0.02)
    parser.add_argument("--tau-change-threshold", type=float, default=0.03)
    parser.add_argument("--omega-change-threshold", type=float, default=0.03)

    parser.add_argument("--omega-int-threshold", type=float, default=3.0015)
    parser.add_argument("--omega-export-warn-threshold", type=float, default=3.0015)
    parser.add_argument("--runtime-fail-count", type=int, default=0)

    parser.add_argument("--profile-k", default="")
    parser.add_argument("--profile-rho-omega", default="")
    parser.add_argument("--profile-rho-thrust", default="")

    parser.add_argument("--scene-ini-p", default="")
    parser.add_argument("--scene-ini-v", default="")
    parser.add_argument("--scene-perching-p-template", default="")
    parser.add_argument("--scene-perching-v", default="")
    parser.add_argument("--scene-perching-axis", default="")
    parser.add_argument("--scene-perching-theta", default="")
    parser.add_argument("--scene-zt-values", default="2.0,1.5,1.0")
    parser.add_argument(
        "--scene-role",
        default="unified_mechanism_validation_scene_not_claimed_as_exact_paper_scene",
    )

    parser.add_argument(
        "--result-note",
        default="PASS 表示 terminal-state-adjustment 机制在统一场景下成立，不表示论文 Fig.11 原始单标量结果已复现",
    )
    args = parser.parse_args()

    input_dir = Path(args.input_dir).expanduser().resolve()
    output_summary = Path(args.output_summary).expanduser().resolve()
    output_metrics = Path(args.output_metrics).expanduser().resolve()
    output_summary.parent.mkdir(parents=True, exist_ok=True)
    output_metrics.parent.mkdir(parents=True, exist_ok=True)

    summaries: Dict[float, CaseSummary] = {}
    curves: Dict[float, CurveSeries] = {}
    for zt in ZT_ORDER:
        summaries[zt] = read_case_summary(input_dir, zt)
        curves[zt] = read_case_curve(input_dir, zt)

    write_metrics_tsv(output_metrics, summaries)

    resampled: Dict[float, CurveSeries] = {}
    for zt in ZT_ORDER:
        resampled[zt] = normalize_and_resample(curves[zt], args.resample_count, args.norm_epsilon)

    pair_stats: Dict[Tuple[float, float], PairMetric] = {}
    for za, zb in PAIR_ORDER:
        p = pair_metrics(resampled[za], resampled[zb], args.norm_epsilon)
        p.za = za
        p.zb = zb
        pair_stats[(za, zb)] = p

    if args.output_pairwise:
        output_pairwise = Path(args.output_pairwise).expanduser().resolve()
    else:
        output_pairwise = output_summary.parent / "mechanism_pairwise.tsv"
    output_pairwise.parent.mkdir(parents=True, exist_ok=True)
    write_pairwise_tsv(output_pairwise, pair_stats)

    vt2_values = [summaries[zt].vt2 for zt in ZT_ORDER]
    vt_norm_values = [summaries[zt].vt_norm for zt in ZT_ORDER]
    range_vt2 = max(vt2_values) - min(vt2_values)
    range_vt_norm = max(vt_norm_values) - min(vt_norm_values)
    mean_abs_vt2 = mean_abs(vt2_values)
    mean_abs_vt_norm = mean_abs(vt_norm_values)

    rel_vt2 = range_vt2 / max(mean_abs_vt2, args.norm_epsilon)
    rel_vt_norm = range_vt_norm / max(mean_abs_vt_norm, args.norm_epsilon)

    vt2_pass = range_vt2 >= args.adapt_abs_threshold_vt2 and rel_vt2 >= args.adapt_rel_threshold_vt2
    vt_norm_pass = (
        range_vt_norm >= args.adapt_abs_threshold_vtnorm
        and rel_vt_norm >= args.adapt_rel_threshold_vtnorm
    )

    if vt2_pass and vt_norm_pass:
        adaptation_driver = "both"
    elif vt2_pass:
        adaptation_driver = "vt2"
    elif vt_norm_pass:
        adaptation_driver = "vt_norm"
    else:
        adaptation_driver = "none"
    adaptation_gate_pass = vt2_pass or vt_norm_pass

    traj_values = [pair_stats[p].traj_rmse_norm for p in PAIR_ORDER]
    tau_values = [pair_stats[p].tau_rmse_norm for p in PAIR_ORDER]
    omega_values = [pair_stats[p].omega_rmse_norm for p in PAIR_ORDER]

    traj_change_pass = max(traj_values) >= args.traj_change_threshold
    tau_change_pass = max(tau_values) >= args.tau_change_threshold
    omega_change_pass = max(omega_values) >= args.omega_change_threshold
    curve_sync_gate_pass = traj_change_pass and (tau_change_pass or omega_change_pass)

    eps_levels = [summaries[zt].winner_eps_level for zt in ZT_ORDER]
    strict_count = sum(1 for e in eps_levels if e == "strict")
    relaxed_count = sum(1 for e in eps_levels if e == "relaxed")
    fallback_count = sum(1 for e in eps_levels if e == "fallback")
    unknown_eps_count = sum(1 for e in eps_levels if e not in ("strict", "relaxed", "fallback"))

    solve_mode_values = [summaries[zt].solve_mode for zt in ZT_ORDER]
    solve_mode_single_count = sum(1 for s in solve_mode_values if s == "single")
    solve_mode_all_single = solve_mode_single_count == len(ZT_ORDER)

    max_abs_omega2_int_overall = max(summaries[zt].max_abs_omega2_int for zt in ZT_ORDER)
    finite_export_omegas = [
        summaries[zt].max_abs_omega2_export
        for zt in ZT_ORDER
        if math.isfinite(summaries[zt].max_abs_omega2_export)
    ]
    max_abs_omega2_export_overall = max(finite_export_omegas) if finite_export_omegas else float("nan")
    omega_export_warn_count = sum(
        1
        for zt in ZT_ORDER
        if math.isfinite(summaries[zt].max_abs_omega2_export)
        and summaries[zt].max_abs_omega2_export > args.omega_export_warn_threshold
    )

    feasibility_gate_pass = (
        args.runtime_fail_count == 0
        and fallback_count == 0
        and unknown_eps_count == 0
        and solve_mode_all_single
        and max_abs_omega2_int_overall <= args.omega_int_threshold
    )

    overall_pass = feasibility_gate_pass and adaptation_gate_pass and curve_sync_gate_pass

    with output_summary.open("w") as f:
        f.write("analysis_type=fig11_mechanism_validation_v1_1\n")
        f.write(f"input_dir={input_dir}\n")
        f.write(f"overall_result={pass_flag(overall_pass)}\n")
        f.write(f"result_note={args.result_note}\n")
        f.write(f"scene_role={args.scene_role}\n")

        f.write(f"profile_K={args.profile_k}\n")
        f.write(f"profile_rhoOmega={args.profile_rho_omega}\n")
        f.write(f"profile_rhoThrust={args.profile_rho_thrust}\n")

        f.write(f"scene_ini_p={args.scene_ini_p}\n")
        f.write(f"scene_ini_v={args.scene_ini_v}\n")
        f.write(f"scene_perching_p_template={args.scene_perching_p_template}\n")
        f.write(f"scene_perching_v={args.scene_perching_v}\n")
        f.write(f"scene_perching_axis={args.scene_perching_axis}\n")
        f.write(f"scene_perching_theta={args.scene_perching_theta}\n")
        f.write(f"scene_zt_values={args.scene_zt_values}\n")

        f.write(f"norm_epsilon={args.norm_epsilon:.9f}\n")
        f.write(f"resample_count={args.resample_count}\n")

        f.write(f"omega_int_threshold={args.omega_int_threshold:.9f}\n")
        f.write(f"omega_export_warn_threshold={args.omega_export_warn_threshold:.9f}\n")

        f.write(f"strict_count={strict_count}\n")
        f.write(f"relaxed_count={relaxed_count}\n")
        f.write(f"fallback_count={fallback_count}\n")
        f.write(f"unknown_eps_count={unknown_eps_count}\n")
        f.write(f"runtime_fail_count={args.runtime_fail_count}\n")

        f.write(f"solve_mode_single_count={solve_mode_single_count}\n")
        f.write(f"solve_mode_all_single={str(solve_mode_all_single).lower()}\n")

        f.write(f"max_abs_omega2_int_overall={max_abs_omega2_int_overall:.9f}\n")
        f.write(f"max_abs_omega2_export_overall={max_abs_omega2_export_overall:.9f}\n")
        f.write(f"omega_export_warn_count={omega_export_warn_count}\n")

        f.write(f"feasibility_gate={pass_flag(feasibility_gate_pass)}\n")

        f.write(f"adapt_abs_threshold_vt2={args.adapt_abs_threshold_vt2:.9f}\n")
        f.write(f"adapt_rel_threshold_vt2={args.adapt_rel_threshold_vt2:.9f}\n")
        f.write(f"adapt_abs_threshold_vtnorm={args.adapt_abs_threshold_vtnorm:.9f}\n")
        f.write(f"adapt_rel_threshold_vtnorm={args.adapt_rel_threshold_vtnorm:.9f}\n")

        f.write(f"range_vt2={range_vt2:.9f}\n")
        f.write(f"range_vt_norm={range_vt_norm:.9f}\n")
        f.write(f"mean_abs_vt2={mean_abs_vt2:.9f}\n")
        f.write(f"mean_abs_vt_norm={mean_abs_vt_norm:.9f}\n")
        f.write(f"relative_range_vt2={rel_vt2:.9f}\n")
        f.write(f"relative_range_vt_norm={rel_vt_norm:.9f}\n")
        f.write(f"vt2_adaptation_pass={pass_flag(vt2_pass)}\n")
        f.write(f"vtnorm_adaptation_pass={pass_flag(vt_norm_pass)}\n")
        f.write(f"adaptation_driver={adaptation_driver}\n")
        f.write(f"adaptation_gate={pass_flag(adaptation_gate_pass)}\n")

        f.write(f"traj_change_threshold={args.traj_change_threshold:.9f}\n")
        f.write(f"tau_change_threshold={args.tau_change_threshold:.9f}\n")
        f.write(f"omega_change_threshold={args.omega_change_threshold:.9f}\n")

        for za, zb in PAIR_ORDER:
            p = pair_stats[(za, zb)]
            key = f"{za:.1f}_{zb:.1f}"
            f.write(f"traj_rmse_norm_{key}={p.traj_rmse_norm:.9f}\n")
            f.write(f"tau_rmse_norm_{key}={p.tau_rmse_norm:.9f}\n")
            f.write(f"omega_rmse_norm_{key}={p.omega_rmse_norm:.9f}\n")
            f.write(f"raw_span_x_{key}={p.raw_span_x:.9f}\n")
            f.write(f"raw_span_z_{key}={p.raw_span_z:.9f}\n")
            f.write(f"span_x_{key}={p.span_x:.9f}\n")
            f.write(f"span_z_{key}={p.span_z:.9f}\n")
            f.write(f"raw_mean_abs_union_tau_{key}={p.raw_mean_abs_union_tau:.9f}\n")
            f.write(f"raw_mean_abs_union_omega_{key}={p.raw_mean_abs_union_omega:.9f}\n")
            f.write(f"mean_abs_union_tau_{key}={p.mean_abs_union_tau:.9f}\n")
            f.write(f"mean_abs_union_omega_{key}={p.mean_abs_union_omega:.9f}\n")

        f.write(f"traj_change_pass={pass_flag(traj_change_pass)}\n")
        f.write(f"tau_change_pass={pass_flag(tau_change_pass)}\n")
        f.write(f"omega_change_pass={pass_flag(omega_change_pass)}\n")
        f.write(f"curve_sync_gate={pass_flag(curve_sync_gate_pass)}\n")

        for zt in ZT_ORDER:
            s = summaries[zt]
            key = f"{zt:.1f}".replace(".", "_")
            f.write(f"vt1_zt_{key}={s.vt1:.9f}\n")
            f.write(f"vt2_zt_{key}={s.vt2:.9f}\n")
            f.write(f"vt_norm_zt_{key}={s.vt_norm:.9f}\n")
            f.write(f"neg_vt2_zt_{key}={-s.vt2:.9f}\n")
            f.write(f"vt_signed_zt_{key}={s.vt_signed:.9f}\n")
            f.write(f"duration_zt_{key}={s.duration:.9f}\n")
            f.write(f"winner_eps_level_zt_{key}={s.winner_eps_level}\n")
            f.write(f"max_abs_omega2_int_zt_{key}={s.max_abs_omega2_int:.9f}\n")

    print(f"Wrote metrics: {output_metrics}")
    print(f"Wrote pairwise: {output_pairwise}")
    print(f"Wrote summary: {output_summary}")


if __name__ == "__main__":
    main()
