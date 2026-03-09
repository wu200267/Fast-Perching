#!/usr/bin/env python3

import argparse
import csv
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


REQUIRED_SUMMARY_KEYS = [
    "vt1",
    "vt2",
    "vt_norm",
    "vt_signed",
    "v1",
    "v2",
    "duration",
    "winner_eps_level",
    "max_abs_omega2_int",
]

PAPER_VT_BY_ZT = {
    2.0: -0.388,
    1.5: -0.049,
    1.0: -0.022,
}
PAPER_MEAN_ABS_VALUE = sum(abs(v) for v in PAPER_VT_BY_ZT.values()) / len(PAPER_VT_BY_ZT)

PRIMARY_CANDIDATES = ("vt2", "neg_vt2", "vt_norm")
CANDIDATE_SPECS = [
    ("vt2", "primary", "signed_candidate"),
    ("neg_vt2", "primary", "signed_candidate"),
    ("vt_norm", "primary", "unsigned_candidate"),
    ("vt1", "lateral_slip_audit_only", "signed_candidate"),
    ("vt_signed", "legacy_projection_proxy_only", "signed_candidate"),
]


@dataclass
class BaselineCase:
    zt: float
    vt1: float
    vt2: float
    vt_norm: float
    vt_signed: float
    v1: str
    v2: str
    duration: float
    winner_eps_level: str
    max_abs_omega2_int: float


@dataclass
class CandidateMetric:
    baseline_id: str
    baseline_dir: str
    baseline_feasibility_status: str
    candidate: str
    candidate_role: str
    candidate_sign_type: str
    q20: float
    q15: float
    q10: float
    monotonic_deficit: float
    monotonic_margin: float
    sign_match_count: int
    sign_mismatch_count: int
    mean_abs_value: float
    paper_mean_abs_value: float
    magnitude_scale_ratio: float
    mae_to_paper: float
    rmse_to_paper: float
    err20: float
    err15: float
    err10: float
    baseline_primary_rank: int
    aggregate_primary_rank: int


@dataclass
class BaselineCaseRow:
    baseline_id: str
    baseline_dir: str
    baseline_feasibility_status: str
    zt: float
    vt1: float
    vt2: float
    vt_norm: float
    vt_signed: float
    v1: str
    v2: str
    duration: float
    winner_eps_level: str
    max_abs_omega2_int: float


@dataclass
class AggregateRow:
    candidate: str
    aggregate_primary_rank: int
    baseline_count: int
    sign_match_count_total: int
    sign_mismatch_count_total: int
    sign_mismatch_count_mean: float
    magnitude_scale_gap_mean: float
    mae_to_paper_mean: float
    rmse_to_paper_mean: float
    monotonic_deficit_mean: float
    monotonic_margin_mean: float


def parse_summary(path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or "=" not in line:
            continue
        k, v = line.split("=", 1)
        data[k.strip()] = v.strip()
    return data


def load_baseline_cases(baseline_dir: Path) -> Dict[float, BaselineCase]:
    cases: Dict[float, BaselineCase] = {}
    for zt in [2.0, 1.5, 1.0]:
        case_tag = f"zt_{str(zt).replace('.', '_')}"
        summary_path = baseline_dir / f"{case_tag}_summary.txt"
        if not summary_path.exists():
            raise RuntimeError(f"Missing summary: {summary_path}")
        raw = parse_summary(summary_path)
        for key in REQUIRED_SUMMARY_KEYS:
            if key not in raw:
                raise RuntimeError(f"Missing key '{key}' in {summary_path}")

        case = BaselineCase(
            zt=zt,
            vt1=float(raw["vt1"]),
            vt2=float(raw["vt2"]),
            vt_norm=float(raw["vt_norm"]),
            vt_signed=float(raw["vt_signed"]),
            v1=raw["v1"],
            v2=raw["v2"],
            duration=float(raw["duration"]),
            winner_eps_level=raw["winner_eps_level"],
            max_abs_omega2_int=float(raw["max_abs_omega2_int"]),
        )
        cases[zt] = case
    return cases


def baseline_feasibility_status(cases: Dict[float, BaselineCase]) -> str:
    all_strict = all(c.winner_eps_level == "strict" for c in cases.values())
    return "all_strict" if all_strict else "has_relaxed"


def candidate_values(name: str, cases: Dict[float, BaselineCase]) -> Tuple[float, float, float]:
    c20, c15, c10 = cases[2.0], cases[1.5], cases[1.0]
    if name == "vt1":
        return c20.vt1, c15.vt1, c10.vt1
    if name == "vt2":
        return c20.vt2, c15.vt2, c10.vt2
    if name == "neg_vt2":
        return -c20.vt2, -c15.vt2, -c10.vt2
    if name == "vt_norm":
        return c20.vt_norm, c15.vt_norm, c10.vt_norm
    if name == "vt_signed":
        return c20.vt_signed, c15.vt_signed, c10.vt_signed
    raise RuntimeError(f"Unknown candidate: {name}")


def monotonic_metrics(q20: float, q15: float, q10: float) -> Tuple[float, float]:
    # Target trend from paper caption text: q20 < q15 < q10
    d1 = max(0.0, q20 - q15)
    d2 = max(0.0, q15 - q10)
    deficit = d1 + d2
    margin = min(q15 - q20, q10 - q15)
    return deficit, margin


def sign_class(x: float, eps: float) -> int:
    if x > eps:
        return 1
    if x < -eps:
        return -1
    return 0


def candidate_ranking_key(m: CandidateMetric) -> Tuple[float, float, float, float, float, str]:
    return (
        m.sign_mismatch_count,
        abs(m.magnitude_scale_ratio - 1.0),
        m.mae_to_paper,
        m.monotonic_deficit,
        -m.monotonic_margin,
        m.candidate,
    )


def compute_candidate_metric(
    baseline_id: str,
    baseline_dir: str,
    baseline_feasibility_status: str,
    candidate: str,
    role: str,
    sign_type: str,
    cases: Dict[float, BaselineCase],
    sign_eps: float,
) -> CandidateMetric:
    q20, q15, q10 = candidate_values(candidate, cases)
    deficit, margin = monotonic_metrics(q20, q15, q10)

    e20 = q20 - PAPER_VT_BY_ZT[2.0]
    e15 = q15 - PAPER_VT_BY_ZT[1.5]
    e10 = q10 - PAPER_VT_BY_ZT[1.0]
    mae = (abs(e20) + abs(e15) + abs(e10)) / 3.0
    rmse = math.sqrt((e20 * e20 + e15 * e15 + e10 * e10) / 3.0)

    mean_abs = (abs(q20) + abs(q15) + abs(q10)) / 3.0
    scale = mean_abs / PAPER_MEAN_ABS_VALUE if PAPER_MEAN_ABS_VALUE > 1e-12 else float("inf")

    paper_signs = [
        sign_class(PAPER_VT_BY_ZT[2.0], sign_eps),
        sign_class(PAPER_VT_BY_ZT[1.5], sign_eps),
        sign_class(PAPER_VT_BY_ZT[1.0], sign_eps),
    ]
    candidate_signs = [
        sign_class(q20, sign_eps),
        sign_class(q15, sign_eps),
        sign_class(q10, sign_eps),
    ]
    sign_match_count = sum(1 for ps, cs in zip(paper_signs, candidate_signs) if ps == cs)
    sign_mismatch_count = 3 - sign_match_count

    return CandidateMetric(
        baseline_id=baseline_id,
        baseline_dir=baseline_dir,
        baseline_feasibility_status=baseline_feasibility_status,
        candidate=candidate,
        candidate_role=role,
        candidate_sign_type=sign_type,
        q20=q20,
        q15=q15,
        q10=q10,
        monotonic_deficit=deficit,
        monotonic_margin=margin,
        sign_match_count=sign_match_count,
        sign_mismatch_count=sign_mismatch_count,
        mean_abs_value=mean_abs,
        paper_mean_abs_value=PAPER_MEAN_ABS_VALUE,
        magnitude_scale_ratio=scale,
        mae_to_paper=mae,
        rmse_to_paper=rmse,
        err20=e20,
        err15=e15,
        err10=e10,
        baseline_primary_rank=0,
        aggregate_primary_rank=0,
    )


def rank_primary(metrics: List[CandidateMetric]) -> List[CandidateMetric]:
    prim = [m for m in metrics if m.candidate_role == "primary"]
    prim_sorted = sorted(prim, key=candidate_ranking_key)
    for idx, m in enumerate(prim_sorted, start=1):
        m.baseline_primary_rank = idx
    return prim_sorted


def aggregate_primary_rows(primary_rows: List[CandidateMetric]) -> List[AggregateRow]:
    by_cand: Dict[str, List[CandidateMetric]] = {}
    for row in primary_rows:
        by_cand.setdefault(row.candidate, []).append(row)

    agg_rows: List[AggregateRow] = []
    for cand in PRIMARY_CANDIDATES:
        rows = by_cand.get(cand, [])
        if not rows:
            continue
        n = len(rows)
        sign_match_total = sum(r.sign_match_count for r in rows)
        sign_mismatch_total = sum(r.sign_mismatch_count for r in rows)
        agg_rows.append(
            AggregateRow(
                candidate=cand,
                aggregate_primary_rank=0,
                baseline_count=n,
                sign_match_count_total=sign_match_total,
                sign_mismatch_count_total=sign_mismatch_total,
                sign_mismatch_count_mean=sign_mismatch_total / n,
                magnitude_scale_gap_mean=sum(abs(r.magnitude_scale_ratio - 1.0) for r in rows) / n,
                mae_to_paper_mean=sum(r.mae_to_paper for r in rows) / n,
                rmse_to_paper_mean=sum(r.rmse_to_paper for r in rows) / n,
                monotonic_deficit_mean=sum(r.monotonic_deficit for r in rows) / n,
                monotonic_margin_mean=sum(r.monotonic_margin for r in rows) / n,
            )
        )

    agg_rows.sort(
        key=lambda r: (
            r.sign_mismatch_count_mean,
            r.magnitude_scale_gap_mean,
            r.mae_to_paper_mean,
            r.monotonic_deficit_mean,
            -r.monotonic_margin_mean,
            r.candidate,
        )
    )
    for idx, row in enumerate(agg_rows, start=1):
        row.aggregate_primary_rank = idx
    return agg_rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze Fig.11 v_t mapping candidates across baselines (v3).")
    parser.add_argument(
        "--baseline",
        action="append",
        required=True,
        help="Baseline descriptor in format <id>:<dir>, e.g. A:/tmp/runA",
    )
    parser.add_argument("--out-dir", required=True, help="Output directory for v3 mapping report/summary")
    parser.add_argument("--weak-margin-eps", type=float, default=1e-3)
    parser.add_argument("--sign-epsilon", type=float, default=1e-6)
    parser.add_argument(
        "--latest-pointer-dir",
        default="",
        help="Optional legacy entry directory to write LATEST_ANALYSIS_POINTER.txt",
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    parsed: List[Tuple[str, Path]] = []
    for desc in args.baseline:
        if ":" not in desc:
            raise RuntimeError(f"Invalid --baseline '{desc}', expected <id>:<dir>")
        bid, bdir = desc.split(":", 1)
        parsed.append((bid.strip(), Path(bdir).expanduser().resolve()))

    report_rows: List[CandidateMetric] = []
    case_rows: List[BaselineCaseRow] = []
    baseline_best: Dict[str, CandidateMetric] = {}
    baseline_status: Dict[str, str] = {}

    for bid, bdir in parsed:
        cases = load_baseline_cases(bdir)
        feas = baseline_feasibility_status(cases)
        baseline_status[bid] = feas

        for zt in [2.0, 1.5, 1.0]:
            case = cases[zt]
            case_rows.append(
                BaselineCaseRow(
                    baseline_id=bid,
                    baseline_dir=str(bdir),
                    baseline_feasibility_status=feas,
                    zt=case.zt,
                    vt1=case.vt1,
                    vt2=case.vt2,
                    vt_norm=case.vt_norm,
                    vt_signed=case.vt_signed,
                    v1=case.v1,
                    v2=case.v2,
                    duration=case.duration,
                    winner_eps_level=case.winner_eps_level,
                    max_abs_omega2_int=case.max_abs_omega2_int,
                )
            )

        local_metrics: List[CandidateMetric] = []
        for candidate, role, sign_type in CANDIDATE_SPECS:
            local_metrics.append(
                compute_candidate_metric(
                    baseline_id=bid,
                    baseline_dir=str(bdir),
                    baseline_feasibility_status=feas,
                    candidate=candidate,
                    role=role,
                    sign_type=sign_type,
                    cases=cases,
                    sign_eps=args.sign_epsilon,
                )
            )

        prim_sorted = rank_primary(local_metrics)
        if not prim_sorted:
            raise RuntimeError(f"No primary candidates for baseline {bid}")
        baseline_best[bid] = prim_sorted[0]
        report_rows.extend(local_metrics)

    primary_rows = [r for r in report_rows if r.candidate_role == "primary"]
    agg_rows = aggregate_primary_rows(primary_rows)
    if not agg_rows:
        raise RuntimeError("No aggregate primary rows computed")
    aggregate_rank_map = {r.candidate: r.aggregate_primary_rank for r in agg_rows}
    for row in report_rows:
        if row.candidate_role == "primary":
            row.aggregate_primary_rank = aggregate_rank_map[row.candidate]

    recommended = agg_rows[0].candidate

    b_ids = [bid for bid, _ in parsed]
    if len(b_ids) < 2:
        consistent = "yes"
    else:
        best_set = {baseline_best[bid].candidate for bid in b_ids}
        consistent = "yes" if len(best_set) == 1 else "no"

    # Paper-fit strength classification.
    # "strong": consistent + low mismatch + clear MAE advantage against runner-up in each baseline.
    # "weak": consistent + non-negative MAE advantage.
    # else "relative_only".
    strong_mae_adv_min = 1e-2
    rec_rows_by_baseline = {r.baseline_id: r for r in primary_rows if r.candidate == recommended}
    per_baseline_sorted: Dict[str, List[CandidateMetric]] = {}
    for bid in b_ids:
        rows = [r for r in primary_rows if r.baseline_id == bid]
        rows.sort(key=candidate_ranking_key)
        per_baseline_sorted[bid] = rows

    mae_adv_all: List[float] = []
    rec_mismatch_all: List[int] = []
    for bid in b_ids:
        rows = per_baseline_sorted[bid]
        rec_row = rec_rows_by_baseline[bid]
        runner_up = next((r for r in rows if r.candidate != recommended), None)
        mae_adv = 0.0 if runner_up is None else (runner_up.mae_to_paper - rec_row.mae_to_paper)
        mae_adv_all.append(mae_adv)
        rec_mismatch_all.append(rec_row.sign_mismatch_count)

    if consistent == "yes" and all(m <= 1 for m in rec_mismatch_all) and all(a >= strong_mae_adv_min for a in mae_adv_all):
        paper_fit_status = "strong"
    elif consistent == "yes" and all(a >= 0.0 for a in mae_adv_all):
        paper_fit_status = "weak"
    else:
        paper_fit_status = "relative_only"

    if paper_fit_status == "strong":
        confidence = "high"
    elif paper_fit_status == "weak":
        confidence = "medium"
    else:
        confidence = "low"

    report_path = out_dir / "vt_mapping_report_v3.tsv"
    with report_path.open("w", newline="") as f:
        writer = csv.writer(f, delimiter="\t")
        writer.writerow(
            [
                "baseline_id",
                "baseline_dir",
                "baseline_feasibility_status",
                "candidate",
                "candidate_role",
                "candidate_sign_type",
                "q_zt_2_0",
                "q_zt_1_5",
                "q_zt_1_0",
                "sign_match_count",
                "sign_mismatch_count",
                "mean_abs_value",
                "paper_mean_abs_value",
                "magnitude_scale_ratio",
                "mae_to_paper",
                "rmse_to_paper",
                "err_zt_2_0",
                "err_zt_1_5",
                "err_zt_1_0",
                "monotonic_deficit",
                "monotonic_margin",
                "baseline_primary_rank",
                "aggregate_primary_rank",
            ]
        )
        for row in sorted(
            report_rows,
            key=lambda r: (
                r.baseline_id,
                r.candidate_role != "primary",
                r.baseline_primary_rank or 999,
                r.candidate,
            ),
        ):
            writer.writerow(
                [
                    row.baseline_id,
                    row.baseline_dir,
                    row.baseline_feasibility_status,
                    row.candidate,
                    row.candidate_role,
                    row.candidate_sign_type,
                    f"{row.q20:.9f}",
                    f"{row.q15:.9f}",
                    f"{row.q10:.9f}",
                    row.sign_match_count,
                    row.sign_mismatch_count,
                    f"{row.mean_abs_value:.9f}",
                    f"{row.paper_mean_abs_value:.9f}",
                    f"{row.magnitude_scale_ratio:.9f}",
                    f"{row.mae_to_paper:.9f}",
                    f"{row.rmse_to_paper:.9f}",
                    f"{row.err20:.9f}",
                    f"{row.err15:.9f}",
                    f"{row.err10:.9f}",
                    f"{row.monotonic_deficit:.9f}",
                    f"{row.monotonic_margin:.9f}",
                    row.baseline_primary_rank if row.candidate_role == "primary" else "",
                    row.aggregate_primary_rank if row.candidate_role == "primary" else "",
                ]
            )

    dual_metrics_path = out_dir / "dual_baseline_metrics_v3.tsv"
    with dual_metrics_path.open("w", newline="") as f:
        writer = csv.writer(f, delimiter="\t")
        writer.writerow(
            [
                "baseline_id",
                "baseline_dir",
                "baseline_feasibility_status",
                "zt",
                "vt1",
                "vt2",
                "vt_norm",
                "vt_signed",
                "v1",
                "v2",
                "duration",
                "winner_eps_level",
                "max_abs_omega2_int",
            ]
        )
        for row in sorted(case_rows, key=lambda r: (r.baseline_id, -r.zt)):
            writer.writerow(
                [
                    row.baseline_id,
                    row.baseline_dir,
                    row.baseline_feasibility_status,
                    f"{row.zt:.1f}",
                    f"{row.vt1:.9f}",
                    f"{row.vt2:.9f}",
                    f"{row.vt_norm:.9f}",
                    f"{row.vt_signed:.9f}",
                    row.v1,
                    row.v2,
                    f"{row.duration:.9f}",
                    row.winner_eps_level,
                    f"{row.max_abs_omega2_int:.9f}",
                ]
            )

    aggregate_path = out_dir / "vt_mapping_aggregate_v3.tsv"
    with aggregate_path.open("w", newline="") as f:
        writer = csv.writer(f, delimiter="\t")
        writer.writerow(
            [
                "candidate",
                "aggregate_primary_rank",
                "baseline_count",
                "sign_match_count_total",
                "sign_mismatch_count_total",
                "sign_mismatch_count_mean",
                "magnitude_scale_gap_mean",
                "mae_to_paper_mean",
                "rmse_to_paper_mean",
                "monotonic_deficit_mean",
                "monotonic_margin_mean",
            ]
        )
        for row in agg_rows:
            writer.writerow(
                [
                    row.candidate,
                    row.aggregate_primary_rank,
                    row.baseline_count,
                    row.sign_match_count_total,
                    row.sign_mismatch_count_total,
                    f"{row.sign_mismatch_count_mean:.9f}",
                    f"{row.magnitude_scale_gap_mean:.9f}",
                    f"{row.mae_to_paper_mean:.9f}",
                    f"{row.rmse_to_paper_mean:.9f}",
                    f"{row.monotonic_deficit_mean:.9f}",
                    f"{row.monotonic_margin_mean:.9f}",
                ]
            )

    summary_path = out_dir / "vt_mapping_summary_v3.txt"
    with summary_path.open("w") as f:
        f.write("analysis_version=v3\n")
        f.write(f"sign_epsilon={args.sign_epsilon:.9f}\n")
        f.write(f"weak_margin_eps={args.weak_margin_eps:.9f}\n")
        f.write(f"paper_vt_zt_2_0={PAPER_VT_BY_ZT[2.0]:.9f}\n")
        f.write(f"paper_vt_zt_1_5={PAPER_VT_BY_ZT[1.5]:.9f}\n")
        f.write(f"paper_vt_zt_1_0={PAPER_VT_BY_ZT[1.0]:.9f}\n")
        f.write(f"paper_mean_abs_value={PAPER_MEAN_ABS_VALUE:.9f}\n")
        f.write("paper_scalar_assumption=single_scalar_projection_of_2d_vt\n")
        f.write("primary_candidate_pool=vt2,neg_vt2,vt_norm\n")
        f.write("vt1_role=lateral_slip_audit_only\n")
        f.write("vt_signed_role=legacy_projection_proxy_only\n")
        f.write("vt_norm_sign_note=unsigned_candidate_sign_mismatch_is_expected_not_bug\n")

        for bid in b_ids:
            best = baseline_best[bid]
            f.write(f"baseline_{bid}_feasibility_status={baseline_status[bid]}\n")
            f.write(f"baseline_{bid}_best_primary_candidate={best.candidate}\n")
            f.write(f"baseline_{bid}_best_sign_mismatch_count={best.sign_mismatch_count}\n")
            f.write(f"baseline_{bid}_best_magnitude_scale_ratio={best.magnitude_scale_ratio:.9f}\n")
            f.write(f"baseline_{bid}_best_mae_to_paper={best.mae_to_paper:.9f}\n")
            f.write(f"baseline_{bid}_best_rmse_to_paper={best.rmse_to_paper:.9f}\n")
            f.write(f"baseline_{bid}_best_monotonic_deficit={best.monotonic_deficit:.9f}\n")
            f.write(f"baseline_{bid}_best_monotonic_margin={best.monotonic_margin:.9f}\n")

        f.write(f"consistent_across_baselines={consistent}\n")
        f.write(f"recommended_primary_candidate={recommended}\n")
        f.write(f"recommendation_confidence={confidence}\n")
        f.write(f"paper_fit_status={paper_fit_status}\n")
        f.write("final_interpretation=vt1_not_fig11_primary_mechanism\n")
        f.write("recommendation_basis=paper_sign_scale_proximity_plus_monotonicity_primary_only\n")
        f.write("\n")
        f.write("natural_language_note_1=vt1_is_lateral_slip_component_not_fig11_primary_mechanism.\n")
        f.write("natural_language_note_2=vt_signed_is_projection_proxy_not_vt_itself.\n")
        f.write("natural_language_note_3=primary_scalar_candidate_should_be_chosen_from_vt2_neg_vt2_vt_norm.\n")
        f.write("natural_language_note_4=recommendation_depends_on_scalar_projection_assumption_for_2d_vt.\n")

    if args.latest_pointer_dir:
        pointer_dir = Path(args.latest_pointer_dir).expanduser().resolve()
        pointer_dir.mkdir(parents=True, exist_ok=True)
        pointer_path = pointer_dir / "LATEST_ANALYSIS_POINTER.txt"
        rel = str(Path(os.path.relpath(out_dir, pointer_dir)))
        with pointer_path.open("w") as f:
            f.write(f"latest_analysis_dir={rel}\n")
            f.write(f"latest_analysis_summary={rel}/vt_mapping_summary_v3.txt\n")
            f.write("note=directory_level_entrypoint; summary_is_current_stage_not_frozen.\n")

    print(f"[vt_mapping_v3] dual metrics: {dual_metrics_path}")
    print(f"[vt_mapping_v3] report: {report_path}")
    print(f"[vt_mapping_v3] aggregate: {aggregate_path}")
    print(f"[vt_mapping_v3] summary: {summary_path}")


if __name__ == "__main__":
    main()
