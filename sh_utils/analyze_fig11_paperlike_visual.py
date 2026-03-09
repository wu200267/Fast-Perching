#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path
from typing import Dict, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_summary(path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or "=" not in line:
            continue
        k, v = line.split("=", 1)
        data[k.strip()] = v.strip()
    return data


def parse_csv(path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                {
                    "t": float(row["t"]),
                    "x": float(row["x"]),
                    "z": float(row["z"]),
                    "tau_norm": float(row["tau_norm"]),
                    "omega2": float(row["omega2"]),
                }
            )
    return rows


def grade_trajectory(cases: List[Dict]) -> (str, Dict[str, bool]):
    starts = [(c["x_start"], c["z_start"]) for c in cases]
    x_starts = [s[0] for s in starts]
    z_starts = [s[1] for s in starts]

    cond_start_aligned = (max(x_starts) - min(x_starts) <= 0.25) and (max(z_starts) - min(z_starts) <= 0.25)
    cond_end_x_aligned = (max(c["x_end"] for c in cases) - min(c["x_end"] for c in cases) <= 0.35)
    cond_end_z_order = all(cases[i]["z_end"] > cases[i + 1]["z_end"] for i in range(len(cases) - 1))
    cond_min_z_order = all(cases[i]["z_min"] > cases[i + 1]["z_min"] for i in range(len(cases) - 1))
    cond_all_have_dip = all(c["z_min"] <= min(c["z_start"], c["z_end"]) - 0.08 for c in cases)
    cond_min_x_interior = all(
        min(c["x_start"], c["x_end"]) + 0.15 <= c["x_at_z_min"] <= max(c["x_start"], c["x_end"]) - 0.15
        for c in cases
    )

    checks = {
        "start_aligned": cond_start_aligned,
        "end_x_aligned": cond_end_x_aligned,
        "end_z_order": cond_end_z_order,
        "min_z_order": cond_min_z_order,
        "all_have_dip": cond_all_have_dip,
        "min_x_interior": cond_min_x_interior,
    }
    score = sum(1 for v in checks.values() if v)
    if score >= 5:
        return "high", checks
    if score >= 3:
        return "medium", checks
    return "low", checks


def grade_thrust(cases: List[Dict]) -> str:
    high = all(abs(c["tau_min"] - 5.0) <= 0.4 and abs(c["tau_max"] - 17.0) <= 0.4 for c in cases)
    if high:
        return "high"
    medium = all(abs(c["tau_min"] - 5.0) <= 1.0 and abs(c["tau_max"] - 17.0) <= 1.0 for c in cases)
    return "medium" if medium else "low"


def grade_omega(cases: List[Dict]) -> str:
    high = all(c["omega_max"] >= 2.7 and c["omega_min"] <= -2.7 for c in cases)
    if high:
        return "high"
    medium = all(c["omega_max"] >= 2.2 and c["omega_min"] <= -2.2 for c in cases)
    return "medium" if medium else "low"


def grade_overall(trajectory: str, thrust: str, omega: str) -> str:
    grades = [trajectory, thrust, omega]
    if grades.count("high") == 3:
        return "high"
    if grades.count("low") == 0 and grades.count("medium") >= 2:
        return "medium"
    return "low"


def make_side_by_side(paper_crop: Path, fig11_reproduced: Path, output: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    if paper_crop.exists():
        paper_img = plt.imread(str(paper_crop))
        axes[0].imshow(paper_img)
        axes[0].set_title("Paper Fig.11 (crop)")
    else:
        axes[0].text(0.5, 0.5, f"Missing\\n{paper_crop}", ha="center", va="center")
        axes[0].set_title("Paper Fig.11 (crop missing)")
    axes[0].axis("off")

    if fig11_reproduced.exists():
        ours_img = plt.imread(str(fig11_reproduced))
        axes[1].imshow(ours_img)
        axes[1].set_title("Paper-like Best Effort")
    else:
        axes[1].text(0.5, 0.5, f"Missing\\n{fig11_reproduced}", ha="center", va="center")
        axes[1].set_title("Reproduced figure missing")
    axes[1].axis("off")

    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=220)


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze paper-like visual reproduction metrics.")
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--paper-crop", default="fig11_outputs/vt_mapping_dual_baseline/paper_fig11_crop.png")
    parser.add_argument("--review-out", default="best_effort_visual_review.txt")
    parser.add_argument("--side-by-side-out", default="fig11_side_by_side_paperlike.png")
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    paper_crop = Path(args.paper_crop)
    review_out = Path(args.review_out)
    if not review_out.is_absolute():
        review_out = input_dir / review_out
    side_by_side_out = Path(args.side_by_side_out)
    if not side_by_side_out.is_absolute():
        side_by_side_out = input_dir / side_by_side_out

    cases: List[Dict] = []
    summary_files = sorted(input_dir.glob("zt_*_summary.txt"))
    if not summary_files:
        raise SystemExit(f"No summary files found in {input_dir}")

    for summary_file in summary_files:
        case_tag = summary_file.stem.replace("_summary", "")
        csv_file = input_dir / f"{case_tag}.csv"
        if not csv_file.exists():
            continue

        s = parse_summary(summary_file)
        rows = parse_csv(csv_file)
        if not rows:
            continue

        x_values = [r["x"] for r in rows]
        z_values = [r["z"] for r in rows]
        tau_values = [r["tau_norm"] for r in rows]
        omega_values = [r["omega2"] for r in rows]

        min_idx = min(range(len(z_values)), key=lambda i: z_values[i])
        vt2 = float(s.get("vt2", "0.0"))

        case = {
            "case_tag": case_tag,
            "zt": float(s.get("perching_pz", "0.0")),
            "tau_min": min(tau_values),
            "tau_max": max(tau_values),
            "omega_min": min(omega_values),
            "omega_max": max(omega_values),
            "x_min": min(x_values),
            "x_max": max(x_values),
            "x_range": max(x_values) - min(x_values),
            "z_min": min(z_values),
            "z_max": max(z_values),
            "z_range": max(z_values) - min(z_values),
            "x_start": x_values[0],
            "z_start": z_values[0],
            "x_end": x_values[-1],
            "z_end": z_values[-1],
            "x_at_z_min": x_values[min_idx],
            "vt_signed": float(s.get("vt_signed", "0.0")),
            "neg_vt2": -vt2,
            "vt2": vt2,
        }
        cases.append(case)

    if len(cases) != 3:
        raise SystemExit(f"Expected 3 cases, got {len(cases)} in {input_dir}")

    cases.sort(key=lambda c: c["zt"], reverse=True)

    trajectory_grade, trajectory_checks = grade_trajectory(cases)
    thrust_grade = grade_thrust(cases)
    omega_grade = grade_omega(cases)
    overall_grade = grade_overall(trajectory_grade, thrust_grade, omega_grade)

    make_side_by_side(paper_crop, input_dir / "fig11_reproduced.png", side_by_side_out)

    lines: List[str] = []
    lines.append("workflow=paperlike_visual_reproduction")
    lines.append("main_target=visual_similarity_to_paper_fig11")
    lines.append("note=This is a visual similarity grade, not a mechanism proof and not an exact numeric reproduction.")
    lines.append("")

    lines.append(f"trajectory_similarity={trajectory_grade}")
    lines.append(f"thrust_boundary_similarity={thrust_grade}")
    lines.append(f"omega_shape_similarity={omega_grade}")
    lines.append(f"overall_visual_similarity={overall_grade}")
    lines.append("")

    lines.append("[trajectory_checks]")
    for k, v in trajectory_checks.items():
        lines.append(f"{k}={'PASS' if v else 'FAIL'}")
    lines.append("")

    lines.append("[per_case_metrics]")
    for c in cases:
        zt_label = f"{c['zt']:.1f}"
        lines.append(
            " ".join(
                [
                    f"case=zt_{zt_label}",
                    f"tau_norm_min={c['tau_min']:.6f}",
                    f"tau_norm_max={c['tau_max']:.6f}",
                    f"omega2_min={c['omega_min']:.6f}",
                    f"omega2_max={c['omega_max']:.6f}",
                    f"x_range={c['x_range']:.6f}",
                    f"z_range={c['z_range']:.6f}",
                    f"x_end={c['x_end']:.6f}",
                    f"z_end={c['z_end']:.6f}",
                    f"vt_signed={c['vt_signed']:.6f}",
                    f"neg_vt2={c['neg_vt2']:.6f}",
                ]
            )
        )

    lines.append("")
    lines.append(f"paper_crop={paper_crop}")
    lines.append(f"reproduced_fig={input_dir / 'fig11_reproduced.png'}")
    lines.append(f"side_by_side_fig={side_by_side_out}")

    review_out.parent.mkdir(parents=True, exist_ok=True)
    review_out.write_text("\n".join(lines) + "\n")
    print(f"[analyze_fig11_paperlike_visual] wrote: {review_out}")
    print(f"[analyze_fig11_paperlike_visual] wrote: {side_by_side_out}")


if __name__ == "__main__":
    main()
