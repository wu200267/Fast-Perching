#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def safe_float(raw: str, default: float = 0.0) -> float:
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


def parse_summary(path: Path):
    data = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or "=" not in line:
            continue
        k, v = line.split("=", 1)
        data[k.strip()] = v.strip()
    return data


def parse_csv(path: Path):
    rows = []
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                {
                    "t": float(row["t"]),
                    "x": float(row["x"]),
                    "z": float(row["z"]),
                    "tau_norm2": float(row["tau_norm2"]),
                    "tau_norm": float(row["tau_norm"]),
                    "omega2": float(row["omega2"]),
                }
            )
    return rows


def main():
    parser = argparse.ArgumentParser(description="Plot Fig.11 reproduction from CSV/summary files.")
    parser.add_argument("--input-dir", required=True, help="Directory containing *_summary.txt and *.csv")
    parser.add_argument("--output", required=True, help="Output PNG path")
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    summary_files = sorted(input_dir.glob("zt_*_summary.txt"))
    if not summary_files:
        raise SystemExit(f"No summary files found in {input_dir}")

    cases = []
    for summary_file in summary_files:
        case_tag = summary_file.stem.replace("_summary", "")
        csv_file = input_dir / f"{case_tag}.csv"
        if not csv_file.exists():
            raise SystemExit(f"Missing CSV for case {case_tag}: {csv_file}")
        summary = parse_summary(summary_file)
        rows = parse_csv(csv_file)
        if not rows:
            raise SystemExit(f"CSV is empty: {csv_file}")
        zt = float(summary.get("perching_pz", "0.0"))
        cases.append(
            {
                "tag": case_tag,
                "zt": zt,
                "summary": summary,
                "rows": rows,
            }
        )

    cases.sort(key=lambda c: c["zt"], reverse=True)

    fig, axs = plt.subplots(2, 2, figsize=(13, 8))
    ax_xz = axs[0, 0]
    ax_tau = axs[0, 1]
    ax_omega = axs[1, 0]
    ax_table = axs[1, 1]

    colors = ["tab:blue", "tab:orange", "tab:green"]
    for idx, case in enumerate(cases):
        color = colors[idx % len(colors)]
        rows = case["rows"]
        t = [r["t"] for r in rows]
        x = [r["x"] for r in rows]
        z = [r["z"] for r in rows]
        tau_norm = [r["tau_norm"] for r in rows]
        omega2 = [r["omega2"] for r in rows]
        label = rf"$z_t={case['zt']:.1f}$"
        ax_xz.plot(x, z, color=color, linewidth=2.0, label=label)
        ax_tau.plot(t, tau_norm, color=color, linewidth=2.0, label=label)
        ax_omega.plot(t, omega2, color=color, linewidth=2.0, label=label)

    ax_xz.set_title(r"Trajectory in $x$-$z$ Plane")
    ax_xz.set_xlabel("x (m)")
    ax_xz.set_ylabel("z (m)")
    ax_xz.grid(True, alpha=0.25)
    ax_xz.legend()

    ax_tau.set_title(r"Thrust Profile ($\|\tau\|_2$)")
    ax_tau.set_xlabel("t (s)")
    ax_tau.set_ylabel(r"$\|\tau\|_2$ (m/s$^2$)")
    ax_tau.axhline(5.0, color="black", linestyle="--", linewidth=1.2, alpha=0.8)
    ax_tau.axhline(17.0, color="black", linestyle="--", linewidth=1.2, alpha=0.8)
    ax_tau.set_ylim(0.0, 20.0)
    ax_tau.grid(True, alpha=0.25)
    ax_tau.legend()

    ax_omega.set_title(r"Body-Rate Component ($\omega_2$)")
    ax_omega.set_xlabel("t (s)")
    ax_omega.set_ylabel(r"$\omega_2$ (rad/s)")
    ax_omega.axhline(3.0, color="black", linestyle="--", linewidth=1.2, alpha=0.8)
    ax_omega.axhline(-3.0, color="black", linestyle="--", linewidth=1.2, alpha=0.8)
    ax_omega.set_ylim(-3.5, 3.5)
    ax_omega.grid(True, alpha=0.25)
    ax_omega.legend()

    ax_table.axis("off")
    ax_table.set_title(r"$z_t$-$v_t$ Summary", loc="left", pad=3.0)
    col_labels = [
        r"$z_t$ (m)",
        r"$v_t^{\mathrm{signed}}$",
        r"$v_t^{\mathrm{primary}}$",
        "axis",
        r"$v_{t1}$",
        r"$v_{t2}$",
        r"$\|v_t\|_2$",
    ]
    cell_text = []
    for case in cases:
        s = case["summary"]
        cell_text.append(
            [
                f"{case['zt']:.1f}",
                f"{safe_float(s.get('vt_signed')):.6f}",
                f"{safe_float(s.get('vt_primary')):.6f}",
                f"{int(safe_float(s.get('vt_primary_axis'), 1.0))}",
                f"{safe_float(s.get('vt1')):.6f}",
                f"{safe_float(s.get('vt2')):.6f}",
                f"{safe_float(s.get('vt_norm')):.6f}",
            ]
        )

    table = ax_table.table(
        cellText=cell_text,
        colLabels=col_labels,
        bbox=[0.02, 0.66, 0.96, 0.31],
        cellLoc="center",
        colLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9.5)

    col_widths = [0.11, 0.18, 0.18, 0.08, 0.14, 0.14, 0.13]
    for (row, col), cell in table.get_celld().items():
        if col < len(col_widths):
            cell.set_width(col_widths[col])
        cell.PAD = 0.02
        if row == 0:
            cell.set_text_props(weight="bold")
            cell.set_facecolor("#f0f0f0")
        elif col != 3:
            cell.set_text_props(ha="right")

    fig.suptitle("Fig.11 Reproduction (Terminal-State Adjustment)")
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=220)
    print(f"[plot_fig11] wrote: {out_path}")


if __name__ == "__main__":
    main()
