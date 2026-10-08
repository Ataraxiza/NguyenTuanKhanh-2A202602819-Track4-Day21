from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from starter.datasets import load_frame
from starter.projection import (
    perturb_extrinsic,
    project_velo_to_image,
    velo_to_cam,
)

CLASSES = ("Car", "Van", "Pedestrian", "Cyclist")


def distance_bin(d: float) -> str:
    if d < 15:
        return "0-15m"
    if d < 30:
        return "15-30m"
    return ">30m"


def points_in_box(points_cam: np.ndarray, obj) -> np.ndarray:
    """Return mask for points inside an object's 3D bounding box."""
    h, w, l = obj.dimensions
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    local = (points_cam - obj.location) @ R
    return (
        (np.abs(local[:, 0]) <= l / 2)
        & (local[:, 1] <= 0)
        & (local[:, 1] >= -h)
        & (np.abs(local[:, 2]) <= w / 2)
    )


def run_one(fr: dict, perturb_type: str, value: float) -> dict:
    pts = fr["points"][np.isfinite(fr["points"]).all(axis=1)]
    cam_true = velo_to_cam(pts[:, :3], fr["calib"])

    if perturb_type == "yaw":
        calib = perturb_extrinsic(fr["calib"], yaw_deg=value)
    elif perturb_type == "pitch":
        calib = perturb_extrinsic(fr["calib"], pitch_deg=value)
    elif perturb_type == "roll":
        calib = perturb_extrinsic(fr["calib"], roll_deg=value)
    else:
        raise ValueError(f"Unknown perturbation: {perturb_type}")

    uv, _, mask = project_velo_to_image(
        pts,
        calib,
        fr["image"].shape,
    )

    uv_all = np.full((len(pts), 2), np.nan)
    uv_all[mask] = uv

    rows = []

    for obj in fr["labels"]:
        if obj.type not in CLASSES:
            continue

        sel = points_in_box(cam_true, obj) & mask
        if not np.any(sel):
            continue

        u, v = uv_all[sel, 0], uv_all[sel, 1]
        x1, y1, x2, y2 = obj.bbox

        hits = int(
            ((u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)).sum()
        )
        object_points = int(sel.sum())
        distance_m = float(np.linalg.norm(obj.location))

        rows.append({
            "class": obj.type,
            "distance_m": distance_m,
            "distance_bin": distance_bin(distance_m),
            "object_points": object_points,
            "hits": hits,
            "hit_ratio": hits / object_points,
        })

    return {
        "rows": rows,
        "n_points": len(pts),
        "inside_image": int(mask.sum()),
    }


def main() -> None:
    ap = argparse.ArgumentParser(
        description="LiDAR-camera calibration QA: yaw/pitch/roll sweep"
    )
    ap.add_argument("--data-root", default="data/kitti_mini")
    ap.add_argument(
        "--frames",
        nargs="+",
        default=["000015", "000001", "000004"],
    )
    ap.add_argument(
        "--yaw-levels",
        nargs="+",
        type=float,
        default=[0, 0.5, 1, 2, 3],
    )
    ap.add_argument(
        "--pitch-levels",
        nargs="+",
        type=float,
        default=[0, 0.5, 1, 2, 3],
    )
    ap.add_argument(
        "--roll-levels",
        nargs="+",
        type=float,
        default=[0, 0.5, 1, 2, 3],
    )
    ap.add_argument(
        "--perturb-type",
        choices=["yaw", "pitch", "roll", "all"],
        default="all",
    )
    ap.add_argument(
        "--warning-threshold",
        type=float,
        default=0.80,
        help="Warn when hit_ratio is below this value.",
    )
    ap.add_argument(
        "--drop-threshold-pp",
        type=float,
        default=20.0,
        help="Detect drift when score drops by this many percentage points.",
    )
    ap.add_argument(
        "--out",
        default="results/calibration_sweep.csv",
    )
    args = ap.parse_args()

    levels = {
        "yaw": args.yaw_levels,
        "pitch": args.pitch_levels,
        "roll": args.roll_levels,
    }

    types = (
        ["yaw", "pitch", "roll"]
        if args.perturb_type == "all"
        else [args.perturb_type]
    )

    rows = []

    print()
    print(
        f"{'frame':>8} | {'type':>6} | {'value':>5} | "
        f"{'class':<11} | {'dist':>6} | {'bin':>8} | "
        f"{'FOV':>7} | {'score':>7} | {'drop':>7} | "
        f"{'warn':>4} | {'detect':>6}"
    )
    print("-" * 105)

    for frame in args.frames:
        fr = load_frame(args.data_root, frame)

        baselines = {}

        for perturb_type in types:
            # Always calculate the zero-perturbation baseline first.
            baseline_result = run_one(fr, perturb_type, 0.0)

            if baseline_result["rows"]:
                baseline_points = sum(
                    r["object_points"] for r in baseline_result["rows"]
                )
                baseline_hits = sum(
                    r["hits"] for r in baseline_result["rows"]
                )
                baselines[perturb_type] = (
                    baseline_hits / baseline_points
                    if baseline_points
                    else np.nan
                )

            for value in levels[perturb_type]:
                result = run_one(fr, perturb_type, value)
                inside_fov_ratio = (
                    result["inside_image"] / result["n_points"]
                    if result["n_points"]
                    else np.nan
                )

                baseline = baselines.get(perturb_type, np.nan)

                for r in result["rows"]:
                    score = r["hit_ratio"]
                    drop_pp = (
                        (baseline - score) * 100
                        if np.isfinite(baseline)
                        else np.nan
                    )
                    warning = int(score < args.warning_threshold)
                    detected = int(
                        np.isfinite(drop_pp)
                        and drop_pp >= args.drop_threshold_pp
                    )

                    row = {
                        "dataset": Path(args.data_root).name,
                        "frame": frame,
                        "perturb_type": perturb_type,
                        "perturb_value": value,
                        "distance_bin": r["distance_bin"],
                        "distance_m": round(r["distance_m"], 2),
                        "object_class": r["class"],
                        "n_points": result["n_points"],
                        "inside_image": result["inside_image"],
                        "inside_fov_ratio": round(inside_fov_ratio, 4),
                        "object_points": r["object_points"],
                        "hits": r["hits"],
                        "hit_ratio": round(score, 4),
                        "alignment_score": round(score, 4),
                        "drop_pp": (
                            round(drop_pp, 2)
                            if np.isfinite(drop_pp)
                            else ""
                        ),
                        "warning": warning,
                        "detected": detected,
                    }

                    rows.append(row)

                    print(
                        f"{frame:>8} | {perturb_type:>6} | "
                        f"{value:>5.1f} | {r['class']:<11} | "
                        f"{r['distance_m']:>6.1f} | "
                        f"{r['distance_bin']:>8} | "
                        f"{inside_fov_ratio * 100:>6.1f}% | "
                        f"{score * 100:>6.1f}% | "
                        f"{drop_pp:>6.1f} | "
                        f"{warning:>4} | {detected:>6}"
                    )

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    fields = [
        "dataset",
        "frame",
        "perturb_type",
        "perturb_value",
        "distance_bin",
        "distance_m",
        "object_class",
        "n_points",
        "inside_image",
        "inside_fov_ratio",
        "object_points",
        "hits",
        "hit_ratio",
        "alignment_score",
        "drop_pp",
        "warning",
        "detected",
    ]

    with open(out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    print()
    print(f"Warning threshold : {args.warning_threshold * 100:.1f}%")
    print(f"Detection threshold: {args.drop_threshold_pp:.1f} pp")
    print(f"-> {out} ({len(rows)} object rows)")


if __name__ == "__main__":
    main()