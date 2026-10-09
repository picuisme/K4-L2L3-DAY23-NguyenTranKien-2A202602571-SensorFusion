#!/usr/bin/env python3
"""Bonus experiments for Day 23 (RUBRIC section 2): visualisation + calibration sweep.

Run from the repo root, after the graded run:

    python student/bonus/bonus_experiments.py --config student/config/paths.yaml

What it does
------------
1. Runs the provided lidar detector once over the configured frames and caches
   detections, valid GT centres and FRONT camera label centres in a JSON file
   OUTSIDE the repo (``--cache``; derived Waymo data is never committed).
2. Replays the tracker with exactly the same loop order as ``fusion-run-lab``
   (predict once -> lidar association -> camera association) using the student
   workspace modules E-H.  With no calibration error the replay must reproduce
   ``student/artifacts/metrics.json``; the script asserts this before going on.
3. Calibration sweep: the tracker is given a camera extrinsic whose yaw is off
   by a known angle, while the pixel measurements still come from the true
   camera.  For every offset it logs the camera innovation, the Mahalanobis
   distance, how many camera measurements pass the chi-square gate, and RMSE.
4. Writes figures and tables to ``student/bonus/``.  Nothing is written to
   ``student/artifacts/`` and ``platform/`` is not modified.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO = Path(__file__).resolve().parents[2]
BONUS = Path(__file__).resolve().parent
os.environ.setdefault("DAY23_STUDENT_ROOT", str(REPO / "student"))
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from scipy.stats import chi2  # noqa: E402

from fusion_lab import tracking_params as params  # noqa: E402
from fusion_lab.evaluation import aggregate_records, tracking_counts  # noqa: E402
from fusion_lab.scripts import run_lab  # noqa: E402
from fusion_lab.tracking.filter import Filter  # noqa: E402
from fusion_lab.tracking.manager import TrackManager  # noqa: E402
from fusion_lab.tracking.sensors import Sensor  # noqa: E402
from fusion_lab.workspace_loader import load_workspace  # noqa: E402

CAMERA_PIXEL_NOISE = 0.5  # same value as run_lab._front_observations
IMAGE_FRAME = 60  # frame whose FRONT image is used for the camera figure


# --------------------------------------------------------------------------
# 1. Detector pass (cached)
# --------------------------------------------------------------------------
def build_cache(config_path: Path, cache_path: Path) -> dict:
    """Run detection once and store per-frame inputs of the tracker."""
    import torch
    from simple_waymo_open_dataset_reader import WaymoDataFileReader, dataset_pb2, label_pb2
    from simple_waymo_open_dataset_reader import utils as waymo_utils

    from fusion_lab.evaluation import detection_counts, valid_ground_truth
    from fusion_lab.lidar_pcl import pcl_from_range_image

    run_lab._setup_import_paths()
    cfg = run_lab._load_paths_config(config_path)
    ws = load_workspace()
    weights = run_lab._resolve_weights(cfg)
    det_cfg = ws["detection_pipeline"].load_fpn_resnet_config(str(weights))
    model = ws["detection_pipeline"].create_fpn_model(det_cfg, str(weights))
    tfrecord = Path(cfg["waymo_dir"]) / cfg["segment"]
    start, end = int(cfg.get("frame_start", 0)), int(cfg.get("frame_end", 198))
    vehicle = label_pb2.Label.Type.TYPE_VEHICLE

    frames, calibration, image_jpeg = [], None, None
    for cnt, frame in enumerate(iter(WaymoDataFileReader(str(tfrecord)))):
        if cnt < start:
            continue
        if cnt > end:
            break
        if calibration is None:
            cal = waymo_utils.get(frame.context.camera_calibrations, dataset_pb2.CameraName.FRONT)
            calibration = {"intrinsic": list(cal.intrinsic), "width": cal.width,
                           "height": cal.height, "extrinsic": list(cal.extrinsic.transform)}
        points = pcl_from_range_image(frame, dataset_pb2.LaserName.TOP)
        tensor = torch.from_numpy(ws["bev_mapping"].bev_maps_from_pcl(points, det_cfg)).unsqueeze(0).float()
        detections = ws["detection_pipeline"].detect_objects_from_bev(tensor, model, det_cfg)
        labels = valid_ground_truth(frame.laser_labels, det_cfg, vehicle)
        group = next((g for g in frame.camera_labels if g.name == dataset_pb2.CameraName.FRONT), None)
        front = None if group is None else [
            [l.box.center_x, l.box.center_y] for l in group.labels if l.type == vehicle]
        if cnt == IMAGE_FRAME:
            image = waymo_utils.get(frame.images, dataset_pb2.CameraName.FRONT)
            image_jpeg = bytes(image.image)
        frames.append({
            "frame": cnt,
            "detections": [[float(v) for v in d] for d in detections],
            "gt": [[l.box.center_x, l.box.center_y, l.box.center_z] for l in labels],
            "front": front,
            **detection_counts(labels, detections, ws["detection_metrics"]),
        })
        print(f"\rdetector pass: frame {cnt}", end="", flush=True)
    print()
    cache = {"segment": tfrecord.name, "frames": frames, "calibration": calibration,
             "lim_x": list(det_cfg.lim_x), "lim_y": list(det_cfg.lim_y)}
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    cache_path.write_text(json.dumps(cache))
    if image_jpeg is not None:
        cache_path.with_suffix(".front.jpg").write_bytes(image_jpeg)
    return cache


# --------------------------------------------------------------------------
# 2. Tracker replay (same order as fusion-run-lab)
# --------------------------------------------------------------------------
def yaw_matrix(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    rot = np.eye(4)
    rot[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    return rot


class LoggingFilter(Filter):
    """Filter that records the camera innovation right before each camera update."""

    def __init__(self, kalman_mod, log):
        super().__init__(kalman_mod)
        self._log = log

    def update(self, track, meas):
        if meas.sensor.name == "camera":
            H = meas.sensor.get_H(track.x)
            gamma = np.asarray(self._k.innovation(track.x, meas)).reshape(-1)
            S = np.asarray(self._k.innovation_covariance(track.P, meas, H))
            before = np.asarray(track.x).reshape(-1)[:3].copy()
            super().update(track, meas)
            after = np.asarray(track.x).reshape(-1)[:3].copy()
            self._log.append({
                "frame": int(round(meas.t / params.dt)), "track": track.id,
                "gamma_u": float(gamma[0]), "gamma_v": float(gamma[1]),
                "mhd_sq": float(gamma @ np.linalg.solve(S, gamma)),
                "before": before.tolist(), "after": after.tolist(),
                "z": np.asarray(meas.z).reshape(-1).tolist(),
            })
        else:
            super().update(track, meas)


def replay(cache: dict, mode: str, yaw_offset_deg: float = 0.0, seed: int = 0) -> dict:
    """Replay tracking; ``yaw_offset_deg`` mis-calibrates the camera extrinsic."""
    ws = load_workspace()
    rng = np.random.default_rng(seed)
    cam_log: list[dict] = []
    kf = LoggingFilter(ws["kalman"], cam_log)
    manager = TrackManager(ws["track_management"])
    lidar = Sensor("lidar", None, ws["camera_fusion"])
    cal = cache["calibration"]
    true_extrinsic = np.asarray(cal["extrinsic"], dtype=float).reshape(4, 4)
    # The tracker *believes* this extrinsic; pixels still come from the true camera.
    believed = true_extrinsic @ yaw_matrix(yaw_offset_deg)
    camera = Sensor("camera", SimpleNamespace(
        intrinsic=cal["intrinsic"], width=cal["width"], height=cal["height"],
        extrinsic=SimpleNamespace(transform=believed.reshape(-1).tolist())), ws["camera_fusion"])
    det_cfg = SimpleNamespace(lim_x=cache["lim_x"], lim_y=cache["lim_y"])
    limit = chi2.ppf(params.gating_threshold, df=2)

    records, history, offered, cam_visible_tracks = [], [], 0, 0
    for item in cache["frames"]:
        cnt = item["frame"]
        labels = [SimpleNamespace(box=SimpleNamespace(center_x=c[0], center_y=c[1], center_z=c[2]))
                  for c in item["gt"]]
        observations = run_lab._lidar_observations(cnt, item["detections"], lidar, det_cfg)
        for track in manager.track_list:
            kf.predict(track)
            track.set_t(cnt * params.dt)
        ws["association"].associate_and_update(manager, observations, kf, lidar)
        after_lidar = {t.id: np.asarray(t.x).reshape(-1)[:3].copy() for t in manager.track_list}
        cam_z = []
        if mode == "fused" and item["front"] is not None:
            cam_obs = []
            for centre in item["front"]:
                camera.generate_measurement(
                    cnt, np.asarray(centre) + rng.normal(0, CAMERA_PIXEL_NOISE, 2), cam_obs)
            offered += len(cam_obs)
            cam_visible_tracks += sum(camera.in_fov(t.x) for t in manager.track_list)
            cam_z = [np.asarray(m.z).reshape(-1).tolist() for m in cam_obs]
            ws["association"].associate_and_update(manager, cam_obs, kf, camera)
        records.append({"mode": mode, "frame": cnt,
                        "det_tp": item["det_tp"], "det_fp": item["det_fp"], "det_fn": item["det_fn"],
                        "valid_gt": len(labels), **tracking_counts(manager.track_list, labels)})
        history.append({
            "frame": cnt, "gt": item["gt"],
            "meas": [np.asarray(m.z).reshape(-1).tolist() for m in observations],
            "cam_z": cam_z,
            "tracks": [{"id": t.id, "state": t.state, "score": float(t.score),
                        "pos": np.asarray(t.x).reshape(-1)[:3].tolist(),
                        "vel": np.asarray(t.x).reshape(-1)[3:].tolist(),
                        "after_lidar": after_lidar.get(t.id, np.full(3, np.nan)).tolist()}
                       for t in manager.track_list],
        })
    frames = [cache["frames"][0]["frame"], cache["frames"][-1]["frame"]]
    metrics = aggregate_records(records, mode, frames, seed, cache["segment"])
    gam = np.array([[e["gamma_u"], e["gamma_v"]] for e in cam_log]).reshape(-1, 2)
    mhd = np.array([e["mhd_sq"] for e in cam_log])
    summary = {
        "yaw_offset_deg": yaw_offset_deg,
        "rmse": metrics["tracking"][mode]["rmse"],
        "matches": metrics["tracking"][mode]["matches"],
        "ghost_track_frames": metrics["tracking"][mode]["ghost_track_frames"],
        "missed_gt_frames": metrics["tracking"][mode]["missed_gt_frames"],
        "camera_meas_offered": offered,
        "camera_tracks_in_fov": int(cam_visible_tracks),
        "camera_updates": len(cam_log),
        "mean_gamma_u_px": float(gam[:, 0].mean()) if len(gam) else None,
        "mean_abs_gamma_u_px": float(np.abs(gam[:, 0]).mean()) if len(gam) else None,
        "mean_abs_gamma_v_px": float(np.abs(gam[:, 1]).mean()) if len(gam) else None,
        "mean_mhd_sq": float(mhd.mean()) if len(mhd) else None,
        "chi2_gate_2d": float(limit),
    }
    return {"metrics": metrics, "summary": summary, "history": history, "cam_log": cam_log}


# --------------------------------------------------------------------------
# 3. Figures
# --------------------------------------------------------------------------
def nearest_gt_error(history: list[dict], track_id: int) -> tuple[list[int], list[float]]:
    """3-D error of one confirmed track against its nearest GT centre (XY gate 2 m)."""
    frames, errors = [], []
    for h in history:
        tr = next((t for t in h["tracks"] if t["id"] == track_id and t["state"] == "confirmed"), None)
        if tr is None or not h["gt"]:
            continue
        gt = np.asarray(h["gt"])
        delta = gt - np.asarray(tr["pos"])
        j = int(np.argmin(np.linalg.norm(delta[:, :2], axis=1)))
        if np.linalg.norm(delta[j, :2]) <= 2.0:
            frames.append(h["frame"])
            errors.append(float(np.linalg.norm(delta[j])))
    return frames, errors


def pick_track(lidar_run: dict, fused_run: dict) -> int:
    """Track with many camera updates and the largest fused-vs-lidar improvement."""
    counts: dict[int, int] = {}
    for e in fused_run["cam_log"]:
        counts[e["track"]] = counts.get(e["track"], 0) + 1
    best, best_gain = None, -np.inf
    for tid, n in counts.items():
        if n < 40:
            continue
        fl, el = nearest_gt_error(lidar_run["history"], tid)
        ff, ef = nearest_gt_error(fused_run["history"], tid)
        common = sorted(set(fl) & set(ff))
        if len(common) < 40:
            continue
        dl, df = dict(zip(fl, el)), dict(zip(ff, ef))
        gain = float(np.sqrt(np.mean([dl[f] ** 2 for f in common]))
                     - np.sqrt(np.mean([df[f] ** 2 for f in common])))
        if gain > best_gain:
            best, best_gain = tid, gain
    if best is None:
        best = max(counts, key=counts.get)
    return best


def fig_bev(lidar_run: dict, fused_run: dict, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(11, 6.5))
    gt = np.array([c for h in fused_run["history"] for c in h["gt"]])
    meas = np.array([m[:3] for h in fused_run["history"] for m in h["meas"]])
    ax.scatter(gt[:, 0], gt[:, 1], s=14, c="#c8c8c8", label="GT vehicle centres (all frames)")
    ax.scatter(meas[:, 0], meas[:, 1], s=5, c="#1f77b4", alpha=0.5, label="Lidar detections (measurements)")
    paths: dict[int, list] = {}
    for h in fused_run["history"]:
        for t in h["tracks"]:
            if t["state"] == "confirmed":
                paths.setdefault(t["id"], []).append(t["pos"])
    for tid, pts in sorted(paths.items()):
        pts = np.asarray(pts)
        line, = ax.plot(pts[:, 0], pts[:, 1], lw=1.8)
        ax.annotate(f"track {tid}", pts[-1, :2], textcoords="offset points", xytext=(5, 4),
                    fontsize=8, color=line.get_color())
    ax.plot([], [], c="k", lw=1.8, label="Confirmed fused tracks (one colour per ID)")
    ax.scatter([0], [0], marker="^", c="k", s=70, label="Ego vehicle")
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_title("BEV, frames 0-198: GT, lidar measurements and confirmed fused tracks\n"
                 f"{len(paths)} track IDs confirmed; ghost_track_frames = "
                 f"{fused_run['metrics']['tracking']['fused']['ghost_track_frames']}")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="center left", bbox_to_anchor=(1.01, 0.5), fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def fig_camera_effect(lidar_run: dict, fused_run: dict, tid: int, out: Path) -> dict:
    fl, el = nearest_gt_error(lidar_run["history"], tid)
    ff, ef = nearest_gt_error(fused_run["history"], tid)
    log = [e for e in fused_run["cam_log"] if e["track"] == tid]
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(fl, el, c="#1f77b4", label=f"lidar-only (RMSE {np.sqrt(np.mean(np.square(el))):.3f} m)")
    axes[0].plot(ff, ef, c="#d62728", label=f"lidar + camera (RMSE {np.sqrt(np.mean(np.square(ef))):.3f} m)")
    axes[0].set_ylabel("3-D position error (m)")
    axes[0].set_title(f"Track {tid}: effect of the camera update on one track")
    axes[0].legend(fontsize=8)
    frames = [e["frame"] for e in log]
    axes[1].plot(frames, [e["gamma_u"] for e in log], c="#2ca02c", label="innovation u (px)")
    axes[1].plot(frames, [e["gamma_v"] for e in log], c="#9467bd", label="innovation v (px)")
    axes[1].axhline(0, c="k", lw=0.6)
    axes[1].set_ylabel("camera innovation (px)")
    axes[1].legend(fontsize=8)
    shift = [float(np.linalg.norm(np.subtract(e["after"], e["before"]))) for e in log]
    axes[2].bar(frames, shift, color="#ff7f0e", width=0.9)
    axes[2].set_ylabel("|x after cam - x after lidar| (m)")
    axes[2].set_xlabel("frame")
    axes[2].set_title("Size of the state correction applied by each camera EKF update", fontsize=9)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    common = sorted(set(fl) & set(ff))
    dl, df = dict(zip(fl, el)), dict(zip(ff, ef))
    return {"track": tid, "frames_compared": len(common),
            "rmse_lidar": float(np.sqrt(np.mean([dl[f] ** 2 for f in common]))),
            "rmse_fused": float(np.sqrt(np.mean([df[f] ** 2 for f in common]))),
            "camera_updates": len(log),
            "mean_abs_gamma_u_px": float(np.mean(np.abs([e["gamma_u"] for e in log]))),
            "mean_abs_gamma_v_px": float(np.mean(np.abs([e["gamma_v"] for e in log]))),
            "mean_correction_m": float(np.mean(shift))}


def fig_image(cache: dict, fused_run: dict, jpeg_path: Path, out: Path) -> bool:
    """FRONT image of one frame with the camera measurement and h(x) before/after."""
    if not jpeg_path.is_file():
        return False
    import cv2

    image = cv2.imdecode(np.frombuffer(jpeg_path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)[:, :, ::-1]
    ws = load_workspace()
    cal = cache["calibration"]
    camera = Sensor("camera", SimpleNamespace(
        intrinsic=cal["intrinsic"], width=cal["width"], height=cal["height"],
        extrinsic=SimpleNamespace(transform=cal["extrinsic"])), ws["camera_fusion"])
    entries = [e for e in fused_run["cam_log"] if e["frame"] == IMAGE_FRAME]
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.imshow(image)
    for e in entries:
        def px(p):
            x = np.asmatrix(np.r_[p, 0, 0, 0]).T
            return np.asarray(camera.get_hx(x)).reshape(-1)
        b, a, z = px(e["before"]), px(e["after"]), e["z"]
        ax.plot(*z, "o", mfc="none", mec="yellow", ms=16, mew=2)
        ax.plot(*b, "x", c="cyan", ms=11, mew=2.5)
        ax.plot(*a, "+", c="red", ms=13, mew=2.5)
        ax.annotate(f"track {e['track']}\n|innov| {np.hypot(e['gamma_u'], e['gamma_v']):.1f} px",
                    z, textcoords="offset points", xytext=(14, -26), color="white", fontsize=9,
                    bbox=dict(fc="black", alpha=0.55, lw=0))
    ax.plot([], [], "o", mfc="none", mec="yellow", ms=10, mew=2, label="camera measurement z (noisy GT box centre)")
    ax.plot([], [], "x", c="cyan", ms=9, mew=2, label="h(x) after lidar update (before camera)")
    ax.plot([], [], "+", c="red", ms=10, mew=2, label="h(x) after camera update")
    ax.legend(loc="lower left", fontsize=9)
    ax.set_title(f"FRONT camera, frame {IMAGE_FRAME}: projected tracks vs camera measurements")
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(out, dpi=80)
    plt.close(fig)
    return True


def fig_sweep(rows: list[dict], lidar_rmse: float, out: Path) -> None:
    offs = [r["yaw_offset_deg"] for r in rows]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.4))
    axes[0].plot(offs, [r["rmse"] for r in rows], "o-", c="#d62728", label="fused RMSE")
    axes[0].axhline(lidar_rmse, c="#1f77b4", ls="--", label=f"lidar-only RMSE {lidar_rmse:.3f} m")
    axes[0].set_ylabel("RMSE (m)")
    axes[0].legend(fontsize=8)
    axes[1].plot(offs, [r["mean_gamma_u_px"] or 0 for r in rows], "o-", c="#2ca02c", label="mean innovation u (signed)")
    axes[1].plot(offs, [r["mean_abs_gamma_v_px"] or 0 for r in rows], "s-", c="#9467bd", label="mean |innovation v|")
    axes[1].set_ylabel("pixels (accepted updates only)")
    axes[1].legend(fontsize=8)
    rate = [100 * r["camera_updates"] / max(1, min(r["camera_meas_offered"], r["camera_tracks_in_fov"])) for r in rows]
    axes[2].plot(offs, rate, "o-", c="#ff7f0e")
    axes[2].set_ylabel("camera updates accepted by the gate (%)")
    for ax in axes:
        ax.set_xlabel("camera yaw mis-calibration (deg)")
        ax.grid(alpha=0.3)
    fig.suptitle("Calibration sweep: tracker believes a rotated camera extrinsic; pixels come from the true camera")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


# --------------------------------------------------------------------------
def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, default=REPO / "student/config/paths.yaml")
    parser.add_argument("--cache", type=Path,
                        default=Path.home() / ".cache/day23_bonus/detections.json")
    parser.add_argument("--offsets", type=float, nargs="+",
                        default=[0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0])
    args = parser.parse_args()

    cache = json.loads(args.cache.read_text()) if args.cache.is_file() else build_cache(
        args.config, args.cache)

    lidar_run = replay(cache, "lidar")
    fused_run = replay(cache, "fused")

    # Sanity: the replay must equal the graded run before any experiment is trusted.
    official_path = REPO / "student/artifacts/metrics.json"
    if official_path.is_file():
        official = json.loads(official_path.read_text())
        for mode, run in (("lidar", lidar_run), ("fused", fused_run)):
            mine, ref = run["metrics"]["tracking"][mode], official["tracking"][mode]
            assert mine["matches"] == ref["matches"], (mode, mine, ref)
            assert mine["ghost_track_frames"] == ref["ghost_track_frames"], (mode, mine, ref)
            assert abs(mine["rmse"] - ref["rmse"]) < 1e-9, (mode, mine, ref)
        print("replay reproduces student/artifacts/metrics.json for both modes")

    fig_bev(lidar_run, fused_run, BONUS / "fig1_bev_tracks.png")
    tid = pick_track(lidar_run, fused_run)
    effect = fig_camera_effect(lidar_run, fused_run, tid, BONUS / "fig2_camera_update_effect.png")
    has_image = fig_image(cache, fused_run, args.cache.with_suffix(".front.jpg"),
                          BONUS / "fig3_front_camera_projection.jpg")

    rows = []
    for offset in args.offsets:
        row = replay(cache, "fused", yaw_offset_deg=offset)["summary"]
        rows.append(row)
        print(json.dumps(row))
    lidar_rmse = lidar_run["metrics"]["tracking"]["lidar"]["rmse"]
    fig_sweep(rows, lidar_rmse, BONUS / "fig4_calibration_sweep.png")

    result = {
        "segment": cache["segment"], "frames": fused_run["metrics"]["frames"], "seed": 0,
        "lidar_only": lidar_run["metrics"]["tracking"]["lidar"],
        "fused_no_offset": fused_run["metrics"]["tracking"]["fused"],
        "camera_effect_on_one_track": effect,
        "front_image_figure": has_image,
        "calibration_sweep": rows,
    }
    (BONUS / "bonus_results.json").write_text(json.dumps(result, indent=2))
    lines = ["| yaw lệch (°) | RMSE fused (m) | matches | ghost | miss | đo camera qua cổng / track trong FOV | mean γ_u (px) | mean \\|γ_u\\| (px) | mean \\|γ_v\\| (px) | mean d² |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        fmt = lambda v, n=2: "–" if v is None else f"{v:.{n}f}"  # noqa: E731
        lines.append(
            f"| {r['yaw_offset_deg']:g} | {r['rmse']:.4f} | {r['matches']} | {r['ghost_track_frames']} | "
            f"{r['missed_gt_frames']} | {r['camera_updates']} / {r['camera_tracks_in_fov']} | "
            f"{fmt(r['mean_gamma_u_px'])} | {fmt(r['mean_abs_gamma_u_px'])} | "
            f"{fmt(r['mean_abs_gamma_v_px'])} | {fmt(r['mean_mhd_sq'])} |")
    (BONUS / "calibration_sweep.md").write_text(
        f"Lidar-only RMSE: {lidar_rmse:.4f} m. Cổng χ² 2D (p = {params.gating_threshold}): "
        f"{rows[0]['chi2_gate_2d']:.2f}.\n\n" + "\n".join(lines) + "\n")
    print((BONUS / "calibration_sweep.md").read_text())
    print(json.dumps(effect, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
