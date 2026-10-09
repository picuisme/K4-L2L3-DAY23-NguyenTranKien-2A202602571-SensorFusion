"""Track initialization, scoring, and deletion helpers.

Part H supplies lidar-driven existence decisions (docs/HUONG_DAN_KY_THUAT.md §2).
Use tracking parameters for the score window, thresholds, and covariance limit.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from fusion_lab.workspace_support import get_tracking_params


def init_track_state_from_meas(meas: Any) -> dict[str, Any]:
    """Initialize track state, covariance, lifecycle state, and score from a measurement.

    Args:
        meas: Lidar measurement with ``z``, ``R``, ``sensor``.

    Returns:
        Dict with keys ``x``, ``P``, ``state``, ``score`` (matrices as ``np.matrix``).
    """
    # vi: TODO Part H — đổi meas.z sang vehicle frame; x = [pos; 0 velocity];
    # vi: P block pos từ R xoay, vel từ sigma_p44/55/66; score = 1/window; state initialized.
    params = get_tracking_params()
    sens_to_veh = np.asarray(meas.sensor.sens_to_veh, dtype=float)
    rotation = sens_to_veh[:3, :3]
    # Measured position in sensor axes -> vehicle axes (homogeneous transform).
    z_sens = np.asarray(meas.z, dtype=float).reshape(-1)[:3]
    position = rotation @ z_sens + sens_to_veh[:3, 3]

    x = np.asmatrix(np.zeros((6, 1)))
    x[0:3, 0] = position.reshape(3, 1)  # velocity unknown -> start at zero

    P = np.asmatrix(np.zeros((6, 6)))
    # Position uncertainty: measurement noise rotated into the vehicle frame.
    P[0:3, 0:3] = rotation @ np.asarray(meas.R, dtype=float) @ rotation.T
    # Velocity is unobserved by one detection -> broad prior.
    P[3:6, 3:6] = np.diag(
        [params.sigma_p44**2, params.sigma_p55**2, params.sigma_p66**2]
    )
    return {"x": x, "P": P, "state": "initialized", "score": 1.0 / params.window}


def update_track_score(track: dict[str, Any], associated: bool) -> dict[str, Any]:
    """Update existence once per lidar frame; camera passes never call this helper.

    A hit adds 1/window, capped at one; an in-FOV miss subtracts 1/window.
    Confirm above confirmed_threshold, and preserve confirmed state after misses.

    Args:
        track: Dict-like track with ``score``, ``state``.
        associated: True for a lidar hit; False for a lidar miss within the lidar FOV.

    Returns:
        Updated track dict.
    """
    # vi: TODO Part H — chỉ lidar: hit +1/window (tối đa 1), miss trong FOV -1/window.
    # vi: score > confirmed_threshold → confirmed; đã confirmed không hạ trạng thái.
    # vi: Camera không gọi hàm này; track chưa confirmed với hit → tentative.
    params = get_tracking_params()
    step = 1.0 / params.window
    if associated:
        track["score"] = min(1.0, track["score"] + step)
    else:
        track["score"] = track["score"] - step
    if track["state"] != "confirmed":
        # Confirmation is sticky: once confirmed, only deletion ends the track.
        if track["score"] > params.confirmed_threshold:
            track["state"] = "confirmed"
        elif associated:
            track["state"] = "tentative"
    return track


def should_delete_track(track: dict[str, Any]) -> bool:
    """Return whether a lidar lifecycle pass should remove this track.

    Delete if either horizontal variance exceeds max_P, or if a confirmed
    track has score < delete_threshold, or an unconfirmed track has score <= 0.
    Camera passes never trigger deletion.

    Args:
        track: Dict with ``score``, ``state``, ``P``.

    Returns:
        True if track should be removed.
    """
    # vi: TODO Part H — Pxx hoặc Pyy > max_P: xóa bất kể score.
    # vi: confirmed: xóa khi score < delete_threshold; chưa confirmed: score <= 0.
    # vi: Các điều kiện là OR; camera không đánh giá/xóa track.
    params = get_tracking_params()
    P = np.asarray(track["P"], dtype=float)
    # Position has drifted too far without correction: the estimate is useless.
    if P[0, 0] > params.max_P or P[1, 1] > params.max_P:
        return True
    if track["state"] == "confirmed":
        return bool(track["score"] < params.delete_threshold)
    return bool(track["score"] <= 0)
