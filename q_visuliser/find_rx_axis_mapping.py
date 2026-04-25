#!/usr/bin/env python3
import argparse
import itertools
import math
import numpy as np
import pandas as pd

AXES = ["x", "y", "z"]

def normalize_quat(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n[n < 1e-12] = 1.0
    return q / n

def quat_conj(q):
    out = q.copy()
    out[..., 1:] *= -1.0
    return out

def quat_mul(a, b):
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ], axis=-1)

def quat_rotate(q, v):
    q = normalize_quat(q)
    p = np.zeros((len(v), 4))
    p[:, 1:] = v
    r = quat_mul(quat_mul(q, p), quat_conj(q))
    return r[:, 1:]

def unit(v):
    n = np.linalg.norm(v, axis=1, keepdims=True)
    n[n < 1e-12] = 1.0
    return v / n

def angular_score(v):
    """
    Lower is better.
    Measures how much the rotated magnetic vector direction moves.
    """
    u = unit(v)
    mean = np.mean(u, axis=0)
    mean_n = np.linalg.norm(mean)
    if mean_n < 1e-12:
        return 999.0
    mean /= mean_n

    dots = np.clip(u @ mean, -1.0, 1.0)
    angles = np.arccos(dots)
    return float(np.std(angles))

def vector_std_score(v):
    """
    Lower is better.
    Measures raw vector stability after rotation.
    """
    scale = np.mean(np.linalg.norm(v, axis=1))
    if scale < 1e-12:
        scale = 1.0
    return float(np.linalg.norm(np.std(v, axis=0)) / scale)

def mapping_string(perm, signs):
    parts = []
    for imu_axis, coil_index, sign in zip(AXES, perm, signs):
        s = "+" if sign > 0 else "-"
        parts.append(f"IMU_{imu_axis.upper()}={s}coil_{AXES[coil_index].upper()}")
    return ", ".join(parts)

def firmware_function(perm, signs):
    lines = []
    for imu_axis, coil_index, sign in zip(AXES, perm, signs):
        s = "" if sign > 0 else "-"
        lines.append(f"      .{imu_axis} = {s}b.{AXES[coil_index]},")
    return (
        "static vec3_t rx_coil_to_imu_frame(vec3_t b) {\n"
        "  return (vec3_t){\n"
        + "\n".join(lines) +
        "\n  };\n"
        "}"
    )

def find_col(df, names):
    lower = {c.lower(): c for c in df.columns}
    for name in names:
        if name.lower() in lower:
            return lower[name.lower()]
    raise KeyError(f"Could not find any of columns: {names}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("--top", type=int, default=12)
    parser.add_argument("--skip", type=int, default=0, help="Skip first N samples")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    bx_col = find_col(df, ["bx", "b_x", "Brx_x", "coil_x"])
    by_col = find_col(df, ["by", "b_y", "Brx_y", "coil_y"])
    bz_col = find_col(df, ["bz", "b_z", "Brx_z", "coil_z"])

    qw_col = find_col(df, ["qw", "q0", "qrx_w", "rx_qw", "rx_q0"])
    qx_col = find_col(df, ["qx", "q1", "qrx_x", "rx_qx", "rx_q1"])
    qy_col = find_col(df, ["qy", "q2", "qrx_y", "rx_qy", "rx_q2"])
    qz_col = find_col(df, ["qz", "q3", "qrx_z", "rx_qz", "rx_q3"])

    if args.skip > 0:
        df = df.iloc[args.skip:].reset_index(drop=True)

    b_coil = df[[bx_col, by_col, bz_col]].to_numpy(dtype=float)
    q_rx = df[[qw_col, qx_col, qy_col, qz_col]].to_numpy(dtype=float)
    q_rx = normalize_quat(q_rx)

    results = []

    # 6 axis permutations × 8 sign choices = 48 mappings
    for perm in itertools.permutations([0, 1, 2]):
        for signs in itertools.product([1, -1], repeat=3):
            b_body = np.zeros_like(b_coil)
            for out_axis in range(3):
                b_body[:, out_axis] = signs[out_axis] * b_coil[:, perm[out_axis]]

            # Test both common quaternion conventions:
            # 1. q rotates RX/body vector into world/TX frame
            # 2. conj(q) rotates RX/body vector into world/TX frame
            for convention, q_used in [
                ("q_rx", q_rx),
                ("conj_q_rx", quat_conj(q_rx)),
            ]:
                b_world = quat_rotate(q_used, b_body)

                direction = angular_score(b_world)
                vector_std = vector_std_score(b_world)

                # Weighted score. Direction stability matters most.
                total = direction + 0.25 * vector_std

                results.append({
                    "total": total,
                    "direction": direction,
                    "vector_std": vector_std,
                    "convention": convention,
                    "perm": perm,
                    "signs": signs,
                    "mapping": mapping_string(perm, signs),
                    "bmag_mean": float(np.mean(np.linalg.norm(b_world, axis=1))),
                    "bmag_std": float(np.std(np.linalg.norm(b_world, axis=1))),
                    "std_x": float(np.std(b_world[:, 0])),
                    "std_y": float(np.std(b_world[:, 1])),
                    "std_z": float(np.std(b_world[:, 2])),
                })

    results.sort(key=lambda r: r["total"])

    print(f"Loaded {len(df)} samples")
    print("\nTop candidate mappings:\n")

    for i, r in enumerate(results[:args.top], 1):
        print(f"#{i}")
        print(f"  quaternion convention: {r['convention']}")
        print(f"  mapping:              {r['mapping']}")
        print(f"  total score:          {r['total']:.8f}")
        print(f"  direction score:      {r['direction']:.8f}")
        print(f"  vector std score:     {r['vector_std']:.8f}")
        print(f"  |B| mean/std:         {r['bmag_mean']:.3f} / {r['bmag_std']:.3f}")
        print(f"  rotated B std XYZ:    {r['std_x']:.3f}, {r['std_y']:.3f}, {r['std_z']:.3f}")
        print()

    best = results[0]
    print("Best firmware function:\n")
    print(firmware_function(best["perm"], best["signs"]))

    print("\nUse quaternion convention:")
    if best["convention"] == "q_rx":
        print("  vec3_t b_tx = quat_rotate_vec(q_rx_frame, b_rx_body);")
    else:
        print("  vec3_t b_tx = quat_rotate_vec(quat_conj(q_rx_frame), b_rx_body);")

if __name__ == "__main__":
    main()
