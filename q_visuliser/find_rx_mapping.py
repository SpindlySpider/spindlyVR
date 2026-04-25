#!/usr/bin/env python3
import itertools
import math
import re
import sys
import numpy as np

# Same as firmware
POSITION_K = 88.46078431372548

# RX IMU/body origin to RX coil centre, in solver units.
# If solver output is metres, this is metres.
RX_BODY_TO_COIL = np.array([-0.045, 0.0, 0.008], dtype=float)


def quat_normalize(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b

    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def quat_rotate_vec(q, v):
    q = quat_normalize(q)
    p = np.array([0.0, v[0], v[1], v[2]], dtype=float)
    r = quat_mul(quat_mul(q, p), quat_conj(q))
    return r[1:4]


def solve_position(B):
    bx, by, bz = B
    bxy = math.sqrt(bx * bx + by * by)

    if bxy < 1e-6:
        return None

    c1 = bz / bxy
    disc = (9.0 * c1 * c1) + 8.0

    if disc < 0:
        return None

    c2 = ((3.0 * c1) + math.sqrt(disc)) / 4.0

    if not math.isfinite(c2) or c2 <= 0.0:
        return None

    denom = ((1.0 + c2 * c2) ** 2.5) * bxy

    if not math.isfinite(denom) or abs(denom) < 1e-12:
        return None

    rho_cubed = (POSITION_K * c2) / denom

    if not math.isfinite(rho_cubed) or rho_cubed <= 0.0:
        return None

    rho = rho_cubed ** (1.0 / 3.0)

    pos = np.array([
        rho * bx / bxy,
        rho * by / bxy,
        c2 * rho,
    ], dtype=float)

    if not np.all(np.isfinite(pos)):
        return None

    return pos


def load_dataset(path):
    rows = []

    data_re = re.compile(
        r"DATA,"
        r"([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+),"
        r"([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+),"
        r"([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)"
    )

    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = data_re.search(line)
            if not m:
                continue

            vals = [float(x) for x in m.groups()]

            brx = np.array(vals[0:3], dtype=float)
            qtx = quat_normalize(vals[3:7])
            qrx = quat_normalize(vals[7:11])

            rows.append((brx, qtx, qrx))

    return rows


def mapping_label(perm, signs):
    coil_names = ["coil_X", "coil_Y", "coil_Z"]
    imu_names = ["IMU_X", "IMU_Y", "IMU_Z"]

    parts = []
    for imu_i in range(3):
        sign = "+" if signs[imu_i] > 0 else "-"
        parts.append(f"{imu_names[imu_i]}={sign}{coil_names[perm[imu_i]]}")

    return ", ".join(parts)


def apply_mapping(brx, perm, signs):
    out = np.zeros(3)

    for imu_i in range(3):
        out[imu_i] = signs[imu_i] * brx[perm[imu_i]]

    return out


def score_candidate(rows, perm, signs, convention):
    btx_list = []
    pos_body_list = []
    fail_count = 0

    for brx, qtx, qrx in rows:
        b_rx_body = apply_mapping(brx, perm, signs)

        if convention == "A":
            # q_a = conj(q_tx) * q_rx
            q_rx_to_tx = quat_mul(quat_conj(qtx), qrx)
        elif convention == "B":
            # q_b = q_tx * conj(q_rx)
            q_rx_to_tx = quat_mul(qtx, quat_conj(qrx))
        else:
            raise ValueError(convention)

        b_tx = quat_rotate_vec(q_rx_to_tx, b_rx_body)
        btx_list.append(b_tx)

        pos_coil = solve_position(b_tx)
        if pos_coil is None:
            fail_count += 1
            continue

        coil_offset_tx = quat_rotate_vec(q_rx_to_tx, RX_BODY_TO_COIL)
        pos_body = pos_coil - coil_offset_tx
        pos_body_list.append(pos_body)

    btx_arr = np.array(btx_list)

    mags = np.linalg.norm(btx_arr, axis=1)
    valid = mags > 1e-9
    btx_unit = btx_arr[valid] / mags[valid, None]

    # Main score: during an in-place rotation, Btx direction should be stable.
    direction_std = np.std(btx_unit, axis=0)
    direction_score = float(np.sum(direction_std ** 2))

    # Magnitude is invariant under rotation/mapping, but useful sanity info.
    mag_mean = float(np.mean(mags))
    mag_std = float(np.std(mags))

    if len(pos_body_list) >= 3:
        pos_arr = np.array(pos_body_list)
        pos_std = np.std(pos_arr, axis=0)
        pos_score = float(np.sum(pos_std ** 2))
    else:
        pos_std = np.array([float("nan")] * 3)
        pos_score = float("inf")

    # Small position variation is useful, but do not let solver branch issues
    # dominate the coil-frame discovery too much.
    total_score = direction_score + 0.25 * pos_score + 0.01 * fail_count

    return {
        "total_score": total_score,
        "direction_score": direction_score,
        "pos_score": pos_score,
        "mag_mean": mag_mean,
        "mag_std": mag_std,
        "pos_std": pos_std,
        "fail_count": fail_count,
    }


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 find_rx_mapping.py imu_coil_dataset.txt")
        sys.exit(1)

    rows = load_dataset(sys.argv[1])

    if len(rows) < 10:
        print(f"Only found {len(rows)} DATA rows. Need more samples.")
        print("Make sure your firmware prints lines beginning with DATA,")
        sys.exit(1)

    results = []

    for convention in ["A", "B"]:
        for perm in itertools.permutations([0, 1, 2]):
            for signs in itertools.product([-1, 1], repeat=3):
                s = score_candidate(rows, perm, signs, convention)
                results.append((s["total_score"], convention, perm, signs, s))

    results.sort(key=lambda x: x[0])

    print(f"\nLoaded {len(rows)} samples\n")
    print("Top candidate mappings:\n")

    for rank, (score, convention, perm, signs, s) in enumerate(results[:12], start=1):
        print(f"#{rank}")
        print(f"  rotation convention: {convention}")
        print(f"  mapping: {mapping_label(perm, signs)}")
        print(f"  total score:      {s['total_score']:.8f}")
        print(f"  direction score:  {s['direction_score']:.8f}")
        print(f"  position score:   {s['pos_score']:.8f}")
        print(f"  |Btx| mean/std:   {s['mag_mean']:.3f} / {s['mag_std']:.3f}")
        print(f"  pos std XYZ:      {s['pos_std'][0]:.4f}, {s['pos_std'][1]:.4f}, {s['pos_std'][2]:.4f}")
        print(f"  solve fails:      {s['fail_count']}")
        print()

    best_score, best_convention, best_perm, best_signs, _ = results[0]

    print("Best firmware function:\n")
    print("static vec3_t rx_coil_to_imu_frame(vec3_t b) {")
    print("  return (vec3_t){")

    src = ["b.x", "b.y", "b.z"]

    for imu_i, field in enumerate(["x", "y", "z"]):
        sign = "-" if best_signs[imu_i] < 0 else ""
        comma = "," if imu_i < 2 else ""
        print(f"      .{field} = {sign}{src[best_perm[imu_i]]}{comma}")

    print("  };")
    print("}")

    print()
    print(f"Use quaternion rotation convention {best_convention}.")
    if best_convention == "A":
        print("A means: q_rx_to_tx = quat_mul(quat_conj(q_tx_frame), q_rx_frame);")
    else:
        print("B means: q_rx_to_tx = quat_mul(q_tx_frame, quat_conj(q_rx_frame));")


if __name__ == "__main__":
    main()
