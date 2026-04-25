import numpy as np

rows = []

with open("log.txt") as f:
    for line in f:
        if not line.startswith("CAL,"):
            continue

        p = line.strip().split(",")

        x = float(p[2])
        y = float(p[3])
        z = float(p[4])

        mag = (x*x + y*y + z*z) ** 0.5

        # reject noise / obvious weird samples
        if 20.0 < mag < 350.0:
            rows.append([x, y, z])

v = np.array(rows)

rms = np.sqrt(np.mean(v * v, axis=0))
target = np.exp(np.mean(np.log(rms)))

gain = target / rms

print("rms x y z:", rms)
print("GAIN_X =", gain[0])
print("GAIN_Y =", gain[1])
print("GAIN_Z =", gain[2])

v2 = v * gain
mag2 = np.linalg.norm(v2, axis=1)

print("corrected mag mean:", np.mean(mag2))
print("corrected mag std :", np.std(mag2))
print("corrected mag std/mean:", np.std(mag2) / np.mean(mag2))
