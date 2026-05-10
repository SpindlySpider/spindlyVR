import numpy as np
import re

regex_search = re.compile(r"(\w) \w+: (\d+\.\d+)")
# config the file which contains x y z logs you want to average
noise_floor_file = "noise_floor.txt"
x_algined = "x_aligned.txt"
y_algined = "y_aligned.txt"
z_algined = "z_aligned.txt"


def compute_average(log_location, noise_floor={"x": 0, "y": 0, "z": 0}):
    log_path = open(f"./logs/{log_location}")
    input_file = log_path.read()
    regex_out = regex_search.findall(input_file)
    noise = {"x": [0, 0], "y": [0, 0], "z": [0, 0]}
    for match in regex_out:
        # print(match)
        key = match[0]
        # store value and count
        noise[key][0] += float(match[1])
        noise[key][1] += 1
    # workout average
    # print(f"Total number of x: {noise['x'][1]}")
    for key in noise.keys():
        # x y z, average them and replace the value with the averaged value
        # noise[key] = (noise[key][0] / noise[key][1]) - noise_floor[key]
        noise[key] = (noise[key][0] / noise[key][1]) - noise_floor[key]
    # print(f"Averaged X: {noise['x']} | Y: {noise['y']} | Z: {noise['z']}")
    return noise


def calibrate(x, y, z):
    print(x, y, z)
    array = np.array([x, y, z], dtype=float)
    normalised = array / array.max()
    normalised = normalised.tolist()

    print(f"calibration results: X: {normalised[0]} | Y:{
          normalised[1]} | Z:{normalised[2]}")


noise_floor = compute_average(noise_floor_file)
print(f"Noise floor X: {noise_floor['x']} | Y: {
      noise_floor['y']} | Z: {noise_floor['z']}")
x = compute_average(x_algined, noise_floor)["x"]
y = compute_average(y_algined, noise_floor)["y"]
z = compute_average(z_algined, noise_floor)["z"]

calibrate(x, y, z)
