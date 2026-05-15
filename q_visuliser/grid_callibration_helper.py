import re
import serial
import numpy as np
import csv

baudrate = 115200
port = "/dev/ttyACM0"
# find bx, by and bz
search = re.compile(r"(\w+):\s+(-?\d+\.\d+)")

# grid size from transmitter e.g. x=5 is interval in CM 5*5 25cm + and - from tx
# interval is space between points 5cm
# deadzone is a zone around the centre since cannot get accurate readings
grid_parameters = {
    "x": 3,
    "y": 3,
    "interval": 10,
    "deadzone": 10
}


def start_serial():
    return serial.Serial(port, baudrate, timeout=1)


def read_serial(serial: serial.Serial, sample_number: int):
    # read serial output and find all bx,by,bz componenets
    samples = {"bx": [], "by": [], "bz": []}
    while len(samples["bx"]) < sample_number:
        line = serial.readline().decode('utf-8').strip()
        if not line:
            continue
        matches = dict(search.findall(line))
        if "bx" in matches and "by" in matches and "bz" in matches:
            samples["bx"].append(float(matches["bx"]))
            samples["by"].append(float(matches["by"]))
            samples["bz"].append(float(matches["bz"]))
        if len(samples['bx']) > 1:
            print(f"Collecting samples: {len(
                samples['bx'])}/{sample_number} | {round((len(samples['bx']) / sample_number)*100)}%")
    return samples


def main():
    serial = start_serial()
    csv_file_path = "./logs/csv_calibration_table2.csv"

    csv_file = open(csv_file_path, "w", newline='\n')

    csv_header = ["x", "y", "bx", "by", "bz", "bx_std", "by_std", "bz_std"]
    csv_writer = csv.DictWriter(csv_file, fieldnames=csv_header)
    csv_writer.writeheader()
    # used to zigzag
    y_values = list(range(-grid_parameters["y"], grid_parameters["y"] + 1))
    x_values = list(range(-grid_parameters["x"], grid_parameters["x"] + 1))
    for i, x in enumerate(x_values):
        y_list = y_values if i % 2 == 0 else reversed(y_values)
        for y in y_list:
            inveral = grid_parameters["interval"]
            deadzone = grid_parameters["deadzone"]
            # deadzone
            if abs(x * inveral) <= deadzone and abs(y*inveral) <= deadzone:
                continue
            # deadzone
            if x == 0 or y == 0:
                # get weird signs around it
                continue
            # tell the user where to put the rx in relation to tx
            print(f"--- [NEXT CAPTURE] ---")
            print(f"Place coil at x:{x*inveral}cm | y: {y*inveral}cm")
            # user moves
            input("[capture - press return]")
            # capture samples
            samples = read_serial(serial, 500)
            # work out average and store
            row = {
                "x": x*inveral,
                "y":  y*inveral,
                "bx": np.mean(samples["bx"]),
                "by": np.mean(samples["by"]),
                "bz": np.mean(samples["bz"]),
                "bx_std": np.std(samples["bx"]),
                "by_std": np.std(samples["by"]),
                "bz_std": np.std(samples["bz"]),
            }
            # open csv and add measurement
            csv_writer.writerow(row)
            csv_file.flush()
            str_out = "".join([f"{key}: {row[key]} | " for key in row.keys()])
            print(str_out)
            # repeat
    # close file
    csv_file.close()
    serial.close()


main()
