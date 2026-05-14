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
grid_parameters = {
    "x": 3,
    "y": 3,
    "inverval": 5
}


def start_serial():
    return serial.Serial(port, baudrate, timeout=1)


def read_serial(serial: serial.Serial, sample_number: int):
    # read serial output and find all bx,by,bz componenets
    samples = {"bx": [], "by": [], "bz": []}
    for i in range(sample_number):
        line = serial.readline().decode('utf-8').strip()
        matches = search.findall(line)
        for key, value in matches:
            if key in ["bx", "by", "bz"]:
                # print(key, value)
                samples[key].append(float(value))
    # print(samples)
    return samples


def main():
    serial = start_serial()
    csv_file_path = "./logs/csv_calibration_table.csv"

    csv_file = open(csv_file_path, "w", newline='\n')

    csv_header = ["x", "y", "bx", "by", "bz"]
    csv_writer = csv.DictWriter(csv_file, fieldnames=csv_header)
    csv_writer.writeheader()
    for x in range(-grid_parameters["x"], grid_parameters["x"]):
        for y in range(-grid_parameters["y"], grid_parameters["y"]):
            inveral = grid_parameters["inverval"]
            # tell the user where to put the rx in relation to tx
            print(f"Place coil at x:{x*inveral}cm | y: {y*inveral}cm")
            # user moves
            input("[capture - press return]")
            # capture samples
            samples = read_serial(serial, 500)
            # work out average and store
            row = {
                "x": x*inveral,
                "y": y*inveral,
                "bx": np.mean(samples["bx"]),
                "by": np.mean(samples["by"]),
                "bz": np.mean(samples["bz"]),
            }
            # open csv and add measurement
            csv_writer.writerow(row)
            str_out = "".join([f"{key}: {row[key]} | " for key in row.keys()])
            print(str_out)
            # repeat


main()
