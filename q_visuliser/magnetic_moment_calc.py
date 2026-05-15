import math
import re

regex_search = re.compile(r"(\w): (\d+\.\d+)")
# config the file which contains x y z logs you want to average
raw_magnetic_moment_file = "raw_magnetic_logs_50cm.txt"


def compute_average(log_location,real_distance):
    # assumes that log has equal number of XYZ samples will break if not
    log_path = open(f"./logs/{log_location}")
    input_file = log_path.read()
    regex_out = regex_search.findall(input_file)
    raw_dict = {"x": [], "y": [], "z": []}
    # extact raw values from log
    for match in regex_out:
        # print(match)
        key = str(match[0]).lower()
        # store value
        raw_dict[key].append(float(match[1]))

    entries = len(raw_dict["x"])
    magnitude = []
    for i in range(entries):
        # calculate the magnitude
        magnitude.append(
            math.sqrt(raw_dict["x"][i]**2 + raw_dict["y"][i]**2 + raw_dict["z"][i]**2)
        )
    moment = 0
    # calculate mean avg
    for mag in magnitude:
        moment += mag
    moment = moment/ len(magnitude)

    return 1*(real_distance/moment)


# real distance is where it was measured from in meters e.g. 50cm -> 0.5m
# also this script assumes that the calibrated moment value is 1
moment = compute_average(raw_magnetic_moment_file,0.5)
print(moment)
