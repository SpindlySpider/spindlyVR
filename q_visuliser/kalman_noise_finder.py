# script to find the noise of position estimation algorthim and linear acceleration on each axis
import math
import re
import copy

regex_search = re.compile(r"((?:\w+\s\w)|\w): (\-?\d+\.\d+)")
raw_position_estimation_accel = "raw_pos_est_lin_accel.txt"


def compute_average(log_location):
    # assumes that log has equal number of XYZ samples will break if not
    log_path = open(f"./logs/{log_location}")
    input_file = log_path.read()
    regex_out = regex_search.findall(input_file)
    values_dict = {"values": [], "mean": 0}

    raw_dict = {
        "x": copy.deepcopy(values_dict),
        "y": copy.deepcopy(values_dict),
        "z": copy.deepcopy(values_dict),
        "accel x": copy.deepcopy(values_dict),
        "accel y": copy.deepcopy(values_dict),
        "accel z": copy.deepcopy(values_dict),
    }  # extract raw values from log and calculate mean
    for match in regex_out:
        # print(match)
        key = str(match[0]).lower()
        # store value
        value = float(match[1])
        raw_dict[key]["values"].append(value)
        raw_dict[key]["mean"] += value

    # calculate mean avg for each axis
    for key in raw_dict.keys():
        num_entries = len(raw_dict[key]["values"])
        raw_dict[key]["count"] = num_entries
        raw_dict[key]["mean"] = raw_dict[key]["mean"] / num_entries

    for key in raw_dict.keys():
        num_entries = raw_dict[key]["count"]
        # workout deviations
        raw_dict[key]["sd"] = sum([(x - raw_dict[key]["mean"])**2
                                   for x in raw_dict[key]["values"]])
        # workout standard diviation
        raw_dict[key]["sd"] = math.sqrt(raw_dict[key]["sd"] /
                                        (raw_dict[key]["count"]-1))

    return raw_dict


# real distance is where it was measured from in meters e.g. 50cm -> 0.5m
# also this script assumes that the calibrated moment value is 1
standard_dict = compute_average(raw_position_estimation_accel)
values = [[key, standard_dict[key]["sd"]] for key in standard_dict.keys()]

for entry in values:
    print(f"{entry[0]}:", entry[1])
