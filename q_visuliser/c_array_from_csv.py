# generate a C array for fingerprinting using generated calibration csv
import csv

csv_file_path = "./logs/csv_calibration_table.csv"

c_type_translator = {
    "bx": "bx",
    "by": "bz",
    "bz": "bz",
    "x": "x",
    "y": "y",
}


def read_csv():
    # reads and generates array entries for C array
    csv_file = open(csv_file_path, "r")
    reader = csv.DictReader(csv_file)
    # print(reader.fieldnames)
    str_out = []
    for row in reader:
        # they will have bx, etc
        string = [row["bx"], row["by"], row["bz"], row["x"], row["y"]]
        string = [f"{float(x)}f" for x in string]
        string = "{ "+", ".join(string) + " },\n"
        str_out.append(string)
    return str_out


def main():
    items = read_csv()
    final_str = f"static const fingerprint_t fingerprints[{len(items)}] = {{\n"
    for item in items:
        final_str += f"     {item}"
    final_str += "};\n"
    print(final_str)


main()
