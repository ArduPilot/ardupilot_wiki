#!/usr/bin/env python
"""****************************************************************************
* generate_motor_json.py
*   -- Yuri Rage - 2024
*
* Abbreviated implementation of AP_MotorsMatrix and AP_MotorsMatrix_test:
*   A bit of a brute force approach to parsing cpp, but it works (for now).
*   JSON output should be functionally identical to AP_MotorsMatrix_test for
*   the purposes of generating copter motor diagrams.
*
* Facilitates proper versioning of Copter motor diagrams completely
*   within ardupilot_wiki (no need for added github tasks) by fetching cpp
*   from Ardupilot/ardupilot and parsing it.
*
* May be somewhat fragile, so prefer running AP_MotorsMatrix_test
*   in an Ardupilot/ardupilot environment to update JSON data.
*
****************************************************************************"""
import argparse
import json
import math
import pathlib
import re
from sys import stderr

import requests
from packaging.version import Version

THIS_SCRIPT = pathlib.Path(__file__).resolve()
DEFAULT_OUTPUT_FILE = THIS_SCRIPT.parent / "motor_diagram_data/AP_Motors_test.json"
AP_GITHUB_OWNER = "ArduPilot"
AP_GITHUB_REPO = "ardupilot"
AP_COPTER_BRANCH = "Copter-"
AP_MOTORS_CLASS_H = "libraries/AP_Motors/AP_Motors_Class.h"
AP_MOTORS_MATRIX_CPP = "libraries/AP_Motors/AP_MotorsMatrix.cpp"
# skip frame types that are not supported by AP_MotorsMatrix_test
SKIP_FRAME_TYPES = ["MOTOR_FRAME_TYPE_Y4"]


def fetch_branches(owner, repo, per_page=100):
    """
    fetch branch data (json) using github api
    """
    branches = []
    url = f"https://api.github.com/repos/{owner}/{repo}/branches"
    params = {"per_page": per_page}
    page = 1
    while url:
        stderr.write(f"\rFetching branches for '{owner}/{repo}' [page {page: >2}] ... ")
        stderr.flush()
        try:
            response = requests.get(url, params=params)
            response.raise_for_status()
            data = response.json()
            branches.extend(
                [
                    {"name": branch["name"], "sha": branch["commit"]["sha"]}
                    for branch in data
                ]
            )
        except requests.RequestException:
            print("request error.", file=stderr)
            return branches

        if "next" in response.links:
            url = response.links["next"]["url"]
            page += 1
        else:
            break

    print("done.", file=stderr)
    return branches


def get_latest_branch(owner, repo, prefix=""):
    """
    fetch branch data and return the latest branch version matching the prefix
    returns a dict with 'name' and 'sha' keys; None if no matches found
    """
    re_prefix = f"^{prefix}"
    if prefix is None or prefix == "":
        re_prefix = ""
    version_regex = re.compile(rf"{re_prefix}(\d+\.\d+\.\d+|\d+\.\d+)")
    branches = fetch_branches(owner, repo)
    version_branches = [
        (branch, Version(match.group(1)))
        for branch in branches
        if (match := version_regex.search(branch["name"]))
    ]
    try:
        return sorted(version_branches, key=lambda bv: bv[1])[-1][0]
    except IndexError:
        print(
            f"Motor Diagrams: No branches found for 'ArduPilot/ardupilot' [{prefix}]",
            file=stderr,
        )
        return None


def fetch_github_raw_file(owner, repo, sha, path):
    """
    fetch raw file data from GitHub using commit SHA
    """
    stderr.write(f"\rFetching '{repo}/{path}' ... ")
    stderr.flush()
    url = f"https://raw.githubusercontent.com/{owner}/{repo}/{sha}/{path}"
    try:
        response = requests.get(url)
        response.raise_for_status()
        print("done.", file=stderr)
        return response.text
    except requests.RequestException:
        print("failed.", file=stderr)
        return None


def fetch_required_files():
    """
    determine stable copter branch and fetch:
        - libraries/AP_Motors/AP_Motors_Class.h
        - libraries/AP_Motors/AP_MotorsMatrix.cpp
    """
    latest_copter = get_latest_branch(
        AP_GITHUB_OWNER, AP_GITHUB_REPO, prefix=AP_COPTER_BRANCH
    )

    # fetch raw h and cpp files for parsing
    motors_class_h = None
    motors_matrix_cpp = None
    if latest_copter:
        print(f"Found '{latest_copter['name']}'", file=stderr)
        motors_class_h = fetch_github_raw_file(
            AP_GITHUB_OWNER, AP_GITHUB_REPO, latest_copter["sha"], AP_MOTORS_CLASS_H
        )
        motors_matrix_cpp = fetch_github_raw_file(
            AP_GITHUB_OWNER, AP_GITHUB_REPO, latest_copter["sha"], AP_MOTORS_MATRIX_CPP
        )
    return latest_copter["name"], motors_class_h, motors_matrix_cpp


def parse_header_matches(cpp_header, pattern, delimiter):
    """
    parse cpp header for matches of pattern
    returns a dict with 'name' and 'value' keys
    """
    matches = pattern.findall(cpp_header)
    return {
        match.split(delimiter)[0].strip(): int(match.split(delimiter)[1])
        for match in matches
    }


def parse_class_header_defs(header_content):
    """
    parse AP_Motors_Class.h file motor_frame_class and motor_frame_type enums
    """
    class_pattern = re.compile(r"MOTOR_FRAME_(?!TYPE)\w+ = \d+")
    type_pattern = re.compile(r"MOTOR_FRAME_TYPE_\w+ = \d+")
    motor_frame_class = parse_header_matches(header_content, class_pattern, "=")
    motor_frame_type = parse_header_matches(header_content, type_pattern, "=")
    # add default frame type 0
    motor_frame_type["default"] = 0
    return motor_frame_class, motor_frame_type


def truncate(number, decimal_places):
    """
    truncate a float decimal_places digits after the decimal point
    """
    str = f"{number:.{decimal_places+1}f}"
    truncated = str[: str.find(".") + decimal_places + 1]
    return float(truncated)


def normalize_yaw_factor(yaw_factor):
    """
    normalize yaw factor to CW, CCW, or ?
    """
    if yaw_factor in ["CW", "CCW"]:
        return yaw_factor
    if yaw_factor == "0":
        return "?"
    try:
        yaw_factor_value = float(yaw_factor)
        if yaw_factor_value < 0:
            return "CW"
        elif yaw_factor_value > 0:
            return "CCW"
        else:
            return "0"
    except ValueError:
        return "?"


def normalize_rpy_factors(frame_type_detail):
    """
    brief implementation of AP_MotorsMatrix::normalise_rpy_factors()
    only implemented properly for roll and pitch, as that's all we need here
    yaw is normalized for json output
    """
    roll_factor = 0
    pitch_factor = 0
    for motor in frame_type_detail["motors"]:
        roll_factor = max(roll_factor, abs(motor["Roll"]))
        pitch_factor = max(pitch_factor, abs(motor["Pitch"]))

    for motor in frame_type_detail["motors"]:
        motor["Roll"] = (
            motor["Roll"]
            if roll_factor == 0
            else truncate(0.5 * motor["Roll"] / roll_factor, 4)
        )
        motor["Pitch"] = (
            motor["Pitch"]
            if pitch_factor == 0
            else truncate(0.5 * motor["Pitch"] / pitch_factor, 4)
        )
        motor["Rotation"] = normalize_yaw_factor(motor["Rotation"])


"""
frame_class_setup_block_pattern
captures <CLASS> from _frame_class_string = "<CLASS>"
and the block of code that follows until the next 'return true'
(all motor setup calls for a given frame class)
"""
frame_class_setup_block_pattern = re.compile(
    r"\_frame_class_string\s*=\s*"  # '_frame_class_string = '
    r"\"(\w+)\""  # '<CLASS>'
    r"(.+?)"  # entire motor setup block for frame class
    r"return\s+?true",  # 'return true' ends the block
    re.DOTALL,
)

"""
frame_type_block_pattern
captures <TYPE> from _frame_type_string = "<TYPE>",
MOTOR_FRAME_TYPE_<TYPE>
and the block of code that follows until the next break statement
(all motor setup calls for a given frame type)
"""
frame_type_block_pattern = re.compile(
    # r"case\s*"  # 'case' starts the block
    r"(?:(?:case)|(?:default))\s*"  # 'case' or 'default' starts the block
    r"(MOTOR_FRAME_TYPE_\w+)?\s*:.*?"  # 'MOTOR_FRAME_TYPE_<TYPE>'
    r"_frame_type_string\s*=\s*"  # 'frame_type_string = '
    r"\"([^\"]+)\""  # '<TYPE>'
    r"(.*?)"  # entire motor setup block for frame type
    r"break",  # 'break' ends the block
    re.DOTALL,
)

"""
add_motors_pattern
captures motor details from the array passed to add_motors()
"""
add_motors_pattern = re.compile(
    r"{\s*"  # '{' starts the array
    r"(-?\d+\.?\d*)f*,\s*"  # angle
    r"(?:AP_MOTORS_MATRIX_YAW_FACTOR_)?(CCW|CW|-?\d+\.?\d*)f*,\s*"  # yaw factor
    r"(\d+)"  # test order
    r"\s*}",  # '}' ends the array
    re.DOTALL,
)

"""
add_motor_pattern
captures motor details from arguments passed to add_motor()
"""
add_single_motor_pattern = re.compile(
    r"add_motor\(AP_MOTORS_MOT_"  # 'add_motor(AP_MOTORS_MOT_'
    r"(\d+),\s*"  # motor number
    r"(-?\d+\.?\d*),\s*"  # roll factor
    r"(-?\d+\.?\d*),\s*"  # pitch factor
    r"(?:AP_MOTORS_MATRIX_YAW_FACTOR_)?(CCW|CW|-?\d+\.?\d*)f*,\s*"  # yaw factor
    r"(\d+)",  # test order
    re.DOTALL,
)

"""
add_motors_raw_pattern
captures motor details from the array passed to add_motors_raw()
"""
add_motors_raw_pattern = re.compile(
    r"{\s*"  # '{' starts the array
    r"(-?\d+\.?\d*)f*,\s*"  # roll factor
    r"(-?\d+\.?\d*)f*,\s*"  # pitch factor
    r"(?:AP_MOTORS_MATRIX_YAW_FACTOR_)?(CCW|CW|-?\d+\.?\d*)f*,\s*"  # yaw factor
    r"(\d+)"  # test order
    r"\s*}",  # '}' ends the array
    re.DOTALL,
)


def parse_motor_block(block, pattern, is_raw=False):
    """
    parse motor details from a block of code using a regex pattern
    motors are added via add_motor(), add_motors(), or add_motors_raw()
    this function differentiates based on captured groups in the pattern
    """
    motors = []
    motor_matches = pattern.findall(block)
    for motor in motor_matches:
        if is_raw:
            roll_factor, pitch_factor, yaw_factor, test_order = motor
            motors.append(
                {
                    "Number": len(motors) + 1,
                    "TestOrder": int(test_order),
                    "Rotation": yaw_factor,
                    "Roll": float(roll_factor),
                    "Pitch": float(pitch_factor),
                }
            )
        elif len(motor) == 3:
            angle, yaw_factor, test_order = motor
            motors.append(
                {
                    "Number": len(motors) + 1,
                    "TestOrder": int(test_order),
                    "Rotation": yaw_factor,
                    "Roll": math.cos(math.radians(float(angle) + 90)),
                    "Pitch": math.cos(math.radians(float(angle))),
                }
            )
        elif len(motor) == 5:
            motor_number, roll_factor, pitch_factor, yaw_factor, test_order = motor
            motors.append(
                {
                    "Number": int(motor_number),
                    "TestOrder": int(test_order),
                    "Rotation": yaw_factor,
                    "Roll": math.cos(math.radians(float(roll_factor) + 90)),
                    "Pitch": math.cos(math.radians(float(pitch_factor))),
                }
            )
    return motors


def generate_motor_matrices(cpp_content, frame_class_defs, frame_type_defs):
    """
    generate motor matrices from AP_MotorsMatrix.cpp
    """
    layouts = []

    # iterate through all frame classes
    for (
        frame_class_string,
        frame_class_setup_block,
    ) in frame_class_setup_block_pattern.findall(cpp_content):
        frame_class = f"MOTOR_FRAME_{frame_class_string}"

        # iterate through all frame types of this class
        for (
            frame_type,
            frame_type_string,
            frame_type_block,
        ) in frame_type_block_pattern.findall(frame_class_setup_block):
            if frame_type == "":
                frame_type = frame_type_string
            if frame_type in SKIP_FRAME_TYPES:
                continue

            layout = {
                "Class": frame_class_defs[frame_class],
                "ClassName": frame_class_string,
                "Type": frame_type_defs[frame_type],
                "TypeName": frame_type_string,
                "motors": [],
            }

            # add motor data to layout using one of the three regex patterns
            layout["motors"] += parse_motor_block(frame_type_block, add_motors_pattern)
            if len(layout["motors"]) == 0:
                layout["motors"] += parse_motor_block(
                    frame_type_block, add_single_motor_pattern
                )
            if len(layout["motors"]) == 0:
                layout["motors"] += parse_motor_block(
                    frame_type_block, add_motors_raw_pattern, is_raw=True
                )

            # normalize roll, pitch, yaw factors per AP_MotorsMatrix.cpp
            normalize_rpy_factors(layout)
            layouts.append(layout)

    return layouts


def generate_json():
    """
    initialize json data object(s) and generate motor matrices
    """
    version, ap_motors_class_h, ap_motors_matrix_cpp = fetch_required_files()
    frame_class_defs, frame_type_defs = parse_class_header_defs(ap_motors_class_h)
    print("Generating JSON data ... ", end="", file=stderr)
    stderr.flush()
    motor_matrices = {
        "Version": version,
        "Comments": f"Generated by {pathlib.Path(*THIS_SCRIPT.parts[-2:])}",
        "layouts": generate_motor_matrices(
            ap_motors_matrix_cpp, frame_class_defs, frame_type_defs
        ),
    }
    print(f"({len(motor_matrices['layouts'])} motor layouts) done.", file=stderr)
    return motor_matrices


def write_json(motor_matrices, output_file=None, force_overwrite=False):
    """
    write motor matrices to a JSON file
    if output_file is None, print to stdout
    if output_file is "", write to DEFAULT_OUTPUT_FILE
    """
    if output_file is None:
        print(json.dumps(motor_matrices, indent=4))
        return

    if output_file == "":
        output_file = DEFAULT_OUTPUT_FILE

    try:
        if not force_overwrite and pathlib.Path(output_file).exists():
            print(f"Error: File already exists: '{output_file}'", file=stderr)
            print("Use '-f' or '--force' to overwrite.", file=stderr)
            return
        print(f"Writing JSON data to {output_file} ... ", end="", file=stderr)
        with open(output_file, "w") as json_file:
            json.dump(motor_matrices, json_file, indent=4)
        print("done.", file=stderr)
    except FileNotFoundError as e:
        print(f"Error writing JSON file: {e}", file=stderr)


def generate_all(output_file=None, force_overwrite=False):
    """
    generate motor matrices and write to output file
    """
    motor_matrices = generate_json()
    write_json(motor_matrices, output_file, force_overwrite)


if __name__ == "__main__":
    """
    if run from command line with no arguments, generate and print to stdout
    if run with '-o' or '--output' argument, generate and write to file
    if no filename is given to '-o' or '--output', write to DEFAULT_OUTPUT_FILE
    if run with '-f' or '--force', force overwrite of existing output file
    if run with '-h' or '--help', print help
    """
    parser = argparse.ArgumentParser(
        description="Generate motor matrices (JSON) from 'AP_Motors/AP_MotorsMatrix.cpp'.")
    parser.add_argument(
        "-o",
        "--output",
        nargs="?",
        const=DEFAULT_OUTPUT_FILE,
        default=None,
        metavar="filename",
        help=(
            "Output to file (default is stdout; '-o' or '--output' with no argument writes to "
            f"'{DEFAULT_OUTPUT_FILE.relative_to(THIS_SCRIPT.parent)}')."
        ))
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        default=False,
        help="Force overwrite of existing output file.",
    )
    args = parser.parse_args()
    generate_all(args.output, args.force)
