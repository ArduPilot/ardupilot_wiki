#!/usr/bin/env python
"""****************************************************************************
* build_motor_diagrams.py
*   -- Yuri Rage - 2024
*
*  Generates motor diagrams for the Copter wiki and outputs them as svg files
*    in the ardupilot/copter/build directory.
*
*  Requires the following files in 'scripts/motor_diagram_data':
*    motor_diagram_template.svg:
*      svg template for all diagrams
*      contains a <defs> section with all the svg elements along with
*      a stubbed out <g> (group) for each svg layer
*
*    AP_Motors_test.json is a generated file:
*      Run ./build/linux/examples/AP_Motors_test p > AP_Motors_test.json
*      in an ardupilot dev environment.
*      See: libraries/AP_Motors/examples/AP_Motors_test/AP_Motors_test.cpp
*
*    AP_Motors_display.json overrides select values in AP_Motors_test.json:
*      In some cases, AP_Motors_test.json has either terse class/type names,
*      or the motor layout is not conducive to an intuitive diagram.
*      The QUAD Y4A and BICOPTER layouts are also added, since they are
*      not included in AP_Motors_test.json.
*
*  Usage:
*    build_motor_diagrams.py --help
*
****************************************************************************"""

import argparse
import copy
import json
import math
import os
import pathlib
import time
from dataclasses import dataclass
from sys import argv, stderr
from xml.etree.ElementTree import SubElement, parse, tostring


@dataclass
class Extents:
    """holds the min/max extents of a set of svg elements"""

    minX: float
    minY: float
    maxX: float
    maxY: float

    def update(self, minX=None, minY=None, maxX=None, maxY=None):
        if minX is not None:
            self.minX = minX
        if minY is not None:
            self.minY = minY
        if maxX is not None:
            self.maxX = maxX
        if maxY is not None:
            self.maxY = maxY


""" CONSTANTS """
THIS_SCRIPT = pathlib.Path(__file__).resolve()

DEFAULT_OUTPUT_DIR = THIS_SCRIPT.parent / "../copter/source/images"
AP_MOTORS_TEST_JSON_FILE = THIS_SCRIPT.parent / "motor_diagram_data/AP_Motors_test.json"
AP_MOTORS_DISPLAY_JSON_FILE = (
    THIS_SCRIPT.parent / "motor_diagram_data/AP_Motors_display.json"
)
SVG_TEMPLATE_FILE = THIS_SCRIPT.parent / "motor_diagram_data/motor_diagram_template.svg"
WIKI_OUTPUT_FILE = (
    THIS_SCRIPT.parent / "../copter/source/docs/connect-escs-and-motors.rst"
)
WIKI_BEGIN_COMMENT = "BEGIN MOTOR DIAGRAMS"
WIKI_END_COMMENT = ".. END MOTOR DIAGRAMS"

XLINK_NAMESPACE = "http://www.w3.org/1999/xlink"
BASE_IMAGE_SIZE = 330  # nominal wiki page display size
FRAME_BASE_RADIUS = 385  # base diameter of the frame display
SINGLE_MOTOR_RADIUS = 120  # display radius of a single motor's rotor arc
DIAGRAM_PADDING = 20  # minimum padding around the motor diagram
DODECAHEXA_MULTIPLIER = 1.2  # multiplier to give dodecahexa a little more room
COAXIAL_Y_SCALE = 0.66  # scale factor for coaxial frame 3d skew
CHAR_CODE_BASE = 64  # base character code for lettering (ord('A') - 1)

DIAGRAM_SCALING = {
    # number of motor positions vs display radius
    2: FRAME_BASE_RADIUS * 1.1,
    3: FRAME_BASE_RADIUS * 1.25,
    4: FRAME_BASE_RADIUS * 1,
    5: FRAME_BASE_RADIUS * 1.1,
    6: FRAME_BASE_RADIUS * 1.25,
    7: FRAME_BASE_RADIUS * 1.6,
    8: FRAME_BASE_RADIUS * 1.6,
    9: FRAME_BASE_RADIUS * 1.66,
    10: FRAME_BASE_RADIUS * 1.8,
}
""" END CONSTANTS """


def load_json(file_path):
    """
    load a json file and return the contents as a dict
    """
    try:
        with open(file_path, "r") as file:
            return json.load(file)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Motor Diagrams: load_json() error\n{e}", file=stderr)
        return {}


def load_svg_template(file_path):
    """
    load an svg file and return the contents as an xml.etree.ElementTree
    """
    try:
        return parse(file_path)
    except FileNotFoundError as e:
        print(
            f"Motor Diagrams: Error loading SVG template from {file_path}: {e}",
            file=stderr,
        )
        return None


def get_motors_json():
    """
    load the AP_Motors_test.json and AP_Motors_display.json files
    condense their contents into a single object for processing
    """
    motors_json = load_json(AP_MOTORS_TEST_JSON_FILE)
    display_json = load_json(AP_MOTORS_DISPLAY_JSON_FILE)

    if not motors_json or not display_json:
        return {}

    existing_layouts = {(layout["Class"], layout["Type"]) for layout in motors_json["layouts"]}

    # append additional layouts from display json
    for layout in display_json["layouts"]:
        layout_identifier = (layout["Class"], layout["Type"])
        if layout_identifier not in existing_layouts:
            motors_json["layouts"].append(layout)
            existing_layouts.add(layout_identifier)

    # aggregate overrides from display json
    for layout in motors_json["layouts"]:
        frame_class = str(layout["Class"])
        frame_type = str(layout["Type"])

        class_overrides = display_json.get(frame_class, {})
        type_overrides = class_overrides.get(frame_type, {})

        layout["ClassName"] = class_overrides.get("ClassName", layout.get("ClassName"))
        layout["TypeName"] = type_overrides.get("TypeName", layout.get("TypeName"))
        layout["motors"] = type_overrides.get("motors", layout.get("motors"))
        layout["Skip"] = type_overrides.get("Skip", False)
        if "Notes" in type_overrides:
            layout.setdefault("Notes", []).extend(type_overrides["Notes"])
        if "FrameLines" in type_overrides:
            layout.setdefault("FrameLines", []).extend(type_overrides["FrameLines"])
        if "WikiNotes" in type_overrides:
            layout["WikiNotes"] = type_overrides["WikiNotes"]

    return motors_json


def replace_namespace_prefixes(xml_str):
    """
    python's pedantic xml module outputs incompatible namespace prefixes
    so, replace them with the correct prefixes for proper svg output
    """
    replacements = [
        ("xmlns:ns0", "xmlns"),
        ("xmlns:ns1", "xmlns:xlink"),
        ("ns0:", ""),
        ("ns1:", "xlink:"),
    ]
    for old, new in replacements:
        xml_str = xml_str.replace(old, new)
    return xml_str


def get_filename(frame_class, frame_type, frame_name):
    """
    create consistent filenames for motor diagrams
    abbreviate slightly for long frame names
    """

    replacements = [
        ("betaflight", "bf"),
        ("reversed", "rev"),
        ("clockwise", "cw"),
        ("counterclockwise", "ccw"),
        ("no yaw torque", "nyt"),
        (" ", "_"),
        ("+", "plus"),
        ("(", ""),
        (")", ""),
        ("/", ""),
    ]
    filename = frame_name.lower()
    for old, new in replacements:
        filename = filename.replace(old, new)

    filename = filename.strip("_")  # remove leading and trailing underscores
    return f"m_{frame_class:02d}_{frame_type:02d}_{filename}.svg"


def get_translated_coordinates(x, y, radius):
    """
    scale motor vector (Roll/Pitch) output to svg coordinates
    """
    θ = math.atan2(-y, -x)
    r = math.sqrt(y ** 2 + x ** 2) * radius
    x_translated = r * math.cos(θ)
    y_translated = r * math.sin(θ)
    return x_translated, y_translated, r, θ


def append_svg_element(parent, element_type, x=0, y=0, arg=""):
    """
    append an svg element to the parent
    arg is an optional id to use as a reference in an xlink:href
    """
    elem = SubElement(parent, element_type, x=str(x), y=str(y))
    if element_type == "use":
        elem.set(f"{{{XLINK_NAMESPACE}}}href", f"#{arg}")
    if element_type == "text":
        elem.text = arg
    return elem


def append_svg_line(parent, x1=0, y1=0, x2=0, y2=0):
    """
    draw an svg line element
    """
    elem = SubElement(parent, "line", x1=str(x1), y1=str(y1), x2=str(x2), y2=str(y2))
    return elem


def append_svg_rect(parent, x=0, y=0, width=0, height=0):
    """
    draw an svg rect element
    """
    elem = SubElement(
        parent,
        "rect",
        x=str(x),
        y=str(y),
        width=str(width),
        height=str(height),
        fill="white",
    )
    return elem


def locate_svg_elements(svg_root):
    """
    locate and return the svg elements for the motor diagram
    """
    motor_diagram_layers = svg_root.find('.//*[@id="motor-diagram-layers"]')
    layer_background = svg_root.find('.//*[@id="layer-background"]')
    layer_frame = svg_root.find('.//*[@id="layer-frame"]')
    layer_motors = svg_root.find('.//*[@id="layer-motors"]')
    layer_motor_numbers = svg_root.find('.//*[@id="layer-motor-numbers"]')
    layer_motor_letters = svg_root.find('.//*[@id="layer-motor-letters"]')
    layer_frame_name = svg_root.find('.//*[@id="layer-frame-name"]')
    layer_frame_notes = svg_root.find('.//*[@id="layer-frame-notes"]')
    return (
        motor_diagram_layers,
        layer_background,
        layer_frame,
        layer_motors,
        layer_motor_numbers,
        layer_motor_letters,
        layer_frame_name,
        layer_frame_notes,
    )


def calculate_unique_motor_positions(layout):
    """
    unique_motor_positions determines diagram scaling
    is_coaxial determines whether to apply a 3d skew to the diagram
    """
    unique_motor_position_defs = {
        (float(motor["Roll"]), float(motor["Pitch"])) for motor in layout["motors"]
    }
    unique_motor_positions = len(unique_motor_position_defs)
    is_coaxial = len(layout["motors"]) > unique_motor_positions

    for motor in layout["motors"]:
        if motor["Rotation"] == "tail-servo":
            unique_motor_positions -= 1

    return unique_motor_positions, is_coaxial


def append_motor_letter(
    is_coaxial,
    motor,
    layer_motor_letters,
    base_font_size,
    extents,
    x,
    y,
    r,
    θ,
    unique_motor_positions,
    test_order,
):
    """
    append the test order letter (red font)
    return letter_element for use in handle_av_tails()
    """
    # calculate motor letter position
    if is_coaxial:
        x_offset = 130
        if abs(x) < 1 and y > 0:
            x = -1  # bottom-center motors have left offset lettering
        x = x + x_offset if x > 0 else x - x_offset
    elif motor["Rotation"] == "tail-servo":
        x += 60
    else:
        x = (r + SINGLE_MOTOR_RADIUS + DIAGRAM_PADDING) * math.cos(θ)
        y = (r + SINGLE_MOTOR_RADIUS + DIAGRAM_PADDING) * math.sin(θ)

    # do not add lettering for bi-copters
    letter_element = None
    if unique_motor_positions > 2:
        letter_element = append_svg_element(
            layer_motor_letters, "text", x, y, chr(test_order + CHAR_CODE_BASE)
        )
        # if letter is added, it's potentially the most distant point from the origin
        extents.update(
            min(extents.minX, x - base_font_size / 2),
            min(extents.minY, y - base_font_size / 2),
            max(extents.maxX, x + base_font_size / 2),
            max(extents.maxY, y + base_font_size / 2),
        )
    return letter_element


def handle_av_tails(layout, motor_element, number_element, letter_element, x, y):
    """
    apply unique 3d transforms to the tail rotors of a-tail and v-tail frames
    """
    if y < 0:
        # only add transforms for tail rotors (above the diagram origin)
        return
    x_scale = -1 if x < 0 and "v" not in layout["TypeName"].lower() else 1
    y_scale = -1
    if x < 0 and "v" in layout["TypeName"].lower():
        y_scale = 1
    elif "v" not in layout["TypeName"].lower():
        y_scale = 1
    motor_element.set("style", f"transform: rotate3d({x_scale}, {y_scale}, 0, 45deg)")
    number_element.set("style", f"transform: rotate3d({x_scale}, {y_scale}, 0, 45deg)")
    if letter_element is not None:
        letter_element.set(
            "style", f"transform: rotate3d({x_scale}, {y_scale}, 0, 45deg)"
        )


def append_frame(
    svg_root, layout, layer_frame, is_coaxial, motor_center_points, frame_display_radius
):
    """
    append frame depiction, including lines to represent frame arms
    """
    if is_coaxial:
        # place the frame in the 3d layer
        layer_frame = svg_root.find('.//*[@id="layer-frame-3d"]')
    # add frame lines connecting motors to the frame
    if "FrameLines" in layout:
        # FrameLines from AP_Motors_display.json overrides
        for line in layout["FrameLines"]:
            x1, y1, r1, θ1 = get_translated_coordinates(
                line[0], line[1], frame_display_radius
            )
            x2, y2, r2, θ2 = get_translated_coordinates(
                line[2], line[3], frame_display_radius
            )
            append_svg_line(layer_frame, x1, y1, x2, y2)
    else:
        # simply connect all motors to the frame's center (at the diagram origin)
        for motor in layout["motors"]:
            x2, y2 = motor_center_points[motor["Number"]]
            append_svg_line(layer_frame, 0, 0, x2, y2)

    append_svg_element(layer_frame, "use", 0, -5, "frame-2d")


def append_footer_text(
    layout,
    layer_frame_name,
    layer_frame_notes,
    extents,
    base_font_size,
    frame_notes_font_size,
):
    """
    append frame class/type name and notes at the bottom of the diagram
    """
    frame_name = f'{layout["ClassName"]} {layout["TypeName"]}'
    textElem = append_svg_element(
        layer_frame_name, "text", 0, extents.maxY + base_font_size, frame_name
    )
    extents.maxY += base_font_size * 1.5

    # add frame notes
    if "Notes" in layout:
        for note in layout["Notes"]:
            textElem = append_svg_element(layer_frame_notes, "text", 0, 0, f"{note}")
            textElem.set("y", str(extents.maxY + frame_notes_font_size))
            extents.maxY += frame_notes_font_size * 1.5
    return frame_name


def finalize_diagram(svg_root, layer_background, extents):
    """
    set final svg size, view box, and background
    """
    # set svg size and view box
    extents.update(
        minX=extents.minX - DIAGRAM_PADDING,
        minY=extents.minY - DIAGRAM_PADDING,
        maxX=extents.maxX + DIAGRAM_PADDING,
        maxY=extents.maxY + DIAGRAM_PADDING,
    )

    svg_width = abs(extents.minX) + abs(extents.maxX)
    svg_height = abs(extents.minY) + abs(extents.maxY)

    # TODO: transparent background? (comment out next line)
    append_svg_rect(layer_background, extents.minX, extents.minY, svg_width, svg_height)

    svg_root.set("width", f"{max(svg_width, svg_height)}")
    svg_root.set("height", f"{max(svg_width, svg_height)}")
    svg_root.set("viewBox", f"{extents.minX} {extents.minY} {svg_width} {svg_height}")
    return svg_width, svg_height


def write_svg_file(
    element, frame_class, frame_type, display_name, output_dir=DEFAULT_OUTPUT_DIR
):
    """
    write the svg file to disk
    """
    filename = get_filename(frame_class, frame_type, display_name)
    if not output_dir.exists():
        output_dir.mkdir(parents=True)

    # python's pedantic xml module outputs incompatible namespace prefixes
    generated_xml = tostring(element.getroot(), "utf-8")
    xml_out = replace_namespace_prefixes(generated_xml.decode("utf-8"))

    output_file = output_dir / filename

    with open(output_file, "w") as f:
        f.write(xml_out)

    return output_file


def generate_diagram(
    layout, svg_template, output_dir=DEFAULT_OUTPUT_DIR, diagram_list=[]
):
    """
    generate a motor diagram (svg) for a given frame class and type
    """
    if not layout:  # this should never happen
        print("Motor Diagrams: Encountered null layout.", file=stderr)
        return

    # deep copy, as we may reuse the template for multiple diagrams
    svg = copy.deepcopy(svg_template)
    svg_root = svg.getroot()
    (
        motor_diagram_layers,
        layer_background,
        layer_frame,
        layer_motors,
        layer_motor_numbers,
        layer_motor_letters,
        layer_frame_name,
        layer_frame_notes,
    ) = locate_svg_elements(svg_root)

    base_font_size = float(motor_diagram_layers.get("font-size"))
    frame_notes_font_size = float(layer_frame_notes.get("font-size"))

    # calculate unique motor positions
    unique_motor_positions, is_coaxial = calculate_unique_motor_positions(layout)

    # scale diagram based on number of motor positions
    frame_display_radius = DIAGRAM_SCALING[unique_motor_positions]

    # dodecahexa needs a little more room to display
    frame_display_radius *= DODECAHEXA_MULTIPLIER if len(layout["motors"]) == 12 else 1

    motor_center_points = {0: (0, 0)}
    extents = Extents(minX=0, minY=0, maxX=0, maxY=0)

    # add motor elements (rotor arc/direction and output numbering)
    for motor in layout["motors"]:
        test_order = motor["TestOrder"]
        roll, pitch = float(motor["Roll"]), float(motor["Pitch"])
        x, y, r, θ = get_translated_coordinates(roll, pitch, frame_display_radius)

        motor_rotation = motor["Rotation"]
        if motor_rotation == "?":
            motor_rotation = "NYT"

        if is_coaxial:
            remainder = 0
            if len(layout["motors"]) == 4:
                remainder = 1  # for QUAD Y4A
            layer_suffix = "-bottom" if test_order % 2 == remainder else "-top"
            if "normal" in motor_rotation:
                layer_suffix = "-middle"
            rotation_suffix = ""
            if "-" not in motor_rotation and test_order % 2 == remainder:
                rotation_suffix = "-flipped"
            motor_rotation = f"{motor_rotation}{rotation_suffix}"
            layer_motors = svg_root.find(f'.//*[@id="layer-motors{layer_suffix}"]')
            layer_motor_numbers = svg_root.find(
                f'.//*[@id="layer-numbers{layer_suffix}"]'
            )
            layer_motor_letters = svg_root.find(
                f'.//*[@id="layer-letters{layer_suffix}"]'
            )
        elif y > 0:
            motor_rotation = f"{motor_rotation}-flipped"

        motor_element = append_svg_element(layer_motors, "use", x, y, motor_rotation)
        number_element = append_svg_element(
            layer_motor_numbers, "text", x, y, str(motor["Number"])
        )
        motor_center_points[motor["Number"]] = (x, y)
        extents.update(
            min(extents.minX, x - SINGLE_MOTOR_RADIUS),
            min(extents.minY, y - SINGLE_MOTOR_RADIUS),
            max(extents.maxX, x + SINGLE_MOTOR_RADIUS),
            max(extents.maxY, y + SINGLE_MOTOR_RADIUS),
        )

        # add motor letter (red font for test order)
        letter_element = append_motor_letter(
            is_coaxial,
            motor,
            layer_motor_letters,
            base_font_size,
            extents,
            x,
            y,
            r,
            θ,
            unique_motor_positions,
            test_order,
        )

        # quad a-tail and v-tail have unique displays
        if "tail" in layout["TypeName"].lower():
            handle_av_tails(layout, motor_element, number_element, letter_element, x, y)
    """ END for motor in layout['motors'] """

    # draw frame
    append_frame(
        svg_root,
        layout,
        layer_frame,
        is_coaxial,
        motor_center_points,
        frame_display_radius,
    )

    # adjust diagram extents for coaxial frame 3d skew in y
    if is_coaxial:
        extents = Extents(
            minX=extents.minX,
            minY=extents.minY * COAXIAL_Y_SCALE,
            maxX=extents.maxX,
            maxY=extents.maxY * COAXIAL_Y_SCALE,
        )

    # append frame class/name and notes
    frame_name = append_footer_text(
        layout,
        layer_frame_name,
        layer_frame_notes,
        extents,
        base_font_size,
        frame_notes_font_size,
    )

    # set final size, view box, and background
    svg_width, svg_height = finalize_diagram(svg_root, layer_background, extents)

    # write to file
    output_file = write_svg_file(
        svg, layout["Class"], layout["Type"], frame_name, output_dir
    )

    # update diagram list
    list_entry = {
        "filename": output_file.name,
        "ClassName": layout["ClassName"],
        "TypeName": layout["TypeName"],
        "width": svg_width,
        "height": svg_height,
    }
    if "WikiNotes" in layout:
        list_entry["wiki_notes"] = layout["WikiNotes"]
    diagram_list.append(list_entry)

    return output_file


def init_data():
    """
    initialize data structures from external files
    """
    json = get_motors_json()
    template = load_svg_template(SVG_TEMPLATE_FILE)
    if not json:
        print("Exiting on error.", file=stderr)
        exit(1)
    return json, template


def generate_single_diagram():
    """
    debug function to generate a single motor diagram for review
    """
    motors_json, svg_template = init_data()

    frame_classes = {
        layout["Class"]: layout["ClassName"] for layout in motors_json["layouts"]
    }
    print("\nAvailable frame classes:")
    for class_id, class_name in sorted(frame_classes.items()):
        print(f"{class_id}: {class_name}")

    try:
        frame_class = int(input("Enter frame class: "))
        if frame_class not in frame_classes:
            raise ValueError(f"FRAME_CLASS={frame_class} does not exist.")

        frame_types = [
            layout
            for layout in motors_json["layouts"]
            if layout["Class"] == frame_class and not layout.get("Skip", False)
        ]
        if not frame_types:
            raise ValueError(f"No available frame types for FRAME_CLASS={frame_class}.")

        print(f"\nAvailable frame types for FRAME_CLASS={frame_class}:")
        for selected_layout in frame_types:
            print(f'{selected_layout["Type"]}: {selected_layout["TypeName"]}')

        frame_type = int(input("Enter frame type: "))
        selected_layout = next(
            (layout for layout in frame_types if layout["Type"] == frame_type), None
        )
        if selected_layout is None:
            raise ValueError(
                f"FRAME_TYPE={frame_type} does not exist for FRAME_CLASS={frame_class}."
            )

        diagram_list = []
        output_file = generate_diagram(
            selected_layout, svg_template, DEFAULT_OUTPUT_DIR, diagram_list
        )
        print(f"\nGenerated '{output_file.relative_to(THIS_SCRIPT.parent)}'")

        wiki_diagram_str = generate_wiki_image_tag(diagram_list[0])
        print("\nUse wiki image tag:\n")
        print(wiki_diagram_str.strip())

    except ValueError as e:
        print(str(e), file=stderr)
        print("No diagram generated.", file=stderr)


def generate_wiki_image_tag(diagram):
    # calculate scale (generated images vary in size slightly)
    scale = int(100 * BASE_IMAGE_SIZE / diagram["height"])
    # if aspect ratio is a bit wide, adjust for display
    if diagram["width"] / diagram["height"] > 1.25:
        scale = int(100 * BASE_IMAGE_SIZE / diagram["width"])
    wiki_image_str = f'.. image:: ../images/{diagram["filename"]}\n'
    wiki_image_str += f'    :target: ../_images/{diagram["filename"]}\n'
    wiki_image_str += f"    :scale: {scale}%\n"
    wiki_image_str += f'    :alt: {diagram["ClassName"]} {diagram["TypeName"]}\n\n'
    if "wiki_notes" in diagram:
        for note in diagram["wiki_notes"]:
            wiki_image_str += f".. note::\n    {note}\n\n"
    return wiki_image_str


def generate_all_diagrams():
    """
    generate all motor diagrams for all frame classes and types
    """
    diagram_list = []
    motors_json, svg_template = init_data()
    layouts = [
        layout for layout in motors_json["layouts"] if not layout.get("Skip", False)
    ]
    layout_count = len(layouts)
    last_line_len = 0
    for i in range(layout_count):
        output_file = generate_diagram(
            layouts[i], svg_template, diagram_list=diagram_list
        )
        percentage = (i + 1) / layout_count * 100
        msg = f"Generating motor diagrams: [{percentage:3.0f}%] '{output_file.relative_to(THIS_SCRIPT.parent)}'"
        pad_count = max(0, last_line_len - len(msg))
        last_line_len = len(msg)
        stderr.write(f'\r{msg}{" " * pad_count}')
        stderr.flush()
    print(f"\nGenerated {len(diagram_list)} motor diagrams.", file=stderr)
    return diagram_list


def generate_wiki_page(diagram_list, preview=True):
    """
    generate a markdown page for the copter wiki
    adds a warning to the top of the file regarding direct editing
    replaces the stub comment with the generated diagrams
    outputs to copter/source/docs/
    """
    msg = (
        "\nWiki section preview:"
        if preview
        else f"Replacing image tags in '{WIKI_OUTPUT_FILE.relative_to(THIS_SCRIPT.parent)}' ..."
    )
    print(f"{msg}", end="", file=stderr)
    stderr.flush()

    # sort by filename to ensure consistent output
    diagram_list = sorted(diagram_list, key=lambda d: d["filename"])

    # generate image/note tags
    wiki_diagram_str = ""
    current_frame_class = ""
    for diagram in diagram_list:
        # add section header for each frame class
        if diagram["ClassName"] != current_frame_class:
            current_frame_class = diagram["ClassName"]
            wiki_diagram_str += f'{diagram["ClassName"]} FRAMES\n'
            wiki_diagram_str += "-" * len(f'{diagram["ClassName"]} FRAMES') + "\n\n"

        # append image tag and any notes from AP_Motors_display.json
        wiki_diagram_str += generate_wiki_image_tag(diagram)

    if preview:
        print(f"\n\n{wiki_diagram_str.strip()}\n\n")
        return

    with open(WIKI_OUTPUT_FILE, "r") as file:
        file_contents = file.read()
        file.close()

    # insert file content
    file_start = file_contents.split(WIKI_BEGIN_COMMENT)[0]
    file_end = file_contents.split(WIKI_END_COMMENT)[1]
    file_contents = ("\n\n").join(
        [
            file_start + WIKI_BEGIN_COMMENT,
            wiki_diagram_str.strip(),
            WIKI_END_COMMENT + file_end,
        ]
    )

    with open(WIKI_OUTPUT_FILE, "w") as file:
        file.write(file_contents)

    print("done.", file=stderr)


def build_all(preview=True):
    """
    generate all motor diagrams for all frame classes and types
    """
    start_time = time.time()
    print("Motor diagrams: Starting build...", file=stderr)

    diagram_list = generate_all_diagrams()
    generate_wiki_page(diagram_list, preview)

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"Motor diagrams: Build complete ({elapsed_time:.3f} seconds).", file=stderr)


def clean(directory="."):
    """
    Delete files matching the patterns m_0*.svg and m_1*.svg in the specified directory.
    """
    patterns = ["m_0*.svg", "m_1*.svg"]
    dir_path = pathlib.Path(directory)
    files_to_delete = []

    for pattern in patterns:
        files_to_delete.extend(dir_path.glob(pattern))

    file_count = len(files_to_delete)
    if file_count == 0:
        print(
            f"Motor diagrams directory clean: '{directory.relative_to(THIS_SCRIPT.parent)}'",
            file=stderr,
        )
        return
    last_line_len = 0
    err_count = 0
    for i in range(file_count):
        percentage = (i + 1) / file_count * 100
        try:
            os.remove(files_to_delete[i])
            msg = f"Cleaning svg files: [{percentage:3.0f}%] '{files_to_delete[i].relative_to(THIS_SCRIPT.parent)}'"
            pad_count = max(0, last_line_len - len(msg))
            last_line_len = len(msg)
            stderr.write(f'\r{msg}{" " * pad_count}')
            stderr.flush()
        except OSError as e:
            err_count += 1
            print(f"\nError deleting file {files_to_delete[i]}: {e}", file=stderr)
    print(
        f"\nCleaned {file_count - err_count} files in '{directory.relative_to(THIS_SCRIPT.parent)}'",
        file=stderr,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=(
            "Generate ArduPilot Copter motor diagram(s) and optionally insert them into the "
            "associated Copter wiki page."
        )
    )
    parser.add_argument(
        "-s",
        "--single",
        action="store_true",
        help="Generate a single motor diagram (prompts for input).",
    )
    parser.add_argument(
        "-d",
        "--diagrams",
        action="store_true",
        help="Generate all motor diagrams and exit (do not build or output the copter wiki page).",
    )
    parser.add_argument(
        "-p",
        "--preview",
        action="store_true",
        help="Generate all motor diagrams and preview the copter wiki page diagram section (outputs to stdout).",
    )
    parser.add_argument(
        "-b",
        "--build",
        action="store_true",
        help="Generate all motor diagrams and insert them into the wiki page.",
    )
    parser.add_argument(
        "-c",
        "--clean",
        action="store_true",
        help="Delete files matching m_0*.svg and m_1*.svg in the images and script directories.",
    )

    if len(argv) == 1:
        parser.print_help()
        exit(1)

    args = parser.parse_args()

    if args.single:
        generate_single_diagram()
        exit(0)

    if args.clean:
        clean(THIS_SCRIPT.parent)
        clean(DEFAULT_OUTPUT_DIR)

    if args.diagrams:
        generate_all_diagrams()
        exit(0)

    if args.build or args.preview:
        build_all(preview=args.preview)
        exit(0)
