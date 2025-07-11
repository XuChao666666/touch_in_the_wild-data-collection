#!/usr/bin/env python3

import os
import argparse
import json
import datetime
import re

import rosbag2_py
from rclpy.serialization import deserialize_message

try:
    from rummy_tactile_msgs.msg import TactileInput
except ImportError:
    TactileInput = None
    print("WARNING: rummy_tactile_msgs is not installed in this environment.")

def extract_tactile_from_bag(bag_path, topic):
    """
    Extract TactileInput from the ROS 2 bag on the specified topic.
    Returns a list of time strings (exactly as in msg.local_time) and list of float arrays (msg.data).

    We convert msg.data to regular Python lists (instead of NumPy arrays) so we can serialize to JSON.
    """
    times_str = []
    values = []

    print(f"Reading bag={bag_path}, topic={topic}")
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    while reader.has_next():
        msg_topic, msg_data, _ = reader.read_next()
        if msg_topic == topic:
            msg = deserialize_message(msg_data, TactileInput)
            # Store the raw time string
            times_str.append(msg.local_time)
            # Convert array-like to plain Python list for JSON compatibility
            values.append(list(msg.data))

    print(f"Found {len(times_str)} messages in {topic}")
    return times_str, values


def main():
    parser = argparse.ArgumentParser(description="Convert ROS bag tactile data to a single .json file.")
    parser.add_argument("--bag", required=True, help="Path to the ROS 2 bag (folder).")
    parser.add_argument("--tactile_topic_left", default="/tactile_input_left",
                        help="Left Tactile topic (default /tactile_input_left).")
    parser.add_argument("--tactile_topic_right", default="/tactile_input_right",
                        help="Right Tactile topic (default /tactile_input_right).")
    parser.add_argument("--out_json", help="Path to output .json file.")
    args = parser.parse_args()

    if not args.out_json:
        bag_basename = os.path.basename(os.path.normpath(args.bag))
        args.out_json = os.path.join(args.bag, f"{bag_basename}.json")

    left_times_str, left_values = extract_tactile_from_bag(args.bag, args.tactile_topic_left)
    right_times_str, right_values = extract_tactile_from_bag(args.bag, args.tactile_topic_right)

    # Pack into a dictionary
    out_dict = {
        f"topic:{args.tactile_topic_left}": {
            "times": left_times_str,
            "values": left_values
        },
        f"topic:{args.tactile_topic_right}": {
            "times": right_times_str,
            "values": right_values
        }
    }

    with open(args.out_json, "w") as f:
        json.dump(out_dict, f, indent=2)

    print(f"Saved tactile data to {args.out_json}")


if __name__ == "__main__":
    main()
