#!/usr/bin/env bash

# 1. Start the rosbag2 recording
OUTPUT_DIR="tactile_recording_$(date +%Y%m%d_%H%M%S)"
START_TIME=$(date +%s)

echo "Starting rosbag2 recording in folder: $OUTPUT_DIR"
ros2 bag record \
  --output "$OUTPUT_DIR" \
  /tactile_input_left \
  /tactile_input_right &
RECORD_PID=$!

trap "echo 'Stopping recording...'; kill -INT $RECORD_PID" SIGINT

while kill -0 $RECORD_PID 2>/dev/null; do
    NOW=$(date +%s)
    ELAPSED=$((NOW - START_TIME))
    echo "Recording for $ELAPSED seconds..."
    sleep 5
done

# 2. Automatically convert the bag to JSON
echo "Converting bag to JSON..."

python3 <<EOF
#!/usr/bin/env python3

import os
import json
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message

try:
    from rummy_tactile_msgs.msg import TactileInput
except ImportError:
    TactileInput = None
    print("ERROR: rummy_tactile_msgs is not installed or TactileInput is unavailable.")
    sys.exit(1)

def extract_tactile_from_bag(bag_path, topic):
    """
    Extract TactileInput messages from the ROS 2 bag on the specified topic.
    Returns two lists: (times_str, values).
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
            # Store the raw time string from msg.local_time
            times_str.append(msg.local_time)
            # Convert the array-like 'msg.data' to a Python list
            values.append(list(msg.data))

    print(f"Found {len(times_str)} messages in {topic}")
    return times_str, values

def main():
    bag_path = "${OUTPUT_DIR}"
    left_topic = "/tactile_input_left"
    right_topic = "/tactile_input_right"

    bag_basename = os.path.basename(os.path.normpath(bag_path))
    out_json = os.path.join(bag_path, f"{bag_basename}.json")

    left_times_str, left_values = extract_tactile_from_bag(bag_path, left_topic)
    right_times_str, right_values = extract_tactile_from_bag(bag_path, right_topic)

    out_dict = {
        f"topic:{left_topic}": {
            "times": left_times_str,
            "values": left_values
        },
        f"topic:{right_topic}": {
            "times": right_times_str,
            "values": right_values
        }
    }

    with open(out_json, "w") as f:
        json.dump(out_dict, f, indent=2)

    print(f"Saved tactile data to {out_json}")

if __name__ == "__main__":
    main()
EOF
