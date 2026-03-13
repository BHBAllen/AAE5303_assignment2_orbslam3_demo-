#!/usr/bin/env python3
import argparse
import math
import rosbag


def latlonalt_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """
    Approximate LLA -> local ENU using small-area equirectangular approximation.
    Good enough for a short UAV trajectory like this dataset.
    """
    R = 6378137.0  # Earth radius in meters
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat0_rad = math.radians(lat0)
    lon0_rad = math.radians(lon0)

    east = (lon_rad - lon0_rad) * math.cos(lat0_rad) * R
    north = (lat_rad - lat0_rad) * R
    up = alt - alt0
    return east, north, up


def to_sec(stamp):
    return stamp.secs + stamp.nsecs * 1e-9


def main():
    parser = argparse.ArgumentParser(description="Extract RTK ground truth from ROS bag into TUM format.")
    parser.add_argument("bag", help="Path to ROS bag file")
    parser.add_argument("--output", default="ground_truth.txt", help="Output TUM trajectory file")
    parser.add_argument("--max-attitude-diff", type=float, default=0.05,
                        help="Maximum allowed time difference when matching attitude to RTK (seconds)")
    args = parser.parse_args()

    rtk_topic = "/dji_osdk_ros/rtk_position"
    att_topic = "/dji_osdk_ros/attitude"

    print(f"Reading bag: {args.bag}")

    attitudes = []
    rtk_positions = []

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[att_topic]):
            ts = to_sec(msg.header.stamp)
            q = msg.quaternion
            attitudes.append((ts, q.x, q.y, q.z, q.w))

        for topic, msg, t in bag.read_messages(topics=[rtk_topic]):
            ts = to_sec(msg.header.stamp)
            rtk_positions.append((ts, msg.latitude, msg.longitude, msg.altitude))

    if not attitudes:
        raise RuntimeError(f"No messages found on topic: {att_topic}")
    if not rtk_positions:
        raise RuntimeError(f"No messages found on topic: {rtk_topic}")

    print(f"Loaded {len(attitudes)} attitude messages")
    print(f"Loaded {len(rtk_positions)} RTK position messages")

    # Reference origin = first RTK point
    t0, lat0, lon0, alt0 = rtk_positions[0]

    # Two-pointer nearest-neighbor matching in time
    att_idx = 0
    matched = 0
    skipped = 0

    with open(args.output, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")

        for ts, lat, lon, alt in rtk_positions:
            while att_idx + 1 < len(attitudes) and abs(attitudes[att_idx + 1][0] - ts) <= abs(attitudes[att_idx][0] - ts):
                att_idx += 1

            att_ts, qx, qy, qz, qw = attitudes[att_idx]
            dt = abs(att_ts - ts)

            if dt > args.max_attitude_diff:
                skipped += 1
                continue

            x, y, z = latlonalt_to_enu(lat, lon, alt, lat0, lon0, alt0)
            f.write(f"{ts:.6f} {x:.6f} {y:.6f} {z:.6f} {qx:.7f} {qy:.7f} {qz:.7f} {qw:.7f}\n")
            matched += 1

    print(f"Matched and wrote {matched} poses to {args.output}")
    print(f"Skipped {skipped} RTK poses due to missing nearby attitude")


if __name__ == "__main__":
    main()
