#!/usr/bin/env python3
import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from mine_slam_calibration.extrinsics import audit_extrinsics, load_extrinsics


def main():
    parser = argparse.ArgumentParser(description="Audit Tunnel-LIO extrinsics YAML.")
    parser.add_argument("file", help="Path to extrinsics.yaml")
    args = parser.parse_args()

    issues = audit_extrinsics(load_extrinsics(args.file))
    if issues:
        for issue in issues:
            print("ERROR: {}".format(issue), file=sys.stderr)
        return 1

    print("OK: extrinsics audit passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
