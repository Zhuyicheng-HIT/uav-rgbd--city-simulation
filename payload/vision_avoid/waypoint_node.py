#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline RGB-D flight replay entrypoint.

Replay, simulation, and flight share the same vision implementation in
sim_waypoint_node.py. This wrapper only selects DEBUG mode before importing the
shared pipeline, avoiding another large copy of the algorithm.
"""

import os

os.environ.setdefault("VISION_AVOID_RUN_MODE", "DEBUG")
os.environ.setdefault("VISION_AVOID_DISPLAY", "1")

from sim_waypoint_node import main


if __name__ == "__main__":
    raise SystemExit(main())
