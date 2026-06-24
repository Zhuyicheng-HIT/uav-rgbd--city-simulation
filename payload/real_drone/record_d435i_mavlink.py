#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compatibility entry for the old real-flight recorder name.

The real-flight architecture was changed after testing:

- MAVROS is the only process that opens the flight-controller serial port.
- real_vision_node.py is the only process that opens the D435i pipeline.
- flight_record_mavlink.py records MAVROS flight-controller topics only.

This file keeps the historical command name available, but no longer opens
/dev/ttyACM0 or D435i directly.
"""

from __future__ import annotations

import sys

from flight_record_mavlink import main as mavros_logger_main


def main() -> int:
    print(
        "[WARN] record_d435i_mavlink.py has been replaced by the MAVROS-only "
        "flight recorder.\n"
        "[WARN] RGB-D db3 recording is now handled by real_vision_node.py "
        "with --record-db3.\n"
        "[WARN] Continuing as: python3 flight_record_mavlink.py\n",
        file=sys.stderr,
    )
    return mavros_logger_main()


if __name__ == "__main__":
    raise SystemExit(main())
