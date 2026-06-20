#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/zyc/vision_sim_ws/install/setup.bash

MODEL_PATH="${YOLO_MODEL:-/home/zyc/vision_sim_ws/models/irreality.pt}"
CONF="${YOLO_CONF:-0.35}"
IOU="${YOLO_IOU:-0.1}"
DEVICE="${YOLO_DEVICE:-0}"
IMGSZ="${YOLO_IMGSZ:-640}"
YOLO_HZ="${YOLO_HZ:-12}"

python3 /home/zyc/vision_sim_ws/scripts/d435i_mavros_opencv_viewer.py --source ros --color-topic /camera/camera/color/image_raw --depth-topic /camera/camera/depth/image_rect_raw --yolo-model "$MODEL_PATH" --yolo-conf "$CONF" --yolo-iou "$IOU" --yolo-device "$DEVICE" --yolo-imgsz "$IMGSZ" --yolo-hz "$YOLO_HZ"
