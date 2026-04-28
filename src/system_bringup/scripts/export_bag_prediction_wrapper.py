#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from pathlib import Path

import rospy
import rospkg


def _get_script_path() -> Path:
    pkg_dir = Path(rospkg.RosPack().get_path("system_bringup"))
    ws_dir = pkg_dir.parent.parent
    script_path = ws_dir / "scripts" / "export_bag_prediction.py"
    if not script_path.exists():
        raise FileNotFoundError(f"export script not found: {script_path}")
    return script_path


def main() -> None:
    rospy.init_node("export_bag_prediction_wrapper", anonymous=False)

    script_path = _get_script_path()
    argv = [
        sys.executable,
        str(script_path),
        "--video-name",
        rospy.get_param("~video_name", "塔吊标准节-静态预警-评估"),
        "--output",
        rospy.get_param("~output", "/home/nvidia/ws_livox/data/prediction_塔吊标准节_静态预警_评估.csv"),
        "--warning-topic",
        rospy.get_param("~warning_topic", "/warning/state"),
        "--reference-topic",
        rospy.get_param("~reference_topic", "/livox/imu_192_168_1_125"),
        "--bag-start-sec",
        str(rospy.get_param("~bag_start_sec", 1776053869.63)),
        "--bag-play-rate",
        str(rospy.get_param("~bag_play_rate", 1.0)),
        "--idle-timeout-sec",
        str(rospy.get_param("~idle_timeout_sec", 5.0)),
        "--startup-timeout-sec",
        str(rospy.get_param("~startup_timeout_sec", 600.0)),
    ]

    rospy.loginfo("[export_bag_prediction_wrapper] exec: %s", " ".join(argv))
    os.execv(sys.executable, argv)


if __name__ == "__main__":
    main()
