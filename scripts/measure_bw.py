#!/usr/bin/env python3
"""
Single-process bandwidth measurement for all ROS topics.
Subscribes to every topic simultaneously in one node — no TCP contention
from multiple rostopic processes.
"""
import sys
import time
import threading
import math
import subprocess
import signal
from collections import defaultdict

import rospy

WINDOW = 10.0  # seconds to measure

# Import message types lazily — rospy handles this via the subscriber


def main():
    # Get all topics
    topics = subprocess.check_output(
        "source /opt/ros/noetic/setup.bash && source ~/ws_livox/devel/setup.bash && rostopic list 2>/dev/null",
        shell=True, executable='/bin/bash'
    ).decode().strip().split('\n')
    topics = [t.strip() for t in topics if t.strip()]

    if not topics:
        print("ERROR: No topics found")
        return 1

    print(f"Measuring bandwidth for {len(topics)} topics (window={WINDOW}s, single ROS node)...")
    print(f"Start time: {time.strftime('%H:%M:%S')}")
    print()

    # Get topic types first
    topic_types = {}
    for t in topics:
        try:
            r = subprocess.run(
                f"source /opt/ros/noetic/setup.bash && source ~/ws_livox/devel/setup.bash && timeout 3 rostopic info {t} 2>/dev/null",
                shell=True, executable='/bin/bash', capture_output=True, text=True, timeout=5
            )
            for line in r.stdout.split('\n'):
                if 'Type:' in line:
                    topic_types[t] = line.split('Type:')[1].strip()
                    break
        except Exception:
            pass

    # Stats per topic
    stats = defaultdict(lambda: {'bytes': 0, 'msgs': 0, 'first': None, 'last': None})
    lock = threading.Lock()
    start_time = None
    subscribers = []
    finished = threading.Event()

    def make_callback(topic):
        def cb(msg, _topic=topic):
            with lock:
                now = time.time()
                s = stats[_topic]
                if s['first'] is None:
                    s['first'] = now
                s['last'] = now
                s['msgs'] += 1
                # AnyMsg stores raw wire bytes in _buff
                try:
                    s['bytes'] += len(msg._buff)
                except Exception:
                    pass
        return cb

    # Initialize ROS node
    rospy.init_node('bw_meter', anonymous=True, disable_signals=True)

    # Subscribe to all topics
    for t in topics:
        try:
            sub = rospy.Subscriber(t, rospy.AnyMsg, make_callback(t))
            subscribers.append(sub)
        except Exception as e:
            print(f"  WARN: cannot subscribe to {t}: {e}")

    print(f"Subscribed to {len(subscribers)} topics.")
    print(f"Measuring for {WINDOW} seconds...")
    sys.stdout.flush()

    start_time = time.time()

    # Spin with a timer
    timer_start = time.time()
    rate = rospy.Rate(100)  # 100Hz spin
    while not rospy.is_shutdown() and (time.time() - timer_start) < WINDOW:
        # Process pending callbacks
        try:
            # Poll with short timeout
            rospy.rostime.wallsleep(0.01)
        except Exception:
            pass

    end_time = time.time()
    elapsed = end_time - start_time

    # Unsubscribe all
    for sub in subscribers:
        sub.unregister()

    # Print results
    print()
    print(f"{'Topic':<60} {'Bandwidth':>15}  {'Msgs':>6}  {'MsgType':>35}")
    print("-" * 120)

    has_data = 0
    no_data = 0

    for t in sorted(stats.keys(), key=lambda x: stats[x]['bytes'], reverse=True):
        s = stats[t]
        bw = s['bytes'] / elapsed if elapsed > 0 else 0
        bw_s = format_bw(bw)
        mtype = topic_types.get(t, '?')[:35]

        if s['msgs'] == 0:
            print(f"{t:<60} {'0 (no msgs)':>15}  {s['msgs']:>6}  {mtype:<35}")
            no_data += 1
        else:
            print(f"{t:<60} {bw_s:>15}  {s['msgs']:>6}  {mtype:<35}")
            has_data += 1

    print("-" * 120)
    print(f"Active: {has_data}   Idle: {no_data}   Total: {len(topics)}")
    print(f"Elapsed: {elapsed:.1f}s")

    # Also show the no-subscription topics
    no_sub = [t for t in topics if t not in stats]
    if no_sub:
        print(f"Could not subscribe: {len(no_sub)}")
        for t in no_sub:
            print(f"  {t}")


def format_bw(bytes_per_sec):
    if bytes_per_sec < 1024:
        return f"{bytes_per_sec:.2f}B/s"
    elif bytes_per_sec < 1024 * 1024:
        return f"{bytes_per_sec / 1024:.2f}KB/s"
    else:
        return f"{bytes_per_sec / (1024 * 1024):.2f}MB/s"


if __name__ == '__main__':
    main()
