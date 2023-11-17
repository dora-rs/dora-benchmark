#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# to run:
# colcon build
# . ./install/setup.bash
# ros2 run py_pubsub talker & python3 dora_subscriber.py
#

import time

import numpy as np
import pyarrow as pa
from helper import record_results
import dora

pa.array([])

context = dora.experimental.ros2_bridge.Ros2Context()
node = context.new_node(
    "turtle_teleop",
    "/ros2_demo",
    dora.experimental.ros2_bridge.Ros2NodeOptions(rosout=True),
)

topic_qos = dora.experimental.ros2_bridge.Ros2QosPolicies(
    reliable=True, max_blocking_time=0.1
)

subscription_topic = node.create_topic("/topic", "std_msgs::UInt8MultiArray", topic_qos)
# twist_writer = node.create_publisher(turtle_twist_topic)

# turtle_pose_topic = node.create_topic("/turtle1/pose", "turtlesim::Pose", topic_qos)
subscription_reader = node.create_subscription(subscription_topic)


current_size = 8
n = 0
i = 0
latencies = []

NAME = "dora Node"


while True:
    array = subscription_reader.next()

    if array == None:
        continue
    t_received = time.perf_counter_ns()

    data = np.array(array[0]["data"].as_py(), dtype=np.uint8)
    length = len(data)

    if length != current_size:
        if n > 0:
            record_results(NAME, current_size, latencies)
        current_size = length
        n = 0
        start = time.perf_counter_ns()
        latencies = []

    ## Arrow Test: Using Arrow
    t_send = data[:8].view(np.uint64)[0]
    latencies.append((t_received - t_send) / 1000)

    n += 1
    i += 1

    time.sleep(0.1)


record_results(NAME, current_size, latencies)
