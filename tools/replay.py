# Open-loop replay: feed recorded frames into the real node and print its decisions
# (nothing is published).
#   python3 replay.py BAG_NAME START END [key=value ...]     e.g. replay.py test_curve6 80 95
import os
import sqlite3
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import rclpy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from line_follower import line_follower as lf  # noqa: E402

bag, a, b = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
clock = [0.0]
lf.time.monotonic = lambda: clock[0]          # node runs on bag time
rclpy.init()
node = lf.LineFollower()
node.cmd_vel_pub.publish = lambda msg: None   # never publish anything
for kv in sys.argv[4:]:
    k, v = kv.split('=')
    setattr(node, k, float(v))
node.running = True
log = []
orig_drive, orig_stop = node.drive, node.stop_robot


def drive(lin, ang, action):
    log.append((clock[0], lin, ang, action))
    orig_drive(lin, ang, action)


def stop_robot():
    log.append((clock[0], 0.0, 0.0, 'หยุด'))
    orig_stop()


node.drive, node.stop_robot = drive, stop_robot
bridge = CvBridge()
db = sqlite3.connect(os.path.expanduser(f'~/{bag}/{bag}_0.db3'))
topics = {tid: n for tid, n in db.execute('SELECT id, name FROM topics')}
t0 = None
rows = db.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp')
for tid, ts, raw in rows:
    t0 = t0 or ts * 1e-9
    t = ts * 1e-9 - t0
    if topics[tid] != '/image_raw/compressed' or not a <= t <= b:
        continue
    clock[0] = t
    img = cv2.imdecode(np.frombuffer(bytes(deserialize_message(raw, CompressedImage).data),
                                     np.uint8), 1)
    node.image_callback(bridge.cv2_to_imgmsg(img, 'bgr8'))
prev = None
for t, lin, ang, act in log:
    if act != prev:
        print(f'  {t:5.1f}s  {act}  (v={lin:+.2f}, w={ang:+.2f})')
        prev = act
node.destroy_node()
rclpy.shutdown()
