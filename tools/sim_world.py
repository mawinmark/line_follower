# Tiny 2D simulator: a diff-drive robot on a floor with a line track.
# Publishes a perspective camera view on /image_raw, integrates /cmd_vel, and reports progress.
#   python3 sim_world.py T0 DURATION ccw|cw OUT_PREFIX
# Environment (defaults = calibrated to the real robot with the old, high camera):
#   TRACK=rect|u   RADIUS (rect corner radius)   UR, UL (U: bend radius, arm length)
#   LATENCY CMD_DELAY MOTOR_TAU FPS DEADBAND   NEAR NEAR_HW FAR FAR_HW (camera ground footprint)
#   LINE (line colour as "h,s,v", OpenCV HSV)
import collections
import math
import os
import sys
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

T0, DURATION, DIRECTION, OUT = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3], sys.argv[4]
env = os.environ.get
PPM = 200                 # world map pixels per meter
SIZE_M = 4.0              # world is 4 m x 4 m
TRACK_KIND = env('TRACK', 'rect')
LATENCY = float(env('LATENCY', 0.35))     # camera delay (s)
RADIUS = float(env('RADIUS', 0.03))       # rect: corner radius (m)
UR = float(env('UR', 0.3))                # U: bend radius (m)
UL = float(env('UL', 0.8))                # U: straight arm length (m)
FPS = float(env('FPS', 10))               # camera frame rate
DEADBAND = float(env('DEADBAND', 0))      # wheel speed below this (m/s) = no motion
CMD_DELAY = float(env('CMD_DELAY', 0.15))  # cmd_vel delay (s)
MOTOR_TAU = float(env('MOTOR_TAU', 0.35))  # motor response time constant (s)
WHEEL_SEP = 0.16                          # distance between wheels (m)
LINE_W = 0.05                             # line width (m)
# camera footprint on the ground, robot frame (x forward, y left)
NEAR, NEAR_HW = float(env('NEAR', 0.32)), float(env('NEAR_HW', 0.045))
FAR, FAR_HW = float(env('FAR', 1.2)), float(env('FAR_HW', 0.17))
IMG_W, IMG_H = 320, 240


def hsv_to_bgr(h, s, v):
    return [int(c) for c in cv2.cvtColor(np.uint8([[[h, s, v]]]), cv2.COLOR_HSV2BGR)[0, 0]]


FLOOR = hsv_to_bgr(140, 100, 100)   # purple floor (measured on real camera)
LINE = hsv_to_bgr(*[int(c) for c in env('LINE', '17,66,178').split(',')])  # pale yellow


def polyline(segs, step=0.01):
    pts = []
    for s in segs:
        if s[0] == 'arc':
            _, ax, ay, r, a0, a1 = s
            n = max(2, int(abs(math.radians(a1 - a0)) * r / step))
            for i in range(n):
                a = math.radians(a0 + (a1 - a0) * i / n)
                pts.append((ax + r * math.cos(a), ay + r * math.sin(a)))
        else:
            (sx, sy), (ex, ey) = s
            n = max(2, int(math.hypot(ex - sx, ey - sy) / step))
            pts += [(sx + (ex - sx) * i / n, sy + (ey - sy) * i / n) for i in range(n)]
    return np.array(pts)


def rounded_rect(cx, cy, w, h, r):
    # centerline (counter-clockwise), starting at the middle of the bottom side
    x0, x1, y0, y1 = cx - w / 2 + r, cx + w / 2 - r, cy - h / 2 + r, cy + h / 2 - r
    return polyline([((cx, cy - h / 2), (x1, cy - h / 2)), ('arc', x1, y0, r, -90, 0),
                     ((cx + w / 2, y0), (cx + w / 2, y1)), ('arc', x1, y1, r, 0, 90),
                     ((x1, cy + h / 2), (x0, cy + h / 2)), ('arc', x0, y1, r, 90, 180),
                     ((cx - w / 2, y1), (cx - w / 2, y0)), ('arc', x0, y0, r, 180, 270),
                     ((x0, cy - h / 2), (cx, cy - h / 2))])


def u_track(cx, yc, r, arm):
    # U whose open end is closed by a straight line: round bend at the bottom,
    # two sharp 90-degree corners at the top. Counter-clockwise, starting at the top-right corner.
    top = yc + arm
    return polyline([((cx + r, top), (cx - r, top)), ((cx - r, top), (cx - r, yc)),
                     ('arc', cx, yc, r, 180, 360), ((cx + r, yc), (cx + r, top))])


if TRACK_KIND == 'u':
    CX, YC = 2.0, 1.4
    TRACK = u_track(CX, YC, UR, UL)
    # start in the middle of an arm, heading toward the round bend
    START = ((CX - UR, YC + UL / 2, -math.pi / 2) if DIRECTION == 'ccw'
             else (CX + UR, YC + UL / 2, -math.pi / 2))
else:
    TRACK = rounded_rect(2.0, 2.0, 1.6, 1.2, RADIUS)
    # start 0.7 m before the first corner
    START = ((2.8 - RADIUS - 0.7, 1.4, 0.0) if DIRECTION == 'ccw'
             else (1.2 + RADIUS + 0.7, 1.4, math.pi))
if DIRECTION == 'cw':
    TRACK = TRACK[::-1]
SEG = np.hypot(*np.diff(TRACK, axis=0, append=TRACK[:1]).T)
CUM = np.concatenate([[0], np.cumsum(SEG)[:-1]])
LAP = SEG.sum()


def to_px(x, y):
    return x * PPM, (SIZE_M - y) * PPM


world = np.full((int(SIZE_M * PPM), int(SIZE_M * PPM), 3), FLOOR, np.uint8)
poly = np.array([to_px(x, y) for x, y in TRACK], np.int32)
cv2.polylines(world, [poly], True, LINE, int(LINE_W * PPM), cv2.LINE_AA)


class Sim(Node):
    def __init__(self):
        super().__init__('sim_world')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/image_raw', 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.x, self.y, self.th = START
        self.v = self.w = self.tv = self.tw = 0.0
        self.cmdq = collections.deque()
        self.devlog = []
        self.idx = int(np.argmin(np.hypot(TRACK[:, 0] - self.x, TRACK[:, 1] - self.y)))
        self.progress = 0.0
        self.max_dev = 0.0
        self.path = []
        self.queue = collections.deque()
        self.last = time.time()
        self.log = []
        self.create_timer(0.02, self.step)
        self.create_timer(1 / FPS, self.camera)

    def on_cmd(self, m):
        self.cmdq.append((time.time() + CMD_DELAY, m.linear.x, m.angular.z))

    def step(self):
        now = time.time()
        dt, self.last = now - self.last, now
        while self.cmdq and self.cmdq[0][0] <= now:
            _, self.tv, self.tw = self.cmdq.popleft()
        k = 1.0 if MOTOR_TAU <= 0 else min(1.0, dt / MOTOR_TAU)
        self.v += (self.tv - self.v) * k
        self.w += (self.tw - self.w) * k
        vl, vr = self.v - self.w * WHEEL_SEP / 2, self.v + self.w * WHEEL_SEP / 2
        vl, vr = (0.0 if abs(vl) < DEADBAND else vl), (0.0 if abs(vr) < DEADBAND else vr)
        v, w = (vl + vr) / 2, (vr - vl) / WHEEL_SEP
        self.th += w * dt
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        # progress along the track: search near the previous index
        n = len(TRACK)
        window = (self.idx + np.arange(-30, 60)) % n
        d = np.hypot(TRACK[window, 0] - self.x, TRACK[window, 1] - self.y)
        new = int(window[np.argmin(d)])
        delta = (CUM[new] - CUM[self.idx]) % LAP
        if delta > LAP / 2:
            delta -= LAP
        self.progress += delta
        self.idx = new
        dev = float(d.min())
        self.max_dev = max(self.max_dev, dev)
        self.devlog.append((now - T0, dev))
        self.path.append((self.x, self.y))
        t = now - T0
        if not self.log or t - self.log[-1][0] >= 1.0:
            self.log.append((t, self.progress, dev, self.v, self.w))
        while self.queue and self.queue[0][0] <= now:
            self.pub.publish(self.queue.popleft()[1])

    def camera(self):
        c, s = math.cos(self.th), math.sin(self.th)
        ground = [(NEAR, NEAR_HW), (NEAR, -NEAR_HW), (FAR, FAR_HW), (FAR, -FAR_HW)]
        src = np.float32([to_px(self.x + c * fx - s * fy, self.y + s * fx + c * fy)
                          for fx, fy in ground])
        dst = np.float32([(0, IMG_H - 1), (IMG_W - 1, IMG_H - 1), (0, 0), (IMG_W - 1, 0)])
        img = cv2.warpPerspective(world, cv2.getPerspectiveTransform(src, dst), (IMG_W, IMG_H))
        img = cv2.GaussianBlur(img, (5, 5), 0)
        self.queue.append((time.time() + LATENCY, self.bridge.cv2_to_imgmsg(img, 'bgr8')))


rclpy.init()
sim = Sim()
while time.time() < T0 + DURATION + 1.0:
    rclpy.spin_once(sim, timeout_sec=0.01)

print(f'--- {TRACK_KIND} {DIRECTION}: lap length {LAP:.2f} m')
for t, p, dev, v, w in sim.log[::2]:
    print(f'  t={t:5.1f}s  progress={p:5.2f} m  off-line={dev * 100:5.1f} cm  '
          f'v={v:+.2f}  w={w:+.2f}')
print(f'RESULT {TRACK_KIND} {DIRECTION}: progress {sim.progress:.2f} m '
      f'({sim.progress / LAP:.2f} laps), max off-line {sim.max_dev * 100:.1f} cm '
      f'[UR={UR} near={NEAR} far={FAR}]')
# top-down picture of the driven path
view = world.copy()
cv2.polylines(view, [np.array([to_px(x, y) for x, y in sim.path], np.int32)], False,
              (0, 0, 255), 3)
cv2.imwrite(f'{OUT}_{TRACK_KIND}_{DIRECTION}.png', cv2.resize(view[100:700, 100:700], (450, 450)))
sim.destroy_node()
rclpy.shutdown()
