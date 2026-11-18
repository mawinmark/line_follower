import math
import signal
import time
import tkinter as tk
from tkinter import font, ttk

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

CAMERA_TIMEOUT = 1.0  # วินาที: ไม่ได้รับภาพนานกว่านี้ = ถือว่ากล้องหลุด
# ความหน่วงทั้งระบบที่วัดจากหุ่นจริง (ภาพมาถึงช้า ~0.35 s + หุ่นเริ่มขยับหลังสั่ง ~0.56 s)
# ตอนเข้ามุม หุ่นจะไหลไปไกลกว่าที่สั่งเท่ากับ ความเร็ว x ค่านี้ จึงหักออกให้อัตโนมัติ
SYSTEM_DELAY = 0.9
VIEW_WIDTH = 320      # ความกว้างของภาพกล้องที่แสดงในหน้าต่าง (px)


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # Subscribers & Publishers
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        # ค่าที่ปรับได้จากหน้าต่าง UI (นี่คือค่าเริ่มต้นตอนเปิดโปรแกรม)
        self.linear_speed = 0.10   # ความเร็วเดินหน้า (m/s)
        self.angular_speed = 0.30  # ความเร็วเลี้ยวสูงสุด (rad/s) (0.50 ทำให้หุ่นจริงส่ายจนหลุด)
        self.dead_zone = 10.0      # |error| น้อยกว่านี้ (px) = เส้นอยู่กลาง ให้เดินตรง
        self.search_time = 3.0     # เส้นหาย: หมุนหาไปทางที่เห็นเส้นครั้งล่าสุด นานสุดกี่วินาที
        # มุมฉาก: พอมุมเลื่อนลงไปถึงขอบล่างของภาพ ให้เดินตรงต่ออีกกี่เมตรแล้วค่อยหมุน
        # (= ระยะจากจุดบนพื้นที่ขอบล่างภาพมองเห็น ถึงแกนล้อของหุ่น → ควรวัดจากหุ่นจริง)
        # ใส่ระยะที่วัดได้ตรงๆ ได้เลย (โค้ดหักระยะที่หุ่นไหลไปเพราะความหน่วงให้เอง)
        self.corner_distance = 0.30
        self.roi_top = 60.0        # หาเส้นเฉพาะแถบภาพ จากความสูง roi_top %
        self.roi_bottom = 100.0    # ถึง roi_bottom % (นับจากขอบบนของภาพ) = ส่วนที่ใกล้หุ่นที่สุด

        # ช่วงสีของเส้นแบบ HSV ของ OpenCV (H 0–179, S/V 0–255) ปรับได้จากหน้าต่าง
        # ค่าเริ่มต้นวัดจากภาพกล้องจริง: เส้นปกติ H≈14–28 S≈54–101, เส้นที่โดนแสงไฟจนซีด
        # H≈31–35 S≈7–70 V≈215–234, พื้นม่วง H≈135–140, ผนัง/กล่องสีขาว H≈126–173
        self.h_min = 10.0
        self.h_max = 40.0
        self.s_min = 15.0
        self.v_min = 100.0

        # สถานะ ให้ UI เอาไปแสดง
        self.running = False         # เปิดมาหุ่นยังไม่วิ่ง จนกว่าจะกดปุ่มเริ่ม
        self.error = None            # None = ไม่เจอเส้น
        self.action = 'หยุด'         # คำสั่งล่าสุดที่ส่งให้หุ่น (แสดงในหน้าต่าง)
        self.last_image_time = None  # เวลาที่ได้ภาพล่าสุด
        self.angle = None            # มุมของเส้น (องศา) None = ไม่เจอเส้น
        self.trend = 0.0             # แนวโน้มการเลี้ยวช่วงหลัง (+ = ขวา) ใช้เลือกทางหมุนหา
        self.last_seen_time = None   # เวลาที่เห็นเส้นครั้งล่าสุด
        self.mode = 'follow'         # follow = ตามเส้น, corner_forward/corner_turn = เข้ามุมฉาก
        self.mode_start = 0.0        # เวลาที่เริ่ม mode ปัจจุบัน
        self.corner_dir = 0          # เส้นข้างหน้าหักไปทาง: +1 = ขวา, -1 = ซ้าย, 0 = ยังไม่เห็นมุม
        self.view = None             # ภาพกล้องล่าสุด (มีจุดเขียว) ให้ UI แสดง
        self.mask = None             # mask สีเหลืองล่าสุด ให้ UI แสดง
        self.new_frame = False       # True = มีภาพใหม่ที่ UI ยังไม่ได้แสดง

        # เช็กทุก 0.2 วินาที ว่ากล้องยังส่งภาพมาไหม
        self.create_timer(0.2, self.check_camera)

    def start(self):
        self.running = True
        self.mode, self.corner_dir = 'follow', 0
        self.get_logger().info(
            f'Start: forward {self.linear_speed:.2f} m/s, turn {self.angular_speed:.2f} rad/s, '
            f'corner distance {self.corner_distance:.2f} m, '
            f'straight if |error| < {self.dead_zone:.0f} px, search {self.search_time:.1f} s, '
            f'ROI {self.roi_top:.0f}-{self.roi_bottom:.0f} %')

    def stop(self):
        self.running = False
        self.mode, self.corner_dir = 'follow', 0
        self.stop_robot()  # ส่งความเร็ว 0 ทันที ไม่ต้องรอภาพถัดไป
        self.get_logger().info('Stop')

    def camera_ok(self):
        return (self.last_image_time is not None
                and time.monotonic() - self.last_image_time < CAMERA_TIMEOUT)

    def check_camera(self):
        # กำลังวิ่งแต่ภาพไม่มา → หยุดไว้ก่อน กันหุ่นวิ่งต่อทั้งที่มองไม่เห็นเส้น
        # (ภาพกลับมาเมื่อไหร่ ก็เดินตามเส้นต่อเอง)
        if self.running and not self.camera_ok():
            self.stop_robot()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        height, width, _ = frame.shape

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for yellow color (ปรับได้จากหน้าต่าง)
        lower_yellow = np.array([self.h_min, self.s_min, self.v_min], dtype=int)  # Lower bound
        upper_yellow = np.array([self.h_max, 255, 255], dtype=int)                # Upper bound

        # Create mask to filter only yellow regions
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # หาเส้นเฉพาะแถบภาพ roi_top–roi_bottom % (มองล่วงหน้า จะเห็นโค้งก่อนถึง)
        y0 = int(height * self.roi_top / 100)
        y1 = max(y0 + 1, int(height * self.roi_bottom / 100))

        # Detect contours (เฉพาะในแถบ)
        contours, _ = cv2.findContours(mask[y0:y1], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        error = None  # None = ไม่เจอเส้น
        if contours:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])  # X center of mass
                cy = int(M['m01'] / M['m00']) + y0  # Y center of mass (ในภาพเต็ม)

                # Draw the contour and center
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # Compute error from center
                error = cx - width // 2

        # มุมของเส้น (ดูทั้งภาพ): บอกว่าเส้นข้างหน้าหักไปทางไหน (ใช้เตรียมเลี้ยวมุมฉาก)
        angle, blob_x, strips = self.line_angle(mask, frame)

        # เส้นยังแตะขอบล่างของภาพอยู่ไหม (ไม่แตะ = มุมเลื่อนเลยขอบล่างภาพไปแล้ว)
        bottom = mask[int(height * 0.9):]
        at_bottom = cv2.countNonZero(bottom) > 0.03 * bottom.size

        self.error = error
        self.angle = angle
        self.last_image_time = time.monotonic()
        # ถ้าเส้นไม่อยู่ในกรอบฟ้า แต่ยังเห็นที่อื่น ใช้ตำแหน่งของก้อนเส้นทั้งก้อนแทน
        control_error = error if error is not None or blob_x is None else blob_x - width // 2
        self.move_robot(control_error, angle, strips, at_bottom, width)

        # วาดกรอบแถบที่ใช้หาเส้น (สีฟ้า) บนภาพกล้อง และ (สีเทา) บน mask
        cv2.rectangle(frame, (0, y0), (width - 1, y1 - 1), (255, 255, 0), 2)
        # เส้นสีส้ม = "ขอบล่างภาพ" ที่ใช้ตัดสินว่าถึงมุมแล้ว (เส้นต้องไม่เหลือใต้เส้นส้มนี้)
        cv2.line(frame, (0, int(height * 0.9)), (width - 1, int(height * 0.9)), (0, 140, 255), 1)
        cv2.rectangle(mask, (0, y0), (width - 1, y1 - 1), 128, 2)

        # เก็บภาพไว้ให้หน้าต่าง UI แสดง (แทน cv2.imshow ที่เคยเปิดหน้าต่างแยก)
        self.view = frame
        self.mask = mask  # Shows only the yellow parts detected
        self.new_frame = True

    def line_angle(self, mask, frame, n=8):
        # มุมของเส้นเทียบกับแนวตั้ง (องศา): 0 = ตรงขึ้นไป, + = เอียงขวา, - = เอียงซ้าย
        # วิธี: แบ่งภาพเป็น n แถบแนวนอน หาจุดกลางของเส้นในแต่ละแถบ แล้วลากเส้นตรงผ่านจุดเหล่านั้น
        # (ใช้ได้ทั้งตอนกล้องมองไกลและตอนกล้องมองใกล้จนเส้นกินพื้นที่เกือบทั้งภาพ)
        # คืนค่า (มุม, ตำแหน่ง x ของก้อนเส้น, จำนวนแถบที่มีเส้น)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None, 0
        biggest = max(contours, key=cv2.contourArea)
        M = cv2.moments(biggest)
        if M['m00'] < 50:  # ก้อนเล็กเกินไป น่าจะเป็นจุดรบกวน
            return None, None, 0
        blob = np.zeros_like(mask)
        cv2.drawContours(blob, [biggest], -1, 255, -1)  # ใช้เฉพาะก้อนเส้นที่ใหญ่ที่สุด
        h = mask.shape[0]
        ys, xs = [], []
        for i in range(n):
            strip = blob[i * h // n:(i + 1) * h // n]
            m = cv2.moments(strip, binaryImage=True)
            if m['m00'] > 0.03 * strip.size:
                xs.append(m['m10'] / m['m00'])
                ys.append((i + 0.5) * h / n)
        blob_x = M['m10'] / M['m00']
        if len(xs) < 3:  # เส้นสั้นเกินไป (เช่น เส้นที่พาดขวางภาพ) บอกมุมไม่ได้
            return None, blob_x, len(xs)
        slope, intercept = np.polyfit(ys, xs, 1)  # x = slope * y + intercept
        # วาดแนวเส้นที่วัดได้ (สีม่วง)
        cv2.line(frame, (int(slope * ys[0] + intercept), int(ys[0])),
                 (int(slope * ys[-1] + intercept), int(ys[-1])), (255, 0, 255), 2)
        return math.degrees(math.atan(-slope)), blob_x, len(xs)

    def side(self):
        return 'ขวา' if self.corner_dir > 0 else 'ซ้าย'

    def move_robot(self, error, angle, strips, at_bottom, width):
        if not self.running:  # ยังไม่กดเริ่ม หรือกดหยุดไปแล้ว → ไม่ส่งคำสั่ง
            return
        now = time.monotonic()

        # ---------- เลี้ยวมุมฉาก ขั้นที่ 1: เดินตรงไปให้แกนล้อถึงมุม ----------
        # (กล้องมองไม่เห็นพื้นใกล้หุ่น ตอนนี้มุมอยู่ใต้ขอบล่างภาพ จึงเดินตรงตามระยะที่ตั้งไว้)
        if self.mode == 'corner_forward':
            forward_time = self.corner_distance / max(self.linear_speed, 0.01) - SYSTEM_DELAY
            if now - self.mode_start < forward_time:
                self.drive(self.linear_speed, 0.0, f'เดินตรงไปที่มุม ({self.side()})')
                return
            self.mode, self.mode_start = 'corner_turn', now

        # ---------- เลี้ยวมุมฉาก ขั้นที่ 2: หมุนอยู่กับที่ จนเส้นใหม่ตั้งตรง/ครบ 90° ----------
        # (เส้นที่เยื้องไปข้างๆ หรือมุมที่ไม่ใช่ 90° พอดี ให้ช่วงเดินตามเส้นแก้ต่อเอง)
        if self.mode == 'corner_turn':
            # เส้นตั้งตรง และยาวเกินครึ่งภาพ (กันเส้นหนาที่พาดขวางภาพ ซึ่งวัดมุมได้ใกล้ 0 ได้)
            aligned = angle is not None and abs(angle) < 20 and strips >= 5
            turned_90 = now - self.mode_start >= (math.pi / 2) / max(self.angular_speed, 0.05)
            if not aligned and not turned_90:
                self.drive(0.0, -self.corner_dir * self.angular_speed,
                           f'หมุนเข้ามุม ({self.side()})')
                return
            # หมุนเสร็จ → กลับไปเดินตามเส้น
            # ถ้ายังไม่เห็นเส้น ให้หมุนหาต่อไปทางเดียวกับมุม (ไม่เกินเวลาหมุนหา) แทนการหยุดทันที
            self.trend = float(self.corner_dir)
            self.last_seen_time = now
            self.mode, self.corner_dir = 'follow', 0

        # ---------- เดินตามเส้นปกติ ----------
        # มุมที่จำไว้เลื่อนลงไปถึงขอบล่างภาพแล้ว → เริ่มเข้ามุม
        # (เส้นไม่แตะขอบล่าง เหลือแต่เส้นพาดขวาง หรือหายไปจากภาพหมด)
        if self.corner_dir and not at_bottom and (angle is None or abs(angle) > 50):
            self.mode, self.mode_start = 'corner_forward', now
            self.drive(self.linear_speed, 0.0, f'เดินตรงไปที่มุม ({self.side()})')
            return

        if error is None and angle is None:  # ไม่เจอเส้นเลย
            gave_up = self.last_seen_time is None or now - self.last_seen_time > self.search_time
            if gave_up:  # หาจนหมดเวลาแล้ว (หรือไม่เคยเห็นเส้นเลย) → หยุด
                self.stop_robot()
            elif self.trend > 0:  # ช่วงหลังเลี้ยวขวามาตลอด → หมุนขวาหา
                self.drive(0.0, -self.angular_speed, 'หมุนหาเส้น (ขวา)')
            else:  # ช่วงหลังเลี้ยวซ้ายมาตลอด → หมุนซ้ายหา
                self.drive(0.0, self.angular_speed, 'หมุนหาเส้น (ซ้าย)')
            return

        self.last_seen_time = now

        # เตรียมเลี้ยวมุมฉาก: ตอนหุ่นอยู่บนเส้น (เส้นแตะขอบล่างภาพตรงกลาง)
        # ถ้าเส้นเอียงมาก = ข้างหน้ามีมุมหักไปทางนั้น
        # (ถ้าหุ่นเยื้องออกจากเส้น เส้นจะดูเอียงเหมือนกัน แต่ไม่ใช่มุม จึงต้องเช็กว่าเส้นอยู่กลาง)
        centered = error is not None and abs(error) < width * 0.15
        if at_bottom and angle is not None:
            if abs(angle) > 20 and centered:
                self.corner_dir = 1 if angle > 0 else -1
            elif abs(angle) < 10:  # เส้นกลับมาตรงแล้ว ไม่มีมุม
                self.corner_dir = 0
        # รู้แล้วว่าข้างหน้ามีมุม → เดินตรงเข้าหามุม
        # (ไม่เลี้ยวตามเส้นที่หักออกข้าง ไม่งั้นจะเลี้ยวก่อนถึงมุม)
        if self.corner_dir:
            self.drive(self.linear_speed, 0.0, f'เตรียมเข้ามุม ({self.side()})')
            return

        # s = ต้องเลี้ยวแค่ไหน ดูจากตำแหน่งเส้นในกรอบฟ้า (ส่วนที่ใกล้หุ่น)
        #   -1 = เลี้ยวซ้ายเต็มที่, 0 = ตรงไป, +1 = เลี้ยวขวาเต็มที่
        s = max(-1.0, min(1.0, error / (width / 2)))
        self.trend = 0.8 * self.trend + 0.2 * s  # จำแนวโน้มไว้ เผื่อเส้นหายจะได้หมุนหาถูกทาง

        if abs(s) * width / 2 < self.dead_zone:  # แทบไม่ต้องเลี้ยว → เดินตรง
            self.drive(self.linear_speed, 0.0, 'เดินหน้า')
            return
        linear = self.linear_speed * (1 - abs(s))  # ยิ่งต้องเลี้ยวมาก ยิ่งเดินช้าลง
        angular = -self.angular_speed * s          # ยิ่งต้องเลี้ยวมาก ยิ่งเลี้ยวแรง
        self.drive(linear, angular, 'เลี้ยวขวา' if s > 0 else 'เลี้ยวซ้าย')

    def drive(self, linear, angular, action):
        # ส่งความเร็วเดินหน้าและความเร็วเลี้ยวไปพร้อมกัน
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(twist)
        self.action = action

    def move_forward(self, speed):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.action = 'เดินหน้า'

    def move_backward(self, speed):
        twist = Twist()
        twist.linear.x = -speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.action = 'ถอยหลัง'

    def turn_left(self, speed):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = speed
        self.cmd_vel_pub.publish(twist)
        self.action = 'เลี้ยวซ้าย'

    def turn_right(self, speed):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -speed
        self.cmd_vel_pub.publish(twist)
        self.action = 'เลี้ยวขวา'

    def pivot_left(self, speed):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = speed
        self.cmd_vel_pub.publish(twist)
        self.action = 'หมุนซ้าย'

    def pivot_right(self, speed):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -speed
        self.cmd_vel_pub.publish(twist)
        self.action = 'หมุนขวา'

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.action = 'หยุด'


def to_photo(image, width):
    # แปลงภาพ OpenCV เป็นภาพที่ Tk แสดงได้ โดยย่อ/ขยายให้กว้าง width พิกเซล
    height = image.shape[0] * width // image.shape[1]
    image = cv2.resize(image, (width, height))
    if image.ndim == 2:  # ภาพขาวดำ (mask) → รูปแบบ PGM
        header = f'P5 {width} {height} 255 '
    else:  # ภาพสี BGR → RGB รูปแบบ PPM
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        header = f'P6 {width} {height} 255 '
    return tk.PhotoImage(data=header.encode() + image.tobytes(), format='PPM')


class ControlPanel:
    # ช่องกรอกค่า: (ตัวแปรใน node, ข้อความ, ต่ำสุด, สูงสุด, กดลูกศรเพิ่มทีละ, รูปแบบตัวเลข)
    # ค่าสูงสุดมีไว้กันพิมพ์ผิด เช่น ตั้งใจพิมพ์ 0.5 แต่พิมพ์เป็น 5
    FIELDS = [
        ('linear_speed', 'ความเร็วเดินหน้า (m/s)', 0.0, 1.0, 0.01, '%.2f'),
        ('angular_speed', 'ความเร็วเลี้ยวสูงสุด (rad/s)', 0.0, 3.0, 0.05, '%.2f'),
        ('corner_distance', 'มุมฉาก: เดินต่อก่อนหมุน (m)', 0.0, 2.0, 0.05, '%.2f'),
        ('dead_zone', 'เดินตรงเมื่อ |error| < (px)', 1, 1000, 1, '%.0f'),
        ('search_time', 'เส้นหาย: หมุนหานานสุด (วินาที)', 0, 30, 0.5, '%.1f'),
    ]
    # แถบภาพที่ใช้หาเส้น (อยู่ใต้ภาพกล้อง เห็นเป็นกรอบสีฟ้าในภาพ)
    ROI_FIELDS = [
        ('roi_top', 'ขอบบน (%)', 0, 95, 5, '%.0f'),
        ('roi_bottom', 'ขอบล่าง (%)', 5, 100, 5, '%.0f'),
    ]
    # ช่วงสีของเส้นแบบ HSV (อยู่ใต้ภาพ mask ปรับแล้วเห็นผลใน mask ทันที)
    COLOR_FIELDS = [
        ('h_min', 'H ต่ำสุด', 0, 179, 1, '%.0f'),
        ('h_max', 'H สูงสุด', 0, 179, 1, '%.0f'),
        ('s_min', 'S ต่ำสุด', 0, 255, 5, '%.0f'),
        ('v_min', 'V ต่ำสุด', 0, 255, 5, '%.0f'),
    ]
    HINT = 'Enter = ใช้ค่าใหม่  ·  Esc = หยุด'
    GREEN = '#2e7d32'
    RED = '#c62828'
    ORANGE = '#b85c00'

    def __init__(self, node):
        self.node = node

        # ROS กับหน้าต่างทำงานใน thread เดียวกัน: ให้ Tk เรียก spin_once ทุก 10 ms
        # ไม่ต้องแยก thread จึงไม่มีปัญหาสองฝั่งแย่งกันแก้ค่า
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(node)

        self.root = tk.Tk()
        self.root.title('หุ่นยนต์เดินตามเส้น')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.root.bind('<Escape>', lambda event: node.stop())
        for name in ('TkDefaultFont', 'TkTextFont'):
            font.nametofont(name).configure(size=11)

        # ซ้าย = แผงควบคุม, ขวา = ภาพจากกล้อง + mask
        panel = tk.Frame(self.root, padx=14, pady=12)
        panel.grid(row=0, column=0, sticky='n')

        # ป้ายสถานะตัวใหญ่ (เขียว = กำลังวิ่ง, แดง = หยุด)
        self.state_label = tk.Label(panel, font=('Helvetica', 14, 'bold'), fg='white', pady=6)
        self.state_label.grid(row=0, column=0, columnspan=2, sticky='ew', pady=(0, 10))

        # ช่องกรอกค่า: กดลูกศร = ใช้ค่าทันที, พิมพ์เอง = กด Enter
        self.boxes = []
        for row, field in enumerate(self.FIELDS, start=1):
            self.add_field(panel, field, row=row, column=0)
        n = len(self.FIELDS)  # แถวถัดจากช่องกรอกค่า

        # ปุ่มเริ่ม / หยุด
        buttons = tk.Frame(panel)
        buttons.grid(row=n + 1, column=0, columnspan=2, sticky='ew', pady=(12, 4))
        self.start_button = self.make_button(buttons, '▶  เริ่ม', self.GREEN, self.on_start)
        self.stop_button = self.make_button(buttons, '■  หยุด', self.RED, node.stop)

        # บรรทัดคำแนะนำ / แจ้งเมื่อกรอกค่าผิด
        self.message = tk.Label(panel, text=self.HINT, fg='gray35')
        self.message.grid(row=n + 2, column=0, columnspan=2)

        # รายละเอียด: เจอเส้นไหม และกำลังสั่งหุ่นทำอะไร
        ttk.Separator(panel).grid(row=n + 3, column=0, columnspan=2, sticky='ew', pady=8)
        self.line_label = tk.Label(panel)
        self.line_label.grid(row=n + 4, column=0, columnspan=2, sticky='w')
        self.action_label = tk.Label(panel)
        self.action_label.grid(row=n + 5, column=0, columnspan=2, sticky='w')

        # ช่องแสดงภาพ (ก่อนภาพแรกมาจะเป็นสีดำ)
        self.blank = tk.PhotoImage(width=VIEW_WIDTH, height=VIEW_WIDTH * 3 // 4)
        self.camera_view = self.make_view('ภาพจากกล้อง (จุดเขียว = กลางเส้น)', column=1)
        self.mask_view = self.make_view('Mask สีเหลือง (สีขาว = เจอสีเหลือง)', column=2)

        # ช่องปรับแถบภาพที่ใช้หาเส้น ใต้ภาพกล้อง
        roi = tk.Frame(self.camera_view.master)
        roi.pack(anchor='w', pady=(6, 0))
        for i, field in enumerate(self.ROI_FIELDS):
            self.add_field(roi, field, row=0, column=i * 2)
        tk.Label(self.camera_view.master, text='กรอบฟ้า = แถบภาพที่ใช้หาเส้น (% จากขอบบน)',
                 fg='gray35').pack(anchor='w')

        # ช่องปรับช่วงสีของเส้น ใต้ภาพ mask (เรียง 2 x 2)
        colors = tk.Frame(self.mask_view.master)
        colors.pack(anchor='w', pady=(6, 0))
        for i, field in enumerate(self.COLOR_FIELDS):
            self.add_field(colors, field, row=i // 2, column=i % 2 * 2)
        tk.Label(self.mask_view.master, text='H = สี (0–179)   S = ความสด   V = ความสว่าง',
                 fg='gray35').pack(anchor='w')

    def add_field(self, parent, field, row, column):
        # สร้างข้อความ + ช่องกรอก 1 คู่ แล้วเก็บไว้ใน self.boxes ให้ apply_values อ่าน
        attr, text, low, high, step, fmt = field
        label = tk.Label(parent, text=text)
        label.grid(row=row, column=column, sticky='w', padx=(0 if column == 0 else 16, 0), pady=3)
        box = ttk.Spinbox(parent, from_=low, to=high, increment=step, format=fmt,
                          width=6, command=self.apply_values)
        box.set(fmt % getattr(self.node, attr))
        box.bind('<Return>', self.apply_values)
        box.bind('<FocusOut>', self.apply_values)
        box.grid(row=row, column=column + 1, sticky='e', padx=(12, 0), pady=3)
        self.boxes.append((box, attr, text, low, high))

    def make_view(self, title, column):
        box = tk.Frame(self.root, pady=12)
        box.grid(row=0, column=column, sticky='n', padx=(0, 14))
        tk.Label(box, text=title).pack(anchor='w')
        view = tk.Label(box, image=self.blank, bg='black', borderwidth=0)
        view.pack()
        return view

    def make_button(self, parent, text, color, command):
        button = tk.Button(parent, text=text, command=command, font=('Helvetica', 12, 'bold'),
                           bg=color, fg='white', activebackground=color,
                           activeforeground='white', width=8, pady=4)
        button.pack(side='left', expand=True, fill='x', padx=3)
        return button

    def on_start(self):
        if self.apply_values():
            self.node.start()

    def apply_values(self, event=None):
        # อ่านค่าจากทุกช่อง ช่องไหนถูกต้องก็ส่งให้ node ใช้ทันที (คืน True ถ้าถูกครบทุกช่อง)
        problems = []
        for box, attr, text, low, high in self.boxes:
            try:
                value = float(box.get())
            except ValueError:
                value = None
            if value is None or not low <= value <= high:
                problems.append(f'{text}: ใส่ได้ {low}–{high}')
            else:
                setattr(self.node, attr, value)

        if problems:
            self.message.config(text=problems[0], fg=self.RED)
        else:
            self.message.config(text=self.HINT, fg='gray35')
        return not problems

    def refresh_status(self):
        node = self.node
        if node.running:
            self.state_label.config(text='กำลังวิ่ง', bg=self.GREEN)
        else:
            self.state_label.config(text='หยุดอยู่', bg=self.RED)

        if not node.camera_ok():
            line, color = 'กล้อง: ไม่มีภาพจาก /image_raw', self.ORANGE
        elif node.error is None and node.angle is None:
            line, color = 'เส้น: ไม่เจอ', self.ORANGE
        else:
            err = 'นอกกรอบ' if node.error is None else f'{node.error:+d} px'
            ang = '-' if node.angle is None else f'{node.angle:+.0f}°'
            line, color = f'เส้น: error {err}   มุม {ang}', 'black'
        self.line_label.config(text=line, fg=color)
        self.action_label.config(text='คำสั่ง: ' + (node.action if node.running else '-'))

        self.root.after(100, self.refresh_status)  # อัปเดตทุก 0.1 วินาที

    def spin_ros(self):
        # ให้ ROS ทำงาน 1 รอบ (รับภาพ/ส่งคำสั่ง) แล้วนัดตัวเองใหม่ในอีก 10 ms
        try:
            self.executor.spin_once(timeout_sec=0)
            self.show_camera()
        except Exception as e:  # error ในภาพเฟรมเดียว ไม่ควรทำให้ ROS หยุดทำงานทั้งหมด
            self.node.get_logger().error(f'Callback error: {e}')
        self.root.after(10, self.spin_ros)

    def show_camera(self):
        # มีภาพใหม่จาก node → แสดงในหน้าต่าง
        if not self.node.new_frame:
            return
        self.node.new_frame = False
        # ต้องเก็บภาพไว้ใน self ด้วย ไม่งั้น Python จะลบภาพทิ้ง แล้วช่องภาพจะว่าง
        self.camera_photo = to_photo(self.node.view, VIEW_WIDTH)
        self.mask_photo = to_photo(self.node.mask, VIEW_WIDTH)
        self.camera_view.config(image=self.camera_photo)
        self.mask_view.config(image=self.mask_photo)

    def run(self):
        # กด Ctrl+C ใน terminal → ปิดหน้าต่างอย่างเรียบร้อย
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *args: self.root.quit())
        self.refresh_status()
        self.spin_ros()
        try:
            self.root.mainloop()
        finally:
            self.root.destroy()
            self.executor.shutdown()


def main():
    rclpy.init()
    node = LineFollower()
    try:
        ControlPanel(node).run()
    finally:
        node.stop()      # ปิดโปรแกรมด้วยวิธีไหนก็ตาม สั่งหุ่นหยุดก่อนเสมอ
        time.sleep(0.2)  # รอให้คำสั่งหยุดถูกส่งออกไปก่อนปิด node
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
