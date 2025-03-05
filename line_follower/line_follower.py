import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import threading
import sys
import select

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')

        # Subscribers & Publishers
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        self.emergency_stop = False

        # self.max_angular_speed = 0.3  # ค่าการหมุนสูงสุด

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        height, width, _ = frame.shape

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV range for yellow color
        lower_yellow = np.array([20, 100, 100])   # Lower bound (adjust if needed)
        upper_yellow = np.array([40, 255, 255])   # Upper bound

        # Create mask to filter only yellow regions
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Detect contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])  # X center of mass
                cy = int(M['m01'] / M['m00'])  # Y center of mass

                # Draw the contour and center
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # Compute error from center
                error = cx - width // 2
                self.move_robot(error)

        # Show the processed frame (for debugging)
        cv2.imshow("Yellow Line Mask", mask)  # Shows only the yellow parts detected
        cv2.imshow("Line Following", frame)
        cv2.waitKey(1)

    def move_robot(self, error):
        if self.emergency_stop:
            twist = Twist()  # ค่าทั้งหมดเป็น 0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("🛑 Emergency Stop Activated!")
            return  # ออกจากฟังก์ชันทันที
        
        print(error)
        if (error > -50) and (error < 50):
            self.move_forward(0.1)
        else:
            self.stop_robot()

    def move_forward(self, speed):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print("Robot move foward.")

    def move_backward(self, speed):
        twist = Twist()
        twist.linear.x = -speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print("Robot move backward.")

    def turn_left(self, speed):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = speed
        self.cmd_vel_pub.publish(twist)
        print("Robot turn left.")

    def turn_right(self, speed):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = -speed
        self.cmd_vel_pub.publish(twist)
        print("Robot turn right.")


    def pivot_left(self, speed):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = speed
        self.cmd_vel_pub.publish(twist)
        print("Robot pivot left.")

    def pivot_right(self, speed):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -speed
        self.cmd_vel_pub.publish(twist)
        print("Robot pivot right.")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        print("Robot stopped.")

def emergency_stop_listener(node):
    print("Press 'q' to stop the robot immediately.")
    print("Press 'r' to resume movement.")
    
    while True:
        try:
            key = sys.stdin.readline().strip().lower()  # แก้จาก read(1) เป็น readline() + strip()
            
            if key == 'q':  # กด q เพื่อหยุด
                print("\n🛑 Emergency Stop Activated!")
                node.emergency_stop = True  
                stop_robot(node)
            
            elif key == 'r':  # กด r เพื่อเริ่มใหม่
                print("\n✅ Resuming Movement!")
                node.emergency_stop = False  

        except UnicodeDecodeError as e:
            print(f"Error reading input: {e}")

def stop_robot(node):
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    node.cmd_vel_pub.publish(twist)
    print("Robot stopped.")

def main():
    rclpy.init()
    node = LineFollower()

    stop_thread = threading.Thread(target=emergency_stop_listener, args=(node,), daemon=True)
    stop_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()