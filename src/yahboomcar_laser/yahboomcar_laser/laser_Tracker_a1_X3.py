import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from yahboomcar_laser.common import SinglePID
from yahboomcar_msgs.msg import JoyControl

import math
import numpy as np

RAD2DEG = 180 / math.pi


class laserTracker(Node):
    def __init__(self, name):
        super().__init__(name)
        # create a sub
        self.sub_laser = self.create_subscription(LaserScan, "/scan", self.registerScan, 1)
        self.sub_JoyState = self.create_subscription(JoyControl, "/JoyControl", self.JoyControlCallback, 1)
        # create a pub
        self.pub_vel = self.create_publisher(Twist, "/cmd_vel", 1)

        # declareparam
        self.declare_parameter("linear", 0.5)
        self.linear = self.get_parameter("linear").get_parameter_value().double_value
        self.declare_parameter("angular", 1.0)
        self.angular = self.get_parameter("angular").get_parameter_value().double_value
        self.declare_parameter("LaserAngle", 40.0)
        self.LaserAngle = self.get_parameter("LaserAngle").get_parameter_value().double_value
        self.declare_parameter("ResponseDist", 0.55)
        self.ResponseDist = self.get_parameter("ResponseDist").get_parameter_value().double_value
        self.declare_parameter("Switch", False)
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value

        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.ros_ctrl = SinglePID()
        self.priorityAngle = 30  # 40
        self.Moving = False
        self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        self.joy_control = JoyControl()

        self.timer = self.create_timer(0.01, self.on_timer)

    def on_timer(self):
        self.Switch = self.get_parameter("Switch").get_parameter_value().bool_value
        self.angular = self.get_parameter("angular").get_parameter_value().double_value
        self.linear = self.get_parameter("linear").get_parameter_value().double_value
        self.LaserAngle = self.get_parameter("LaserAngle").get_parameter_value().double_value
        self.ResponseDist = self.get_parameter("ResponseDist").get_parameter_value().double_value

    def JoyControlCallback(self, msg):
        if not isinstance(msg, JoyControl):
            return
        self.joy_control = msg
        print(msg)

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan):
            return
        ranges = np.array(scan_data.ranges)
        offset = 0.5
        frontDistList = []
        frontDistIDList = []
        minDistList = []
        minDistIDList = []

        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if abs(angle) > (180 - self.priorityAngle):
                if ranges[i] < (self.ResponseDist + offset):
                    frontDistList.append(ranges[i])
                    frontDistIDList.append(angle)
            elif (180 - self.LaserAngle) < angle < (180 - self.priorityAngle):
                minDistList.append(ranges[i])
                minDistIDList.append(angle)
            elif (self.priorityAngle - 180) < angle < (self.LaserAngle - 180):
                minDistList.append(ranges[i])
                minDistIDList.append(angle)
        if len(frontDistIDList) != 0:
            minDist = min(frontDistList)
            minDistID = frontDistIDList[frontDistList.index(minDist)]
        else:
            minDist = min(minDistList)
            minDistID = minDistIDList[minDistList.index(minDist)]

        velocity = Twist()
        if abs(minDist - self.ResponseDist) < 0.1:
            minDist = self.ResponseDist
        velocity.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, minDist)
        ang_pid_compute = self.ang_pid.pid_compute((180 - abs(minDistID)) / 72, 0)
        if minDistID > 0:
            velocity.angular.z = -ang_pid_compute
        else:
            velocity.angular.z = ang_pid_compute
        if ang_pid_compute < 0.02:
            velocity.angular.z = 0.0

        if self.joy_control.driveactive is True:
            self.pub_vel.publish(velocity)


def main():
    rclpy.init()
    laser_tracker = laserTracker("laser_Tracker_a1")
    print("start it")
    try:
        rclpy.spin(laser_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        laser_tracker.pub_vel.publish(Twist())
        laser_tracker.destroy_node()
        rclpy.shutdown()
