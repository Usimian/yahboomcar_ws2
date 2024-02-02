#!/usr/bin/env python
# encoding: utf-8

# public lib
# import os
import time
import getpass

# import threading
# from time import sleep

# ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool


class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)
        self.Joy_active = False     # True enables joystick
        self.Drive_active = False   # True enables drive globally
        self.Buzzer_active = False
        self.RGBLight_index = 0
        self.cancel_time = time.time()
        self.user_name = getpass.getuser()
        self.linear_Gear = 1
        self.angular_Gear = 1
        self.prev_button_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # create pub
        self.pub_goal = self.create_publisher(GoalID, "move_base/cancel", 10)
        self.pub_cmdVel = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_Buzzer = self.create_publisher(Bool, "Buzzer", 1)
        self.pub_JoyState = self.create_publisher(Bool, "JoyState", 10)
        self.pub_RGBLight = self.create_publisher(Int32, "RGBLight", 10)

        # create sub
        self.sub_Joy = self.create_subscription(Joy, "joy", self.buttonCallback, 10)

        # declare parameter and get the value
        self.declare_parameter("xspeed_limit", 1.0)
        self.declare_parameter("yspeed_limit", 1.0)
        self.declare_parameter("angular_speed_limit", 5.0)
        self.xspeed_limit = self.get_parameter("xspeed_limit").get_parameter_value().double_value
        self.yspeed_limit = self.get_parameter("yspeed_limit").get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy):
            return
        if self.user_name == "root":
            self.user_jetson(joy_data)
        else:
            self.user_pc(joy_data)

    def user_jetson(self, joy_data):
        # Toggle joystick control on/off
        if joy_data.buttons[9] == 1:
            if self.prev_button_state[9] == 0:  # Button pressed
                self.Joy_active = not self.Joy_active   # Toggle movement enable
                if self.Joy_active is True:
                    print("Joystick ON")
                else:
                    print("Joystick OFF")
                    self.pub_cmdVel.publish(Twist())    # Stop moving
                self.prev_button_state[9] = 1
        else:
            self.prev_button_state[9] = 0

        # Toggle drive on/off (Y Button)
        if joy_data.buttons[4] == 1:
            if self.prev_button_state[4] == 0:  # Button pressed
                self.Drive_active = not self.Drive_active   # Toggle movement enable
                Joy_ctrl = Bool()
                Joy_ctrl.data = self.Drive_active
                if self.Drive_active is True:
                    print("Drive ON")
                else:
                    print("Drive OFF")
                    self.pub_cmdVel.publish(Twist())    # Stop moving
                self.pub_JoyState.publish(Joy_ctrl)
                # self.pub_goal.publish(GoalID())
                self.prev_button_state[4] = 1
        else:
            self.prev_button_state[4] = 0

        # RGBLight
        if joy_data.buttons[7] == 1:  # Light bar sequence button pressed (RB)
            RGBLight_ctrl = Int32()
            if self.prev_button_state[7] == 0:  # Button pressed
                if self.RGBLight_index < 7:
                    RGBLight_ctrl.data = self.RGBLight_index
                    self.pub_RGBLight.publish(RGBLight_ctrl)
                    self.RGBLight_index += 1
                    self.prev_button_state[7] = 1
                else:
                    self.RGBLight_index = 0
        else:
            self.prev_button_state[7] = 0  # Toggle back to unpressed

        # Buzzer (start button)
        if joy_data.buttons[11] == 1:
            if self.prev_button_state[11] == 0:  # Button pressed
                Buzzer_ctrl = Bool()
                self.Buzzer_active = not self.Buzzer_active
                Buzzer_ctrl.data = self.Buzzer_active
                self.pub_Buzzer.publish(Buzzer_ctrl)
                print(self.Buzzer_active)
                self.prev_button_state[11] = 1
        else:
            self.prev_button_state[11] = 0

        # XY movement speed (LJ press)
        if joy_data.buttons[13] == 1:
            if self.prev_button_state[13] == 0:  # Button pressed
                if self.linear_Gear == 1.0:
                    self.linear_Gear = 1.0 / 4
                elif self.linear_Gear == 1.0 / 4:
                    self.linear_Gear = 2.0 / 4
                elif self.linear_Gear == 2.0 / 4:
                    self.linear_Gear = 3.0 / 4
                elif self.linear_Gear == 3.0 / 4:
                    self.linear_Gear = 1
                print(f"Linear speed factor = x{self.linear_Gear}")
                self.prev_button_state[13] = 1
        else:
            self.prev_button_state[13] = 0

        # Turn speed (RJ press)
        if joy_data.buttons[14] == 1:
            if self.prev_button_state[14] == 0:  # Button pressed
                if self.angular_Gear == 1.0:
                    self.angular_Gear = 1.0 / 4
                elif self.angular_Gear == 1.0 / 4:
                    self.angular_Gear = 1.0 / 2
                elif self.angular_Gear == 1.0 / 2:
                    self.angular_Gear = 3.0 / 4
                elif self.angular_Gear == 3.0 / 4:
                    self.angular_Gear = 1.0
                print(f"Rotate speed factor = x{self.angular_Gear}")
                self.prev_button_state[14] = 1
        else:
            self.prev_button_state[14] = 0

        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        # ylinear_speed = self.filter_data(joy_data.axes[2]) * self.yspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear

        if xlinear_speed > self.xspeed_limit:
            xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit:
            xlinear_speed = -self.xspeed_limit

        if ylinear_speed > self.yspeed_limit:
            ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit:
            ylinear_speed = -self.yspeed_limit

        if angular_speed > self.angular_speed_limit:
            angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit:
            angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        if self.Joy_active is True:
            self.pub_cmdVel.publish(twist)

    def user_pc(self, joy_data):
        # 取消 Cancel
        if joy_data.axes[5] == -1:
            self.cancel_nav()
        if joy_data.buttons[5] == 1:
            if self.RGBLight_index < 6:
                self.pub_RGBLight.publish(self.RGBLight_index)
                # print ("pub RGBLight success")
            else:
                self.RGBLight_index = 0
            self.RGBLight_index += 1
        if joy_data.buttons[7] == 1:
            self.Buzzer_active = not self.Buzzer_active
            # print "self.Buzzer_active: ", self.Buzzer_active
            self.pub_Buzzer.publish(self.Buzzer_active)
        # Gear control
        if joy_data.buttons[9] == 1:
            if self.linear_Gear == 1.0:
                self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3:
                self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3:
                self.linear_Gear = 1
        if joy_data.buttons[10] == 1:
            if self.angular_Gear == 1.0:
                self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4:
                self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2:
                self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4:
                self.angular_Gear = 1.0
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[2]) * self.angular_speed_limit * self.angular_Gear
        if xlinear_speed > self.xspeed_limit:
            xlinear_speed = self.xspeed_limit
        elif xlinear_speed < -self.xspeed_limit:
            xlinear_speed = -self.xspeed_limit
        if ylinear_speed > self.yspeed_limit:
            ylinear_speed = self.yspeed_limit
        elif ylinear_speed < -self.yspeed_limit:
            ylinear_speed = -self.yspeed_limit
        if angular_speed > self.angular_speed_limit:
            angular_speed = self.angular_speed_limit
        elif angular_speed < -self.angular_speed_limit:
            angular_speed = -self.angular_speed_limit
        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        self.pub_cmdVel.publish(twist)

    def filter_data(self, value):
        if abs(value) < 0.2:
            value = 0
        return value

    def cancel_nav(self):
        Joy_ctrl = Bool()
        Joy_ctrl.data = self.Joy_active
        if self.Joy_active is True:
            print("Joystick drive ON")
        else:
            print("Joystick Drive OFF")
            self.pub_cmdVel.publish(Twist())    # Stop moving
        self.pub_JoyState.publish(Joy_ctrl)
        # self.pub_goal.publish(GoalID())


def main():
    rclpy.init()
    joy_ctrl = JoyTeleop("joy_ctrl")
    rclpy.spin(joy_ctrl)
