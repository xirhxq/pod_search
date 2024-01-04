#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int16
from nav_msgs.msg import Odometry
import numpy as np
from rich.console import Console

class FakeSuav:
    def __init__(self):
        self.console = Console()
        self.x, self.y, self.z, self.yawDeg, self.id = 0.0, 0.0, 0.0, 0.0, -1
        self.xyzTol, self.yawDegTol = 1, 1
        self.xyzKp, self.yawKp = 0.2, 1
        self.xExpected, self.yExpected, self.zExpected, self.yawDegExpected = 0.0, 0.0, 0.0, 0.0
        self.xyzSaturation, self.yawDegSaturation = 5, 10
        self.search_point_subscriber = rospy.Subscriber("/suav/searchPoint", Float64MultiArray, self.search_point_callback)
        self.odom_publisher = rospy.Publisher("/suav/uwb/filter/odom", Odometry, queue_size=10)
        self.flight_data_publisher = rospy.Publisher("/suav/xy_fcu/flight_data", Float32MultiArray, queue_size=10)
        self.uav_state_publisher = rospy.Publisher("/suav/uavState", Int16, queue_size=10)

    def search_point_callback(self, data):
        new_id = data.data[0]
        if new_id != self.id:
            self.xExpected, self.yExpected, self.zExpected, self.yawDegExpected = data.data[1:5]
            self.id = new_id

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            x_change = np.clip((self.xExpected - self.x) * self.xyzKp, -self.xyzSaturation, self.xyzSaturation)
            y_change = np.clip((self.yExpected - self.y) * self.xyzKp, -self.xyzSaturation, self.xyzSaturation)
            z_change = np.clip((self.zExpected - self.z) * self.xyzKp, -self.xyzSaturation, self.xyzSaturation)
            yaw_change = np.clip((self.yawDegExpected - self.yawDeg) * self.yawKp, -self.yawDegSaturation, self.yawDegSaturation)

            self.x += x_change
            self.y += y_change
            self.z += z_change
            self.yawDeg += yaw_change

            odom_msg = Odometry()
            odom_msg.twist.twist.linear.x = self.x
            odom_msg.twist.twist.linear.y = self.y
            odom_msg.twist.twist.linear.z = self.z
            self.odom_publisher.publish(odom_msg)

            flight_data_msg = Float32MultiArray()
            flight_data_msg.data = [0.0]*20
            flight_data_msg.data[15] = np.float32(np.radians(np.mod(90 - self.yawDeg, 360)))
            self.flight_data_publisher.publish(flight_data_msg)

            if abs(self.xExpected - self.x) <= self.xyzTol and abs(self.yawDegExpected - self.yawDeg) <= self.yawDegTol:
                state_msg = Int16(int(self.id * 100 + 1))
            else:
                state_msg = Int16(int(self.id * 100))

            self.uav_state_publisher.publish(state_msg)
            self.console.clear()
            self.console.rule("[bold cyan]FakeSuav Status Update")
            self.console.print(f"x: {self.x:.2f}, y: {self.y:.2f}, z: {self.z:.2f}, yawDeg: {self.yawDeg:.2f}, id: {self.id}")
            self.console.print(f"xExpected: {self.xExpected:.2f}, yExpected: {self.yExpected:.2f}, zExpected: {self.zExpected:.2f}, yawDegExpected: {self.yawDegExpected:.2f}")
            self.console.print(f"xyzTol: {self.xyzTol}, yawDegTol: {self.yawDegTol}, xyzKp: {self.xyzKp}, yawKp: {self.yawKp}")
            if self.id == 6:
                exit(0)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('fake_suav')
    fake_suav = FakeSuav()
    fake_suav.spin()
