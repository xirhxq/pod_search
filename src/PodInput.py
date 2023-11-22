#! /usr/bin/python3

import rospy
from std_msgs.msg import String, Float32

if __name__ == '__main__':
    rospy.init_node('pod_input', anonymous=True)
    p_pub = rospy.Publisher('/suav/pod/expectedPitch', Float32, queue_size=10)
    y_pub = rospy.Publisher('/suav/pod/expectedYaw', Float32, queue_size=10)
    z_pub = rospy.Publisher('/suav/pod/expectedHfov', Float32, queue_size=10)
    mr_pub = rospy.Publisher('/suav/pod/maxRate', Float32, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            py_input_str = input('Enter pitch, yaw, hfov and max rate: ')
        
            p_pub.publish(float(py_input_str.split()[0]))
            y_pub.publish(float(py_input_str.split()[1]))
            z_pub.publish(float(py_input_str.split()[2]))
            mr_pub.publish(float(py_input_str.split()[3]))
        except Exception as e:
            break
        rate.sleep()
