#! /usr/bin/env python3

import cv2
import rospy
from std_msgs.msg import Float32

rospy.init_node('cap', anonymous=True)

cap = cv2.VideoCapture('rtsp://192.168.100.12:8553/eo')

while not rospy.is_shutdown():
    ret, frame = cap.read()
    cv2.namedWindow("Camera", cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow("Camera", 960, 540)
    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()

cv2.destroyAllWindows()
