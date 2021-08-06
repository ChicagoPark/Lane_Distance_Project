#!/usr/bin/env python
# python node needs chmod +x Camera_Publisher.py
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType

def image_pub():

    vid = cv2.VideoCapture('/home/kaai/chicago_ws/src/first_pkg/src/Pytorch_Lane_Detection/video_input/Cutted_Road_Data.mp4')
    # vid = cv2.VideoCapture('video_input/test1_crowd.avi')
    rval, frame = vid.read()
    img_w, img_h = frame.shape[1], frame.shape[0]

    vid.set(cv2.CAP_PROP_FRAME_WIDTH, img_w)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, img_h)
    vid.set(cv2.CAP_PROP_FPS, 1)



    rospy.loginfo("Camera Basic Node Started, now publishing messages")

    while not rospy.is_shutdown():
        bridge = CvBridge()
        rval, frame = vid.read()
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        rate.sleep()
    vid.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rospy.init_node('video_pub_node', anonymous=True)
        pub = rospy.Publisher('video_pub_topic', Image, queue_size=1)
        rate = rospy.Rate(20)
        image_pub()
    except rospy.ROSInterruptException:
        pass
