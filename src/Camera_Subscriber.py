#!/usr/bin/env python
# python node needs chmod +x Camera_Publisher.py
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType


def callback_img(img):
    bridge = CvBridge()
    vid = bridge.imgmsg_to_cv2(img)  # Convert the message to a new image

    rospy.init_node('image_pub_node1', anonymous=True)

    bridge = CvBridge()
    rospy.loginfo("Camera Basic Node Started, now publishing messages")

    while not rospy.is_shutdown():
        pub.publish(bridge.cv2_to_imgmsg(vid, "bgr8"))
    vid.release()
    cv2.destroyAllWindows()

'''
def callback(data):
    rospy.loginfo("RECEIVED DATA: 1")
'''
def listener():
    rospy.init_node("Subscriber_Node", anonymous=True)
    sub = rospy.Subscriber("image_pub_topic", Image, callback_img)
    rospy.spin()

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/Object_image_topic', Image, queue_size=1)
        listener()
    except rospy.ROSInterruptException:
        pass
