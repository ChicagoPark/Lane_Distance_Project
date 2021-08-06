#!/usr/bin/env python
from models import *
from utils1 import *

import numpy as np
import rospy
import math
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType
import torch, os, cv2
#from model.model import parsingNet

import scipy.special, tqdm
import numpy as np

from PIL import Image

from sensor_msgs.msg import Image

import sys, time, datetime, random
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from torch.autograd import Variable

# load weights and set defaults
config_path='/home/kaai/chicago_ws/src/first_pkg/src/pytorch_objectdetecttrack/config/yolov3.cfg'
weights_path='/home/kaai/chicago_ws/src/first_pkg/src/pytorch_objectdetecttrack/config/yolov3.weights'
class_path='/home/kaai/chicago_ws/src/first_pkg/src/pytorch_objectdetecttrack/config/coco.names'
img_size=416
conf_thres=0.8
nms_thres=0.4


# load model and put into eval mode
model = Darknet(config_path, img_size=img_size)
model.load_weights(weights_path)
model.cuda()
model.eval()

classes = utils.load_classes(class_path)
Tensor = torch.cuda.FloatTensor

import cv2
from sort import *
colors=[(255,0,0),(0,255,0),(0,0,255),(255,0,255),(128,0,0),(0,128,0),(0,0,128),(128,0,128),(128,128,0),(0,128,128)]

def detect_image(img):
    # scale and pad image
    ratio = min(img_size/img.size[0], img_size/img.size[1])
    imw = round(img.size[0] * ratio)
    imh = round(img.size[1] * ratio)
    img_transforms = transforms.Compose([ transforms.Resize((imh, imw)),
         transforms.Pad((max(int((imh-imw)/2),0), max(int((imw-imh)/2),0), max(int((imh-imw)/2),0), max(int((imw-imh)/2),0)),
                        (128,128,128)),
         transforms.ToTensor(),
         ])
    # convert image to Tensor
    image_tensor = img_transforms(img).float()
    image_tensor = image_tensor.unsqueeze_(0)
    input_img = Variable(image_tensor.type(Tensor))
    # run inference on the model and get detections
    with torch.no_grad():
        detections = model(input_img)
        detections = utils.non_max_suppression(detections, 80, conf_thres, nms_thres)
    return detections[0]

def callback_img(img):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(img)
   
    #vid = cv2.VideoCapture(3)
    

    cv2.namedWindow('Stream',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Stream', (800,600))

    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #ret,frame=vid.read()


    vw = frame.shape[1]
    vh = frame.shape[0]
    print ("Video size", vw,vh)
    #outvideo = cv2.VideoWriter(videopath.replace(".mp4", "-det.mp4"),fourcc,20.0,(vw,vh))
    
    rospy.loginfo("2222222222222")


    frames = 0
    starttime = time.time()
    #while(True):
    #ret, frame = vid.read()
    #if not ret:
        #break
    frames += 1
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    from PIL import Image
    pilimg = Image.fromarray(frame)
    from sensor_msgs.msg import Image
    detections = detect_image(pilimg)

    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    img = np.array(pilimg)
    pad_x = max(img.shape[0] - img.shape[1], 0) * (img_size / max(img.shape))
    pad_y = max(img.shape[1] - img.shape[0], 0) * (img_size / max(img.shape))
    unpad_h = img_size - pad_y
    unpad_w = img_size - pad_x
    if detections is not None:
        tracked_objects = mot_tracker.update(detections.cpu())

        unique_labels = detections[:, -1].cpu().unique()
        n_cls_preds = len(unique_labels)
        for x1, y1, x2, y2, obj_id, cls_pred in tracked_objects:
            box_h = int(((y2 - y1) / unpad_h) * img.shape[0])
            box_w = int(((x2 - x1) / unpad_w) * img.shape[1])
            y1 = int(((y1 - pad_y // 2) / unpad_h) * img.shape[0])
            x1 = int(((x1 - pad_x // 2) / unpad_w) * img.shape[1])
            color = colors[int(obj_id) % len(colors)]
            cls = classes[int(cls_pred)]
            cv2.rectangle(frame, (x1, y1), (x1+box_w, y1+box_h), color, 4)
            cv2.rectangle(frame, (x1, y1-35), (x1+len(cls)*19+80, y1), color, -1)
            cv2.putText(frame, cls + "-" + str(int(obj_id)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
    
    cv2.imshow('Stream', frame)
    #outvideo.write(frame)
    ch = 0xFF & cv2.waitKey(1)
    #if ch == 27:
     #   break
    pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    totaltime = time.time()-starttime
    print(frames, "frames", totaltime/frames, "s/frame")
    #cv2.destroyAllWindows()
    #outvideo.release()
def listener():
    sub = rospy.Subscriber('image_pub_topic', Image, callback_img)
    rospy.loginfo("Object Node Started, now publishing messages")

    rospy.spin()


if __name__ == "__main__":  ### initialize ros node
    try:
        mot_tracker = Sort() 
        # load model
        rospy.loginfo("Object Node Started, now publishing messages")
        rospy.init_node('Object_Dect_Node', anonymous=True)

        pub = rospy.Publisher('Object_image_topic', Image, queue_size=1)
        listener()
    except rospy.ROSInterruptException:
        pass
