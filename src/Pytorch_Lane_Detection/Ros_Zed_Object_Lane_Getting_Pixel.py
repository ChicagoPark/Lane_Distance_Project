#!/usr/bin/env python

# Lane Module Beginning >>>>>>>>>>>>>------------------>>>>>>>>>>>>>------------------
import numpy as np
import rospy
import math
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType
import torch, os, cv2
from model.model import parsingNet
from utils1.common import merge_config
from utils1.dist_utils import dist_print
import scipy.special, tqdm
import numpy as np
import torchvision.transforms as transforms
from data.dataset import LaneTestDataset
from data.constant import culane_row_anchor, tusimple_row_anchor, youngil_row_anchor
from PIL import Image
from sensor_msgs.msg import Image

# Lane Module Ending --------------------<<<<<<<<<<<<--------------------<<<<<<<<<<<<

# Object Tracking Beginning >>>>>>>>>>>>>------------------>>>>>>>>>>>>>------------------


from models import *
from utils1 import *
import sys, time, datetime, random
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from torch.autograd import Variable
import utils # utils file 을 import 해온다.
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

# Object Tracking Ending --------------------<<<<<<<<<<<<--------------------<<<<<<<<<<<<

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


def callback_img1(img1):
    bridge = CvBridge()
    frame1 = bridge.imgmsg_to_cv2(img1)

    # vid = cv2.VideoCapture(3)

    #cv2.namedWindow('Stream', cv2.WINDOW_NORMAL)
    #cv2.resizeWindow('Stream', (800, 600))

    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # ret,frame=vid.read()

    #frame1 = frame1.convert('RGB')

    vw = frame1.shape[1]
    vh = frame1.shape[0]
    #print("Video size", vw, vh)
    # outvideo = cv2.VideoWriter(videopath.replace(".mp4", "-det.mp4"),fourcc,20.0,(vw,vh))

    #rospy.loginfo("2222222222222")

    frames = 0
    starttime = time.time()
    # while(True):
    # ret, frame = vid.read()
    # if not ret:
    # break
    frames += 1
    frame = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)       # frame 으로 설정하면 lane detection 퍼블리싱에 디텍션 정보가 같이 나온다.
    from PIL import Image
    pilimg = Image.fromarray(frame1)

    # Solving the 4 channel problem when we use zed
    pilimg = pilimg.convert('RGB')

    from sensor_msgs.msg import Image
    detections = detect_image(pilimg)
    #frame = cv2.cvtColor(frame1, cv2.COLOR_RGB2BGR)
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
            cv2.rectangle(frame1, (x1, y1), (x1 + box_w, y1 + box_h), color, 1)
            cv2.rectangle(frame1, (x1, y1 - 25), (x1 + 70, y1), color, -1)
            #cv2.putText(frame1, cls + "-" + str(int(obj_id)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
            cv2.putText(frame1, 'car' + "-" + str(int(obj_id)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            #print(frame1.shape[1])
            if x1+box_w/2 >= 0 and x1+box_w/2 <= frame1.shape[1]*(5/11) and y1+box_h/2 >=0 and y1+box_h/2 <= frame1.shape[0]: # box 의 id 를 담아주는 조건 명시
                objectlist.append((int(x1+box_w/2) , int(y1 + box_h/2), int(obj_id)))

    #cv2.imshow('Stream', frame1)
    # outvideo.write(frame)
    ch = 0xFF & cv2.waitKey(1)
    # if ch == 27:
    #   break
    pub1.publish(bridge.cv2_to_imgmsg(frame1, "8UC4"))
    totaltime = time.time() - starttime
    #print(frames, "frames1", totaltime / frames, "s/frame")
    # cv2.destroyAllWindows()
    # outvideo.release()

def callback_img2(img2):
    #image converting
    bridge = CvBridge()

    frame2 = bridge.imgmsg_to_cv2(img2)  # Convert the message to a new image
    # Solving the 4 channel problem when we use zed

    #vid_fps = int(vid.get(cv2.CAP_PROP_FPS))
    # Getting video size
    #vid_width, vid_height = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)), int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))

    #codec = cv2.VideoWriter_fourcc(*'XVID')  # for avi file output
    # codec = cv2.VideoWriter_fourcc(*'MP4V')     #for MP4 file output

    # size of the width and height has to be same size with output image's size.
    #vout = cv2.VideoWriter('video_output/Youngil_3.mp4', codec, vid_fps, (1640, 590))

    # row_anchor = culane_row_anchor
    row_anchor = youngil_row_anchor
    img_w, img_h = frame2.shape[1], frame2.shape[0]

    #while True:

    # model_input = torch.from_numpy(frame)

    from PIL import Image
    model_input = Image.fromarray(frame2)
    model_input = model_input.convert('RGB')
    from sensor_msgs.msg import Image
    model_input = transforms.Resize((288, 800))(model_input)
    model_input = transforms.ToTensor()(model_input)
    model_input = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])(model_input)

    model_input = model_input.to(device).float()


    with torch.no_grad():
        out = net(model_input[None, ...])

    col_sample = np.linspace(0, 800 - 1, 200)
    col_sample_w = col_sample[1] - col_sample[0]

    # out_j has information about each line from the left
    out_j = out[0].data.cpu().numpy()
    out_j = out_j[:, ::-1, :]
    prob = scipy.special.softmax(out_j[:-1, :, :], axis=0)
    idx = np.arange(200) + 1
    idx = idx.reshape(-1, 1, 1)
    loc = np.sum(prob * idx, axis=0)
    out_j = np.argmax(out_j, axis=0)
    loc[out_j == 200] = 0
    out_j = loc

    # out = out.view(36, 134, 3)
    # out = out.cpu().numpy()

    # c : for draw the line with different color
    # out_j.shape[1] means the number of lane
    num_lane = out_j.shape[1]

    if num_lane <= 2:
        c = 50
        d = 150
    elif num_lane == 3:
        c = 0
        d = 200
        print("y\no\nu\nn\ng\ni\nl")
    else:
        c = 0
        d = 200

    leftx = []
    lefty = []
    rightx = []
    righty = []
    for i in range(num_lane):
        point_list_for_curve = []
        if np.sum(out_j[:, i] != 0) > 2:
            for k in range(out_j.shape[0]):
                if out_j[k, i] > 0:
                    # ppp has (a, b) format. This means each point location !
                    ppp = (int(out_j[k, i] * col_sample_w * img_w / 800) - 1,
                           int(img_h * (row_anchor[cls_num_per_lane - 1 - k] / 288)) - 1)
                    if i == 1:
                        # print(ppp[0], '    ', ppp[1])
                        leftx.append(ppp[0])
                        lefty.append(ppp[1])
                        # print(leftx)
                    if i == 2:
                        # print(ppp[0], '    ', ppp[1])
                        rightx.append(ppp[0])
                        righty.append(ppp[1])
                    cv2.circle(frame2, ppp, 5, (0, c, d), -1)
                    point_list_for_curve.append(ppp)
        point_list_for_curve = np.array(point_list_for_curve, np.int32)
        #cv2.polylines(frame2, [point_list_for_curve], False, (0, c, d), thickness=7)
        c += 50
        d -= 80

        k = []  # plot 을 하기위해서 담는 좌측 레인의 x, y 좌표
        l = []  # plot 을 하기위해서 담는 우측 레인의 x, y 좌표

        # 2차 함수 구하고, 그래프를 그리는 코드
        if len(leftx) != 0 and len(lefty) != 0:
            left_fit = np.polyfit(lefty, leftx, 2)

            # ploty = np.linspace(0, frame.shape[0] - 1, frame.shape[0])
            ploty = np.linspace(int(frame2.shape[0]*(6/11)), frame2.shape[0] - 1,
                                int(frame2.shape[0] * (5 / 11)))  # y좌표에 대해서 상위 1/3 지점 부터 최하단 까지 그래프를 그리기위한 코드
            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]  # 방정식에 대한 x 값을 얻어오기 위한 코드
            for ii, x in enumerate(left_fitx):
                q = (x, ii + int(frame2.shape[0]* (6/11)))  # 상위 1/3 지점부터 그래프를 그리기 시작하므로 인덱스에 해당하는 ii 에 그만큼 추가해준다.
                if (x >= 0):
                    k.append(q)
            k = np.array(k, np.int32)  # polylines 함수에 좌표는 np.array 형태를 유지해줘야하므로 추가한 코드이다.
            cv2.polylines(frame2, [k], False, (235, 206, 135), thickness=7)  # 좌측 레인 : sky blue color
            if len(objectlist) != 0:

                # TO remove overflow
                if len(objectlist) >= 25:
                    objectlist.clear()
                else:
                    objectx = int(objectlist[0][0])
                    objecty = int(objectlist[0][1])
                    objectid = objectlist[0][2]

                    lanex = int(left_fit[0] * objecty ** 2 + left_fit[1] * objecty + left_fit[2])
                    laney = objecty
                    print(f'{objectx}   {objecty}    {lanex}   {laney}')

                    # Plotting between vehicle and lane
                    cv2.line(frame2, (objectx, objecty), (lanex, laney), (0, 0, 255), thickness = 4, lineType=None, shift=None)

                    cv2.rectangle(frame2, (lanex-75, laney), (lanex-45, laney-25), (100, 0, 255), -1)
                    cv2.putText(frame2, str(int(objectid)), (lanex-70, laney-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 255, 255), 1)
                    '''
                    cv2.rectangle(frame1, (x1, y1), (x1 + box_w, y1 + box_h), color, 1)
                    cv2.rectangle(frame1, (x1, y1 - 35), (x1 + len(cls) * 19 + 80, y1), color, -1)
                    cv2.putText(frame1, cls + "-" + str(int(obj_id)), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
                    '''
                    #print(len(objectlist))
                    objectlist.pop(0)
            print(len(objectlist))



        if len(rightx) != 0 and len(righty) != 0:
            ploty = np.linspace(int(frame2.shape[0]*(6/11) ), frame2.shape[0] - 1,
                                int(frame2.shape[0] * (5 / 11)))  # y좌표에 대해서 상위 1/3 지점 부터 최하단 까지 그래프를 그리기위한 코드
            right_fit = np.polyfit(righty, rightx, 2)
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
            for ii, x in enumerate(right_fitx):
                q = (x, ii + int(frame2.shape[0]*(6/11)))
                if (x >= 0):
                    l.append(q)
            l = np.array(l, np.int32)
            cv2.polylines(frame2, [l], False, (160, 114, 0), thickness=7)  # 우측 레인 : deep blue color



    # out = out.cpu().numpy()
    # print(out.shape)
    frame2 = cv2.resize(frame2, (1640, 590), interpolation=cv2.INTER_AREA)
    cv2.imshow('output', frame2)


    pub2.publish(bridge.cv2_to_imgmsg(frame2, "8UC4"))

    key = cv2.waitKey(20)
    #vout.write(frame)  # every frame 에 이미지를 저장한다고 한다.

    '''
    if key == 27:
        break
    '''
#vid.release()
#vout.release()
#cv2.destroyAllWindows()

def listener():
    rospy.loginfo("Object & Lane Node Started, now publishing messages")

    # callback_img1 => Object Tracking
    sub1 = rospy.Subscriber('/zed/zed_node/left/image_rect_color', Image, callback_img1)
    #sub1 = rospy.Subscriber('video_pub_topic', Image, callback_img1)
    # callback_img2 => Lane Detection
    #sub2 = rospy.Subscriber('video_pub_topic', Image, callback_img2)
    sub2 = rospy.Subscriber('/zed/zed_node/left/image_rect_color', Image, callback_img2)

    rospy.spin()


if __name__ == "__main__":  ### initialize ros node
    try:
        objectlist = []
        rospy.init_node('Object_Lane_Merging', anonymous=True)
        mot_tracker = Sort()

        # load model
        rospy.loginfo("Object & Lane Merging Node Started, now publishing messages")


        # load model
        rospy.loginfo("Lane Node Started, now publishing messages")
        #rospy.init_node('Lane_Dect_Node', anonymous=True)
        device = torch.device('cuda' if torch.cuda.is_available() else "cpu")
        cls_num_per_lane = 18
        net = parsingNet(pretrained=False, backbone='18', cls_dim=(201, cls_num_per_lane, 4),
                         use_aux=False).cuda()  # we dont need auxiliary segmentation in testing
        state_dict = torch.load('/home/kaai/chicago_ws/src/first_pkg/src/Pytorch_Lane_Detection/configs/culane_18.pth',
                                map_location='cpu')['model']
        compatible_state_dict = {}
        for k, v in state_dict.items():
            if 'module.' in k:
                compatible_state_dict[k[7:]] = v
            else:
                compatible_state_dict[k] = v
        net.load_state_dict(compatible_state_dict, strict=False)
        net.eval()
        from sensor_msgs.msg import Image
        pub1 = rospy.Publisher('/Object_image_topic', Image, queue_size=1)
        pub2 = rospy.Publisher('/lane_image_topic', Image, queue_size=1)

        listener()
    except rospy.ROSInterruptException:
        pass
