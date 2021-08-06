import cv2
import torch, os, cv2
from model.model import parsingNet
from utils1.common import merge_config
from utils1.dist_utils import dist_print
import torch
import scipy.special, tqdm
import numpy as np
import torchvision.transforms as transforms
from data.dataset import LaneTestDataset
from data.constant import culane_row_anchor, tusimple_row_anchor, youngil_row_anchor
from PIL import Image

import matplotlib.pyplot as plt

if __name__ == '__main__':
    # Load model
    #args, cfg = merge_config()
    device = torch.device('cuda' if torch.cuda.is_available() else "cpu")

    #assert cfg.backbone in ['18', '34', '50', '101', '152', '50next', '101next', '50wide', '101wide']

    cls_num_per_lane = 18

    net = parsingNet(pretrained = False, backbone='18', cls_dim = (201,cls_num_per_lane,4),use_aux=False).cuda() # we dont need auxiliary segmentation in testing

    # Load model information
    state_dict = torch.load('configs/culane_18.pth', map_location='cpu')['model']

    compatible_state_dict = {}
    for k, v in state_dict.items():
        if 'module.' in k:
            compatible_state_dict[k[7:]] = v
        else:
            compatible_state_dict[k] = v
    net.load_state_dict(compatible_state_dict, strict=False)
    net.eval()

    #Image Preprocessing
    '''
    img_transforms = transforms.Compose([
        transforms.Resize((288, 800)),
        transforms.ToTensor(),
        transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
    ])
    '''
    #vid = cv2.VideoCapture(0)
    vid = cv2.VideoCapture('video_input/Dash cam video _ Driving on a highway 2016.mp4')

    cv2.namedWindow("Youngil")
    vid_fps = int(vid.get(cv2.CAP_PROP_FPS))
    # Getting video size
    vid_width, vid_height = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)), int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))

    codec = cv2.VideoWriter_fourcc(*'XVID')  # for avi file output
    #codec = cv2.VideoWriter_fourcc(*'MP4V')     #for MP4 file output

    #size of the width and height has to be same size with output image's size.
    vout = cv2.VideoWriter('video_output/Youngil_3.mp4', codec, vid_fps, (1640, 590))

    #row_anchor = culane_row_anchor
    row_anchor = youngil_row_anchor


    #should to get image size about input
    rval, frame = vid.read()
    img_w, img_h = frame.shape[1], frame.shape[0]



    while True:

        rval, frame = vid.read()

        #model_input = torch.from_numpy(frame)

        model_input = Image.fromarray(frame)
        model_input =transforms.Resize((288,800))(model_input)
        model_input = transforms.ToTensor()(model_input)
        model_input = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])(model_input)


        model_input = model_input.to(device).float()

        with torch.no_grad():
            out = net(model_input[None, ...])

        col_sample = np.linspace(0, 800 - 1, 200)
        col_sample_w = col_sample[1] - col_sample[0]

        #out_j has information about each line from the left
        out_j = out[0].data.cpu().numpy()
        out_j = out_j[:, ::-1, :]
        prob = scipy.special.softmax(out_j[:-1, :, :], axis=0)
        idx = np.arange(200) + 1
        idx = idx.reshape(-1, 1, 1)
        loc = np.sum(prob * idx, axis=0)
        out_j = np.argmax(out_j, axis=0)
        loc[out_j == 200] = 0
        out_j = loc

        #out = out.view(36, 134, 3)
        #out = out.cpu().numpy()

        # c : for draw the line with different color
        #out_j.shape[1] means the number of lane

        num_lane = out_j.shape[1]
        if num_lane <= 2:
            c= 50
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
                            #print(ppp[0], '    ', ppp[1])
                            leftx.append(ppp[0])
                            lefty.append(ppp[1])
                            #print(leftx)
                        if i == 2:
                            #print(ppp[0], '    ', ppp[1])
                            rightx.append(ppp[0])
                            righty.append(ppp[1])

                        cv2.circle(frame, ppp, 5, (0, c, d), -1)
                        point_list_for_curve.append(ppp)


            k = [] # plot 을 하기위해서 담는 좌측 레인의 x, y 좌표
            l = [] # plot 을 하기위해서 담는 우측 레인의 x, y 좌표

            # 2차 함수 구하기
            if len(leftx) != 0 and len(lefty) != 0:
                left_fit = np.polyfit(lefty, leftx, 2)

                #ploty = np.linspace(0, frame.shape[0] - 1, frame.shape[0])
                ploty = np.linspace(int(frame.shape[0]/3), frame.shape[0]-1, int(frame.shape[0]*(2/3))) # y좌표에 대해서 상위 1/3 지점 부터 최하단 까지 그래프를 그리기위한 코드
                left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]                # 방정식에 대한 x 값을 얻어오기 위한 코드
                for ii, x in enumerate(left_fitx):
                    q = (x, ii + int(frame.shape[0]/3))                                                 # 상위 1/3 지점부터 그래프를 그리기 시작하므로 인덱스에 해당하는 ii 에 그만큼 추가해준다.
                    if(x >= 0):
                        k.append(q)
                k = np.array(k, np.int32)                                                               # polylines 함수에 좌표는 np.array 형태를 유지해줘야하므로 추가한 코드이다.
                cv2.polylines(frame, [k], False, (235, 206, 135), thickness=7)                         # 좌측 레인 : sky blue color
                #print(left_fitx)

            if len(rightx) != 0 and len(righty) != 0:
                right_fit = np.polyfit(righty, rightx, 2)
                right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
                for ii, x in enumerate(right_fitx):
                    q = (x, ii + int(frame.shape[0]/3))
                    if(x >= 0):
                        l.append(q)
                l = np.array(l, np.int32)
                cv2.polylines(frame, [l], False, (160, 114, 0), thickness=7)                           # 우측 레인 : deep blue color



            point_list_for_curve = np.array(point_list_for_curve, np.int32)

            #cv2.polylines(frame, [point_list_for_curve], False, (0,c,d), thickness=7)


            c += 50
            d -= 80
        #out = out.cpu().numpy()
        #print(out.shape)
        frame = cv2.resize(frame, (1640, 590), interpolation=cv2.INTER_AREA)
        cv2.imshow('output', frame)
        key = cv2.waitKey(20)
        vout.write(frame)  # every frame 에 이미지를 저장한다고 한다.
        if key == 27:
            break
    vid.release()
    vout.release()
    cv2.destroyAllWindows()
