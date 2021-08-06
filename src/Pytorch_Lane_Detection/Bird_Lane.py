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

        #trandform background




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
        print(out_j.shape[1])
        num_lane = out_j.shape[1]

        c = 0
        d = 200

        for i in range(num_lane):
            point_list_for_curve = []
            if np.sum(out_j[:, i] != 0) > 2:
                for k in range(out_j.shape[0]):
                    if out_j[k, i] > 0:
                        # ppp has (a, b) format. This means each point location !
                        ppp = (int(out_j[k, i] * col_sample_w * img_w / 800) - 1,
                               int(img_h * (row_anchor[cls_num_per_lane - 1 - k] / 288)) - 1)
                        cv2.circle(frame, ppp, 5, (0, c, d), -1)
                        point_list_for_curve.append(ppp)
                    '''
                        #Left Lane
                        if i == 1:

                        #Right Lane
                        if i == 2:
                    '''
            point_list_for_curve = np.array(point_list_for_curve, np.int32)
            cv2.polylines(frame, [point_list_for_curve], False, (0,c,d), thickness=7)
            c += 50
            d -= 80
        #out = out.cpu().numpy()
        #print(out.shape)
        frame = cv2.resize(frame, (1640, 590), interpolation=cv2.INTER_AREA)

        src = np.float32(
            [[940, 268],  # top right
             [1185, 404],  # bottom right
             [323, 404],  # bottom left
             [618, 268]  # top left
             ]
        )

        dst = np.float32(
            [[1000, 355],  # top right
             [1000, 530],
             [500, 530],
             [500, 355]]
        )

        M = cv2.getPerspectiveTransform(src, dst)
        

        frame = cv2.warpPerspective(frame, M, (1640, 590), flags=cv2.INTER_LINEAR)


        cv2.imshow('output', frame)
        key = cv2.waitKey(20)
        vout.write(frame)  # every frame 에 이미지를 저장한다고 한다.
        if key == 27:
            break
    vid.release()
    vout.release()
    cv2.destroyAllWindows()
