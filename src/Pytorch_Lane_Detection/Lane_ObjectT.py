############################Lane Detection Module##################################
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
############################Lane Detection Module##################################

############################Object Tracking Module###############################






############################Object Tracking Module###############################
if __name__ == '__main__':
    # Load model
    args, cfg = merge_config()

    device = torch.device('cuda' if torch.cuda.is_available() else "cpu")

    assert cfg.backbone in ['18', '34', '50', '101', '152', '50next', '101next', '50wide', '101wide']

    if cfg.dataset == 'CULane':
        cls_num_per_lane = 18

    net = parsingNet(pretrained = False, backbone=cfg.backbone,cls_dim = (cfg.griding_num+1,cls_num_per_lane,4),
                    use_aux=False).cuda() # we dont need auxiliary segmentation in testing

    # Load model information
    state_dict = torch.load(cfg.test_model, map_location='cpu')['model']

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
    vid = cv2.VideoCapture('video_input/Road - 1101.mp4')
    cv2.namedWindow("Youngil")
    vid_fps = int(vid.get(cv2.CAP_PROP_FPS))
    # Getting video size
    vid_width, vid_height = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)), int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))

    codec = cv2.VideoWriter_fourcc(*'XVID')  # for avi file output
    #codec = cv2.VideoWriter_fourcc(*'MP4V')     #for MP4 file output

    #size of the width and height has to be same size with output image's size.
    vout = cv2.VideoWriter('video_output/Youngil_2.mp4', codec, vid_fps, (1640, 590))

    row_anchor = culane_row_anchor
    #row_anchor = youngil_row_anchor


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

        col_sample = np.linspace(0, 800 - 1, cfg.griding_num)
        col_sample_w = col_sample[1] - col_sample[0]

        out_j = out[0].data.cpu().numpy()
        out_j = out_j[:, ::-1, :]
        prob = scipy.special.softmax(out_j[:-1, :, :], axis=0)
        idx = np.arange(cfg.griding_num) + 1
        idx = idx.reshape(-1, 1, 1)
        loc = np.sum(prob * idx, axis=0)
        out_j = np.argmax(out_j, axis=0)
        loc[out_j == cfg.griding_num] = 0
        out_j = loc


        out = out.view(36, 134, 3)
        out = out.cpu().numpy()
        print(out.shape)

        for i in range(out_j.shape[1]):
            if np.sum(out_j[:, i] != 0) > 2:
                for k in range(out_j.shape[0]):
                    if out_j[k, i] > 0:
                        ppp = (int(out_j[k, i] * col_sample_w * img_w / 800) - 1,
                               int(img_h * (row_anchor[cls_num_per_lane - 1 - k] / 288)) - 1)
                        cv2.circle(frame, ppp, 5, (0, 255, 0), -1)
        #out = out.cpu().numpy()
        print(out.shape)
        frame = cv2.resize(frame, (1640, 590), interpolation=cv2.INTER_AREA)
        cv2.imshow('output', frame)
        key = cv2.waitKey(20)
        vout.write(frame)  # every frame 에 이미지를 저장한다고 한다.
        if key == 27:
            break
    vid.release()
    vout.release()
    cv2.destroyAllWindows()