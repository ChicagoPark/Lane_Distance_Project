import cv2

vid = cv2.VideoCapture(0)
# vid = cv2.VideoCapture('./data/.mp4')
cv2.namedWindow("Youngil")

vid_fps = int(vid.get(cv2.CAP_PROP_FPS))

# Getting video size
vid_width, vid_height = int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)), int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))

# rval, frame = vid.read()

# Video saving process
codec = cv2.VideoWriter_fourcc(*'XVID')  # for avi file output
# codec = cv2.VideoWriter_fourcc(*'MP4V')     #for MP4 file output

out = cv2.VideoWriter('./Lane_detect.avi', codec, vid_fps, (vid_width, vid_height))

while True:

    rval, frame = vid.read()
    # How to change the image size

    output = cv2.resize(frame, (1640, 590), interpolation=cv2.INTER_AREA)
    print(type(output))
    cv2.imshow('output', output)
    key = cv2.waitKey(20)
    if key == 27:
        break

cv2.destroyWindow("Youngil")

'''
    img_in = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #cv2.putText(img, "FPS: {:.2f}".format(fps), (0, 30), 0, 1, (0, 0, 255), 2)  # frame 을 명시해준다.
    cv2.resizeWindow('output', 1024, 768) 

    out.write(img)  # every frame 에 이미지를 저장한다고 한다.
'''