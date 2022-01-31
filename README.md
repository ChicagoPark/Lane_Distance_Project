`Supported by the National Research Foundation of Korea(NRF) grant funded by the Korea government(MSIT)`

### `Real Time Lateral Distance Estimation of other vehicles`

<img width="450" alt="LiDAR_Arrange" src="https://user-images.githubusercontent.com/73331241/140094544-b9ef6080-121a-44b9-ad79-0d5e1e841e77.jpeg">

## [Basic Sensor] 7 Livox Horizon LiDARs which cover 360 degree, ZED stereo Camera
<img width="450" alt="LiDAR_Arrange" src="https://user-images.githubusercontent.com/73331241/139469477-5e33ac45-71a2-47df-b833-db787c210d53.jpg">

### [0] Project Introduction

`Distance between mid-center points of vehicles and adjacent lanes`

`2D Object Tracking and Lane Detection methods are omitted. My another repository dealt with that.`

### [1] Detecting Lanes and Tracking Objects

https://user-images.githubusercontent.com/73331241/139474003-86f494d0-bda1-44cb-9dca-29766638d81b.mp4

### [2] Point Cloud Data Preprocessing

#### [2-1] Merging multi LiDARs

<img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/140095391-57a5bc40-38da-445e-9c62-de1a6c5f790a.jpeg">

`Before Merging / After Merging`

#### [2-2] Point Cloud Segmentation

<img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/140095450-6cfeb750-521e-41d6-b9aa-f08487e7f30e.jpeg">

`Obstacle Point Cloud / Plane Point Cloud`

### [3] Interesting Pixels Plotting on Image

### Get a point on the lower middle part of bounding box

### Get the points of lanes corresponding adjacent objects

https://user-images.githubusercontent.com/73331241/139473571-bbcd0951-5f6a-44ec-8ecc-5b511dec6036.mov

### [4] Preparing to get Point Cloud projected on Image point for getting distance

#### [4-1] Project the 3D points of obstacles and plane onto the image, respectively

<img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/140095109-bf828129-b8e8-43d7-8cba-02e87a5b9a08.jpeg">


#### [4-2] Measure the distance between the two points using the projected point cloud on specific pixels

https://user-images.githubusercontent.com/73331241/139473050-5e7f5257-36fd-466d-b767-e769b5859b70.mp4

### [5] Appling in real time on driving and display the distance


https://user-images.githubusercontent.com/73331241/139472235-0020e972-21e3-4877-b007-5598c384c4a9.mp4


<!--
# 코드 움직임
1. 카메라와 라이다 작동시키기
2. 오브젝트 디텍션
3.
최종 : distance_esitmation.cpp
-->
