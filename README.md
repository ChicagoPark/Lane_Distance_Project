# Supported by the National Research Foundation of Korea(NRF) grant funded by the Korea government(MSIT)

### `Real Time Lateral Distance Estimation of other vehicles`

## [Basic Sensor] 7 Livox Horizon LiDARs which cover 360 degree, ZED stereo Camera
<img width="450" alt="LiDAR_Arrange" src="https://user-images.githubusercontent.com/73331241/139469477-5e33ac45-71a2-47df-b833-db787c210d53.jpg">

## [0] Project Introduction

`Distance between mid-center points of vehicles and adjacent lanes`


`2D Object Tracking and Lane Detection methods are omitted. My another repository dealt with that.`

## [1] Detecting Lanes and Tracking Objects

https://user-images.githubusercontent.com/73331241/139474003-86f494d0-bda1-44cb-9dca-29766638d81b.mp4

## [2] Interesting Pixels Plotting on Image

### 박스의 중심 하부점에 대해 점을 받아오기

```python
# 오브젝트 좌표 얻어오는것
```
### Object 점에서 인접한 Lane 에 대한 정보를 얻어오기

```python
# 차선 좌표 얻어오는 코드
```

https://user-images.githubusercontent.com/73331241/139473571-bbcd0951-5f6a-44ec-8ecc-5b511dec6036.mov

## [3] Preparing to get Point Cloud projected on Image point for getting distance

https://user-images.githubusercontent.com/73331241/139473050-5e7f5257-36fd-466d-b767-e769b5859b70.mp4

## [4] Appling in real time on driving and display the distance

### 우리가 [2] 에서 구한 interesting point 에 projecting 이 된 Point cloud 에 접근을 하여 거리를 추정하는 것

```python
#point 매칭 부등호 
```

```python
# 거리를 추정하는 
```

https://user-images.githubusercontent.com/73331241/139472235-0020e972-21e3-4877-b007-5598c384c4a9.mp4



# 코드 움직임
1. 카메라와 라이다 작동시키기
2. 오브젝트 디텍션
3.
최종 : distance_esitmation.cpp
