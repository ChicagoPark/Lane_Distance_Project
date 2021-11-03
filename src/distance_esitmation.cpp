#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_msgs/String.h"
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <vector>
#include <math.h>
#include <first_pkg/lane_object.h>
#include <cmath>


using namespace std;

int sub_data[5];

ros::Publisher pub1;
ros::Publisher pub2;
cv::Mat img(720,1280,CV_8UC3, cv::Scalar(255,255,255));
cv::Mat img_object(720,1280,CV_8UC3, cv::Scalar(255,255,255));

void lane_object_pixel(const first_pkg::lane_object::ConstPtr& array_msg)
{
    sub_data[0] = array_msg->data[0];
    sub_data[1] = array_msg->data[1];
    sub_data[2] = array_msg->data[2];
    sub_data[3] = array_msg->data[3];
    sub_data[4] = array_msg->data[4];
    ROS_INFO("I heard : [%d, %d, %d, %d, %d]", sub_data[0], sub_data[1], sub_data[2], sub_data[3], sub_data[4]);
    //ROS_INFO("I heard : [%d, %d, %d, %d, %d]", array_msg->data[0], array_msg->data[1], array_msg->data[2], array_msg->data[3], array_msg->data[4]);
}


void image_raw(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  img = cv_ptr->image;
}

cv::Point *pt_Object = new cv::Point;
cv::Point *pt_Lane = new cv::Point;

float object_lidar_array[3]; // for getting 3D points for object
float lane_lidar_array[3];   // for getting 3D points for lane

// for getting obstacle RANSAC result
void projection_object(const sensor_msgs::PointCloud2ConstPtr& cloud_msg1)
{
  //ROS_INFO("I heard : [%d, %d, %d, %d, %d]", sub_data[1], sub_data[2], sub_data[3], sub_data[4]);
  //std::vector<cv::Point3f> points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ptc (new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg (*cloud_msg1, *pcl_ptc);

  pcl::PassThrough<pcl::PointXYZI> passz;
  passz.setInputCloud (pcl_ptc);
  passz.setFilterFieldName ("x");
  passz.setFilterLimits (1.2, 100);
  passz.filter (*pcl_ptc);

  //std::cout << int(pcl_ptc->points[20].y);

  //for (const auto& point : *pcl_ptc) {
  //points.push_back(cv::Point3f(point.x, point.y, point.z));}

  cv::Mat p(3,4,cv::DataType<double>::type);
  cv::Mat rt(4,4,cv::DataType<double>::type);
  cv::Mat r(4,4,cv::DataType<double>::type);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// 카메라 projection matrix
  p.at<double>(0,0) = 675.5181884765625;    p.at<double>(0,1) = 0.00000000;    p.at<double>(0,2) = 600.1480712890625;    p.at<double>(0,3) = 0.00000000;
  p.at<double>(1,0) = 0.00000000;    p.at<double>(1,1) = 675.5181884765625;    p.at<double>(1,2) = 387.63726806640625;    p.at<double>(1,3) = 0.00000000;
  p.at<double>(2,0) = 0.00000000;    p.at<double>(2,1) = 0.00000000;    p.at<double>(2,2) = 1.00000000;    p.at<double>(2,3) = 0.00000000;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Rotation matrix
  rt.at<double>(0,0) = -0.0440196;    rt.at<double>(0,1) = -0.99941;    rt.at<double>(0,2) = -0.00472239;    rt.at<double>(0,3) = -0.0317374;  // x축 (좌측이동시 -)
  rt.at<double>(1,0) = -0.0122759;    rt.at<double>(1,1) = -0.00321113;    rt.at<double>(1,2) = -0.999472;    rt.at<double>(1,3) = 0.0092413;  // y축 이동(위로 이동시 -)
  rt.at<double>(2,0) = 0.99891;    rt.at<double>(2,1) = -0.0338506;    rt.at<double>(2,2) = -0.0321547;    rt.at<double>(2,3) = -0.555356;  //확대 축소 (줌인(-))
  rt.at<double>(3,0) = 0.00000000;    rt.at<double>(3,1) = 0.00000000;    rt.at<double>(3,2) = 0.00000000;    rt.at<double>(3,3) = 1.00000000;

  cv::Mat X(4,1,cv::DataType<double>::type);
  cv::Mat Y(3,1,cv::DataType<double>::type);
  cv::Mat visImg = img.clone();
  cv::Mat overlay = visImg.clone();

// 단순히 x, y 픽셀 평행 이동
  double offset_x = 20.00;
  double offset_y = 0.00;
  int number = 0;

// getting distance
/*
  float object_lidar_array[3]; // for getting 3D points for object
  float lane_lidar_array[3];   // for getting 3D points for lane
*/
  // Draw a point on Image
  for(auto it=pcl_ptc->begin(); it!=pcl_ptc->end(); ++it) {
    X.at<double>(0,0) = it->x; X.at<double>(1,0) = it->y; X.at<double>(2,0) = it->z; X.at<double>(3,0) = 1;
    Y = p * rt * X;
    cv::Point pt;
    pt.x = Y.at<double>(0,0) / Y.at<double>(0,2) + offset_x;
    pt.y = Y.at<double>(1,0) / Y.at<double>(0,2) + offset_y;
    //std::cout << it->z <<std::endl;ros::Publisher pub;

    float val = it->x;
    float maxval = 10;


    int red = std::min(255, (int)(255*abs((val - maxval)/maxval)));
    //std::cout<<red<<std::endl;
    int green = std::min(255, (int)(255*(1-abs((val - maxval)/maxval))));

    cv::circle(overlay, pt, 2, cv::Scalar(it->intensity,green,red), -1);


    int object_x = sub_data[0];
    int object_y = sub_data[1];
    int lane_x = sub_data[2];
    int lane_y = sub_data[3];ros::Publisher pub;
    int object_id = sub_data[4];


    //if (pt.x == "Object Target x Pixel" && pt.y == "Object Target y Pixel")
    if ((pt.x >= object_x - 5 && pt.x <= object_x + 5) && (pt.y >= object_y - 5 && pt.y <= object_y + 5))
    //if ((pt.x == 400 && pt.y == 400)|| (pt.x == 403 && pt.y == 403))
    {
        object_lidar_array[0] = X.at<double>(0,0);
        object_lidar_array[1] = X.at<double>(1,0);

        object_lidar_array[2] = X.at<double>(2,0);

        pt_Object->x = pt.x;
        pt_Object->y = pt.y;
    }

    /*
    //if (pt.x == "lane Target x Pixel" && pt.y == "lane Target y Pixel")
    if ((pt.x >= lane_x - 5 && pt.x <= lane_x + 5) && (pt.y >= lane_y - 5 && pt.y <= lane_y + 5))
    //if (pt.x == 875 && pt.y == 400)
    {
        lane_lidar_array[0] = X.at<double>(0,0);
        lane_lidar_array[1] = X.at<double>(1,0);
        lane_lidar_array[2] = X.at<double>(2,0);

        pt_Lane->x = pt.x;
        pt_Lane->y = pt.y;
    }
    */
    /*
    if (sub_data[0] != 0 && sub_data[1] !=0 && sub_data[2] !=0 && sub_data[3] !=0 && sub_data[4] !=0)
    {
        cv::circle(overlay, *pt_Object, 2, cv::Scalar(0,255,255), 5);

        cv::circle(overlay, *pt_Lane, 2, cv::Scalar(255,255,0), 5);
    }
    */
  }
/*
  int distance;
  // distance estimation
  if (pt_Object != NULL && pt_Lane != NULL)
  {
      float a = pow(2, (object_lidar_array[0] - lane_lidar_array[0]));
      float b = pow(2, (object_lidar_array[1] - lane_lidar_array[1]));
      float distance = sqrt(a + b);
      //cout << object_array[1] << "\n\n\n\n\n";
      //cout << lane_array[1] << "\n\n\n\n\n";
      cout << "Distance : "<< distance << "\n\n";

      //cout << distance << '\n\n\n\n\n\n';
      //cout << object_array[0];
  }5
*/

  //cv::circle(overlay, *pt_Object, 2, cv::Scal#include<cmath>ar(255,255,0), 5);

  //cv::circle(overlay, *pt_Lane, 2, cv::Scalar(255,255,0), 5);

  float opacity = 0.7;
  cv::addWeighted(overlay, opacity, visImg, 1-opacity, 0, visImg);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImg).toImageMsg();
  pub1.publish(msg);
}

void projection_lane(const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
{
  //std::vector<cv::Point3f> points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ptc (new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg (*cloud_msg2, *pcl_ptc);

  pcl::PassThrough<pcl::PointXYZI> passz;
  passz.setInputCloud (pcl_ptc);
  passz.setFilterFieldName ("x");
  passz.setFilterLimits (1.5, 100);
  passz.filter (*pcl_ptc);

  //std::cout << int(pcl_ptc->points[20].y);ros::Publisher pub;

  //for (const auto& point : *pcl_ptc) {
  //points.push_back(cv::Point3f(point.x, point.y, point.z));}

  cv::Mat p(3,4,cv::DataType<double>::type);
  cv::Mat rt(4,4,cv::DataType<double>::type);  cv::Mat r(4,4,cv::DataType<double>::type);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// 카메라 projection matrix
  p.at<double>(0,0) = 675.5181884765625;    p.at<double>(0,1) = 0.00000000;    p.at<double>(0,2) = 600.1480712890625;    p.at<double>(0,3) = 0.00000000;
  p.at<double>(1,0) = 0.00000000;    p.at<double>(1,1) = 675.5181884765625;    p.at<double>(1,2) = 387.63726806640625;    p.at<double>(1,3) = 0.00000000;
  p.at<double>(2,0) = 0.00000000;    p.at<double>(2,1) = 0.00000000;    p.at<double>(2,2) = 1.00000000;    p.at<double>(2,3) = 0.00000000;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Rotation matrix
  rt.at<double>(0,0) = -0.0440196;    rt.at<double>(0,1) = -0.99941;    rt.at<double>(0,2) = -0.00472239;    rt.at<double>(0,3) = -0.0317374;  // x축 (좌측이동시 -)
  rt.at<double>(1,0) = -0.0122759;    rt.at<double>(1,1) = -0.00321113;    rt.at<double>(1,2) = -0.999472;    rt.at<double>(1,3) = 0.0092413;  // y축 이동(위로 이동시 -)
  rt.at<double>(2,0) = 0.99891;    rt.at<double>(2,1) = -0.0338506;    rt.at<double>(2,2) = -0.0321547;    rt.at<double>(2,3) = -0.555356;  //확대 축소 (줌인(-))
  rt.at<double>(3,0) = 0.00000000;    rt.at<double>(3,1) = 0.00000000;    rt.at<double>(3,2) = 0.00000000;    rt.at<double>(3,3) = 1.00000000;

  //ros::Subscriber sub3 = nh.subscribe("/livox/lidar_3WEDH7600114591", 1,
  cv::Mat X(4,1,cv::DataType<double>::type);
  cv::Mat Y(3,1,cv::DataType<double>::type);
  cv::Mat visImg = img.clone();
  cv::Mat overlay = visImg.clone();

// 단순히 x, y 픽셀 평행 이동
  double offset_x = 20.00;ros::Publisher pub;
  double offset_y = 0.00;
  int number = 0;

// getting distance
/*
  float object_lidar_array[3]; // for getting 3D points for object
  float lane_lidar_array[3];   // for getting 3D points for lane
*/


  /*(1) Save exact point cloud point for Object detection pixel when it has same pixel value
   *(2) Save exact point cloud point for Lane detection pixel when it has same pixel value
   * */
  /*
  cv::Point *pt_Object = new cv::Point;#include<cmath>
  cv::Point *pt_Lane = new cv::Point;
  */

/*
  int count = 0;
  for (const auto& pt: pcl_ptc->points){
          //cout << count++ << ": ";
          //cout <<pt.x << ", "<<pt.y << ", "<< pt.z << endl;
      }
*/
  // Draw a point on Image
  for(auto it=pcl_ptc->begin(); it!=pcl_ptc->end(); ++it) {
    X.at<double>(0,0) = it->x; X.at<double>(1,0) = it->y; X.at<double>(2,0) = it->z; X.at<double>(3,0) = 1;
    Y = p * rt * X;
    cv::Point pt;
    pt.x = Y.at<double>(0,0) / Y.at<double>(0,2) + offset_x;
    pt.y = Y.at<double>(1,0) / Y.at<double>(0,2) + offset_y;
    //std::cout << it->z <<std::endl;

    float val = it->x;
    float maxval = 10;



    int red = std::min(255, (int)(255*abs((val - maxval)/maxval)));
    //std::cout<<red<<std::endl;
    int green = std::min(255, (int)(255*(1-abs((val - maxval)/maxval))));


    /*
    cout << X.at<double>(0,0) << "\n\n";#include<cmath>
    cout << X.at<double>(1,0) << "\n\n";
    cout << X.at<double>(2,0) << "\n\n";
    */

    int object_x = sub_data[0];
    int object_y = sub_data[1];
    int lane_x = sub_data[2];
    int lane_y = sub_data[3];
    int object_id = sub_data[4];


    /*
    //if (pt.x == "Object Target x Pixel" && pt.y == "Object Target y Pixel")
    if ((pt.x >= object_x - 5 && pt.x <= object_x + 5) && (pt.y >= object_y - 5 && pt.y <= object_y + 5))
    //if ((pt.x == 400 && pt.y == 400)|| (pt.x == 403 && pt.y == 403))
    {
        object_lidar_array[0] = X.at<double>(0,0);
        object_lidar_array[1] = X.at<double>(1,0);
        object_lidar_array[2] = X.at<double>(2,0);

        pt_Object.x = pt.x;
        pt_Object.y = pt.y;
    }
    */
    //if (pt.x == "lane Target x Pixel" && pt.y == "lane Target y Pixel")
    if ((pt.x >= lane_x - 5 && pt.x <= lane_x + 5) && (pt.y >= lane_y - 5 && pt.y <= lane_y + 5))
    //if (pt.x == 875 && pt.y == 400)
    {
        lane_lidar_array[0] = X.at<double>(0,0);
        lane_lidar_array[1] = X.at<double>(1,0);
        lane_lidar_array[2] = X.at<double>(2,0);

        pt_Lane->x = pt.x;
        pt_Lane->y = pt.y;
    }

    if (sub_data[0] != 0 && sub_data[1] !=0 && sub_data[2] !=0 && sub_data[3] !=0 && sub_data[4] !=0 && pt_Object->y == pt_Lane->y)
    {
        cv::circle(overlay, *pt_Object, 2, cv::Scalar(0,255,255), 5);
        cv::circle(overlay, *pt_Lane, 2, cv::Scalar(255,255,0), 5);
        cout << "x : " << pt_Object->x;
        cout << "y : " << pt_Object->y << "\n";
        cout << "x : " << pt_Lane->x;
        cout << "y : " << pt_Lane->y << "\n";
    }

  }

  int distance;
  // distance estimation

  if (pt_Object != NULL && pt_Lane != NULL && sub_data[0] != 0 && sub_data[1] !=0 && sub_data[2] !=0 && sub_data[3] !=0 && sub_data[4] !=0)
  {
      /*
      float a = pow(2, (object_lidar_array[0] - lane_lidar_array[0]));
      float b = pow(2, (object_lidar_array[1] - lane_lidar_array[1]));
      float distance = sqrt(a + b);
      */
      float distance = abs(pt_Object->y - pt_Lane->y);
      cout << "Distance : "<< distance << "\n\n";

      distance = std::round(distance*100) / 100;


      string string_distance = to_string(distance);
      string_distance.append("M");
      cv::Point Distance_Point;
      Distance_Point.x = sub_data[2] - 40;
      Distance_Point.y = sub_data[3] - 10;
      cv::putText(overlay, string_distance, Distance_Point, 3, 0.6, CV_RGB(255, 255, 255));

  }



  //cv::circle(overlay, *pt_Object, 2, cv::Scalar(255,255,0), 5);

  //cv::circle(overlay, *pt_Lane, 2, cv::Scalar(255,255,0), 5);

  float opacity = 0.7;
  cv::addWeighted(overlay, opacity, visImg, 1-opacity, 0, visImg);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "8UC4", visImg).toImageMsg();
  pub2.publish(msg);

  // To prevent cv2 circle, which is came from ramained points, to stay
  sub_data[0] = 0, sub_data[1]=0,sub_data[2]=0, sub_data[3]=0,sub_data[4]=0;


}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "projection");
  ros::NodeHandle nh;

  // 차선을 인식한 결과를 담은 이미지를 가진다.
  ros::Subscriber sub1 = nh.subscribe("/lane_image_topic", 1, image_raw);
  //ros::Subscriber sub2 = nh.subscribe ("/point_process/livox_merge", 1, projection);
  //lane_image_topic

  // object detection 의 중심 하부에 대한 정보를 가진다.
  ros::Subscriber sub2 = nh.subscribe("Obst", 1, projection_object);

  //ros::Subscriber sub3 = nh.subscribe("/livox/lidar_3WEDH7600114591", 1, projection_lane);
  
  // 물체에 대한 인접 Lane 좌표에 대한 픽셀 정보
  ros::Subscriber sub3 = nh.subscribe("Plane", 1, projection_lane);
  
  // 얘는 확인해봐야할 
  ros::Subscriber sub4 = nh.subscribe("/pixel_topic", 1, lane_object_pixel);

  pub1 = nh.advertise<sensor_msgs::Image>("projected_obst", 1);


  pub2 = nh.advertise<sensor_msgs::Image>("projected_lane", 1);


  ros::spin ();
}
