#include "ros/ros.h"
#include "math.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "std_msgs/String.h"

#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>


ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZI> ());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud5 (new pcl::PointCloud<pcl::PointXYZI> ());

void ///livox/lidar_3WEDH5900101801  front right
cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg_2)
{
  pcl::PointCloud<pcl::PointXYZI>* source_cloud_2 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud_2 = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::fromROSMsg (*cloud_msg_2, *source_cloud_2);

  Eigen::Translation3f init_translation(-0.0709682, -0.228406, -0.0323041);
  Eigen::AngleAxisf init_rotation_x(3.1371, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(-3.13974261, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(2.389844438, Eigen::Vector3f::UnitZ()); // 라디안으로 계산해준다.

  Eigen::Matrix4f m = (init_translation * init_rotation_x*init_rotation_y* init_rotation_z).matrix();

  pcl::transformPointCloud (*source_cloud_2, *transformed_cloud_2, m);
  pcl::copyPointCloud(*transformed_cloud_2, *cloud2);

  delete source_cloud_2;
  delete transformed_cloud_2;

}

///////////////////////////////////////////////////////////////////////////////////////////////


void // livox/(back)
cloud_cb5 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg_5)
{
  //pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud_5 = (new pcl::PointCloud<pcl::PointXYZI>);
  //pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_5 = (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy_f (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg (*cloud_msg_5, *cloud_copy);

  Eigen::Translation3f init_translation(-0.0580248, 0.168895, 0.00728544);
  Eigen::AngleAxisf init_rotation_x(-0.0138957, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(-0.003501, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(0.772943, Eigen::Vector3f::UnitZ()); // 라디안으로 계산해준다.

  Eigen::Matrix4f m = (init_translation * init_rotation_x*init_rotation_y* init_rotation_z).matrix();

  pcl::transformPointCloud (*cloud_copy, *cloud_copy_f, m);
 // pcl::copyPointCloud(*transformed_cloud_5, *cloud5);

  pcl::PointCloud<pcl::PointXYZI>::Ptr zoff (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass1; 
  pass1.setInputCloud (cloud_copy_f);
  pass1.setFilterFieldName ("z");
  pass1.setFilterLimits (-1, 0.5);
  pass1.filter (*zoff);

  pcl::copyPointCloud(*zoff, *cloud5);

  //delete source_cloud_5;
  //delete transformed_cloud_5;
  //delete zoff;

}

//////////////////////////////////////////////////////////////////////////////////////////////

void 
cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& cloud_msg2)
{
  pcl::fromROSMsg (*cloud_msg2, *cloud1);
  pcl::PointCloud<pcl::PointXYZI>* merge_cloud = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::PCLPointCloud2* cloud1_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud2_pc = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2* cloud5_pc = new pcl::PCLPointCloud2;
  pcl::PointCloud<pcl::PointXYZI>* transformed_cloud_f = new pcl::PointCloud<pcl::PointXYZI>;

  pcl::PCLPointCloud2* merge_cloud_pc = new pcl::PCLPointCloud2;

  pcl::toPCLPointCloud2(*cloud1, *cloud1_pc);
  pcl::toPCLPointCloud2(*cloud2, *cloud2_pc);
  pcl::toPCLPointCloud2(*cloud5, *cloud5_pc);
  pcl::toPCLPointCloud2(*merge_cloud, *merge_cloud_pc);

  pcl::concatenatePointCloud(*cloud1_pc, *cloud2_pc, *merge_cloud_pc);
  pcl::concatenatePointCloud(*merge_cloud_pc, *cloud5_pc, *merge_cloud_pc);


  sensor_msgs::PointCloud2 output; 
  pcl_conversions::fromPCL(*merge_cloud_pc, output);
  pub.publish(output);
  delete merge_cloud;
  delete cloud1_pc;
  delete cloud2_pc;
  delete cloud5_pc;
  delete merge_cloud_pc;

}


///////////////////////////////////////////////////////////////////////////////////////////////

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "merge");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("/livox/lidar_3WEDH7600114591", 1, cloud_cb1);
  ros::Subscriber sub2 = nh.subscribe ("/livox/lidar_3WEDH5900101801", 1, cloud_cb2);
  ros::Subscriber sub5 = nh.subscribe ("/livox/lidar_3WEDH7600101631", 1, cloud_cb5);
  

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_process/livox_merge", 1);
 
  // Spin
  ros::spin ();
}
