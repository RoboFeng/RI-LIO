#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <ros/ros.h>
#include <mypoint_type.h>
#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef ouster_ros::Point PointOuster;
typedef pcl::PointCloud<PointOuster> PointCloudOuster;

enum LID_TYPE
{
  OUST128 = 1,
};

class Preprocess
{
public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  void extract_lidar_param();
  void process_oust128(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudOuster::Ptr &pcl_out, cv::Mat &ref_img_out);

  // Preprocessed point cloud and reflectivity images
  PointCloudOuster pl_ouster;
  cv::Mat ref_img;

  double blind;
  string metadata_json, calibration_json;

  // Lidar related parameters - read from JSON file
  int height;                        // Horizontal resolution of radar
  int width;                         // Number of radar lines
  vector<int> pixel_shift_by_row;    // Pixel shift of each line of image
  vector<float> azi;                 // Horizontal mechanical bias of laser head
  vector<float> alt;                 // Vertical mechanical bias of laser head
  float lidar_origin_to_beam_origin; // Distance deviation from the center of the laser head to the center of the LiDAR
  float beam_angle_up;               // The angle of the top laser head
  float beam_angle_down;             // The angle of the lowest laser head

private:
  void extract_ouster_param();
  void extract_calibration_param();
  void OUST128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
};

#endif