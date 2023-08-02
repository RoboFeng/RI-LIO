#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : blind(0.5) {}

Preprocess::~Preprocess() {}

// Call after setting the json file path (param:metadata_json)
void Preprocess::extract_lidar_param()
{
  extract_ouster_param();
  extract_calibration_param();
}

// Read the parameters of ouster LiDAR
void Preprocess::extract_ouster_param()
{
  Json::Value root;
  Json::Reader reader;
  Json::Value array;
  
  std::ifstream ifs(ROOT_DIR + metadata_json); // open file
  if (!reader.parse(ifs, root))
  {
    ROS_ERROR("Can NOT Find Ouster LiDAR Metadata File!\n");
  }
  else
  {
    // Read the basic parameters of LiDAR
    height = root["data_format"]["pixels_per_column"].asInt();
    width = root["data_format"]["columns_per_frame"].asInt();
    beam_angle_up = root["beam_altitude_angles"][0].asFloat();
    beam_angle_up = beam_angle_up * M_PI / 180;
    beam_angle_down = -root["beam_altitude_angles"][height - 1].asFloat();
    beam_angle_down = beam_angle_down * M_PI / 180;

    // Read the pixel_shift_by_row
    array = root["data_format"]["pixel_shift_by_row"];
    pixel_shift_by_row.resize(array.size());
    for (int i = 0; i < array.size(); i++)
    {
      pixel_shift_by_row[i] = array[i].asInt();
    }

    // Read the mechanical deviation of the ideal model of the laser head (unit in mm in the JSON file)
    lidar_origin_to_beam_origin = root["lidar_origin_to_beam_origin_mm"].asFloat() * 0.001;
  }
}

// Read the internal parameters of LiDAR projection
void Preprocess::extract_calibration_param()
{
  Json::Value root;
  Json::Reader reader;
  Json::Value array;

  std::ifstream ifs(ROOT_DIR + calibration_json); // open file
  if (!reader.parse(ifs, root))
  {
    ROS_ERROR("Can NOT Find Lidar Calibration File!\n");
  }
  else
  {
    array = root["azi"];
    azi.resize(array.size());
    for (int i = 0; i < array.size(); i++)
    {
      azi[i] = array[i].asFloat();
    }
    array = root["alt"];
    alt.resize(array.size());
    for (int i = 0; i < array.size(); i++)
    {
      alt[i] = array[i].asFloat();
    }
  }
}

void Preprocess::process_oust128(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudOuster::Ptr &pcl_out, cv::Mat &ref_img_out)
{
  OUST128_handler(msg);
  *pcl_out = pl_ouster;
  ref_img_out = ref_img.clone();
}

void Preprocess::OUST128_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_ouster.clear();
  pcl::fromROSMsg(*msg, pl_ouster);
  int plsize = pl_ouster.size();
  if (pl_ouster.points.size() != height * width)
  {
    ROS_ERROR("Invaild points size!");
    return;
  }
  ref_img = cv::Mat(height, width, CV_8UC1, cv::Scalar(0));
  for (int v = 0; v < height; v++)
    for (int u = 0; u < width; u++)
    {
      const int uu = (u + width - pixel_shift_by_row[v]) % width;
      const auto &pt = pl_ouster.points[v * width + uu];
      ref_img.at<uint8_t>(v, u) = pt.reflectivity;
    }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "ouster";
  output.header.stamp = ct;
}
