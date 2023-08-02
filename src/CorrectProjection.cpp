#include <ros/ros.h>
#include <Eigen/Core>
#include <omp.h>
#include <mutex>
#include <condition_variable>
#include <csignal>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

#include "common_lib.h"

mutex mtx_buffer;
condition_variable sig_buffer;

vector<float> azi; // Horizontal mechanical bias of laser head
vector<float> alt; // Vertical mechanical bias of laser head
vector<int> num_perring;

string lid_topic, metadata_path, out_path;
int selnum_perring; // The number of points selected in each ring for calculating bias
double blind;
int frame_skip;
bool flg_exit = false;

deque<PointCloudOuster::Ptr> lidar_buffer;

// Lidar related parameters - read from JSON file
int height;                        // Horizontal resolution of radar
int width;                         // Number of radar lines
vector<int> pixel_shift_by_row;    // Pixel shift of each line of image
float lidar_origin_to_beam_origin; // Distance deviation from the center of the laser head to the center of the LiDAR
float beam_angle_up;               // The angle of the top laser head
float beam_angle_down;             // The angle of the lowest laser head

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

int frame_num = 0;
void pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg_in)
{
    mtx_buffer.lock();
    PointCloudOuster::Ptr ptr(new PointCloudOuster());
    frame_num++;
    if ((frame_num % frame_skip) == 0)
    {
        pcl::fromROSMsg(*msg_in, *ptr);
        lidar_buffer.push_back(ptr);
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

// Read the parameters of ouster LiDAR
void extract_ouster_param(string metadata_json)
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

// Lidar projection related parameters
const float PI_INV = 1 / M_PI;
float BEAM_ANGLE_INV;
// Project 3D points (sensor_link coordinate system, Z-axis needs to be subtracted by 0.03618) into the LiDAR image
V2D project3Dto2D(V3D point3d)
{
    V2D point2d;
    // u
    point2d[0] = 0.5 * width * (1 - PI_INV * atan2f(point3d[1], point3d[0]));
    if (point2d[0] > width)
        point2d[0] -= width;
    if (point2d[0] < 0)
        point2d[0] += width;
    // v
    float range = sqrtf(point3d[0] * point3d[0] + point3d[1] * point3d[1] + point3d[2] * point3d[2]) - lidar_origin_to_beam_origin;
    point2d[1] = (beam_angle_up - asinf(point3d[2] / range)) * BEAM_ANGLE_INV * height;
    return point2d;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CorrectProjection");
    ros::NodeHandle nh;

    nh.param<int>("frame_skip", frame_skip, 10);
    nh.param<double>("blind", blind, 1.0);
    nh.param<int>("selnum_perring", selnum_perring, 20000);
    nh.param<string>("lid_topic", lid_topic, "/os_cloud_node/points");
    nh.param<string>("metadata_path", metadata_path, "config/metadata_RILIO.json");
    nh.param<string>("out_path", out_path, "config/lidar_calibration_RILIO.json");

    ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 200000, pcl_cbk);
    extract_ouster_param(metadata_path);
    BEAM_ANGLE_INV = 1 / (beam_angle_up + beam_angle_down);

    azi.resize(height);
    alt.resize(height);
    num_perring.resize(height);
    fill(azi.begin(), azi.end(), 0);
    fill(alt.begin(), alt.end(), 0);
    fill(num_perring.begin(), num_perring.end(), 1);

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit)
            break;
        ros::spinOnce();
        if (lidar_buffer.empty())
            continue;
        PointCloudOuster::Ptr pl_ouster;
        pl_ouster = lidar_buffer.front();
        for (int v = 0; v < height; v++)
        {
            if (num_perring[v] >= selnum_perring)
                continue;
            for (int u = 0; u < width; u++)
            {
                if (num_perring[v] >= selnum_perring)
                    break;
                int uu = (u + width - pixel_shift_by_row[v]) % width;
                auto &pt = pl_ouster->points[v * width + uu];
                double range = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                if (range < blind)
                    continue;
                V2D p2d_pro = project3Dto2D(V3D(pt.x, pt.y, pt.z));
                int azi_bias_temp = u - p2d_pro[0];
                if (azi_bias_temp < -floor(width / 1000) * 1000)
                    azi_bias_temp = azi_bias_temp + width;
                if (azi_bias_temp > floor(width / 1000) * 1000)
                    azi_bias_temp = azi_bias_temp - width;
                int alt_bias_temp = v - p2d_pro[1];

                azi[v] = azi[v] + (azi_bias_temp - azi[v]) / num_perring[v];
                alt[v] = alt[v] + (alt_bias_temp - alt[v]) / num_perring[v];

                num_perring[v]++;
            }
        }

        lidar_buffer.pop_front();

        int total_num = 0;
        for (auto p : num_perring)
        {
            total_num += p;
        }
        cout << "Processing:" << total_num * 100 / (selnum_perring * height) << "%" << endl;
        if (total_num >= selnum_perring * height)
            flg_exit = true;
    }

    Json::Value root;
    Json::StyledWriter sw;
    for (auto alt_data : alt)
        root["alt"].append(alt_data);
    for (auto azi_data : azi)
        root["azi"].append(azi_data);
    ofstream os;
    os.open(ROOT_DIR + out_path, std::ios::out | std::ios::trunc);

    if (!os.is_open())
        cout << "Can not open out_path" << endl;
    os << sw.write(root);
    os.close();

    cout << "Done! File saved to" << ROOT_DIR + out_path << endl;

    return 0;
}