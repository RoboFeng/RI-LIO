#ifndef MYPOINT_TYPE_H
#define MYPOINT_TYPE_H
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace ouster_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

namespace undistort
{

    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint8_t ring;
        uint32_t t;
        float qw;
        float qx;
        float qy;
        float qz;
        float tx;
        float ty;
        float tz;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(undistort::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (float, intensity, intensity)
    (std::uint8_t, ring, ring)
    (std::uint32_t, t, t)
    (float, qw, qw)
    (float, qx, qx)
    (float, qy, qy)
    (float, qz, qz)
    (float, tx, tx)
    (float, ty, ty)
    (float, tz, tz)
)

#endif