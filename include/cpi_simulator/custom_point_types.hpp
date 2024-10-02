#ifndef CUSTOM_POINT_TYPES_HPP
#define CUSTOM_POINT_TYPES_HPP

#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <Eigen/StdVector>

// Define the custom point structure
struct PointXYZIRADT {
  PCL_ADD_POINT4D;                 // Macro for x, y, z, padding
  float intensity;                 // Intensity
  uint16_t ring;                   // Ring number
  float azimuth;                   // Azimuth angle
  float distance;                  // Distance
  uint8_t return_type;             // Return type
  double time_stamp;               // Timestamp
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure proper alignment for dynamic
                                   // allocation
      inline PointXYZIRADT() {
    x = 0;
    y = 0;
    z = 0;
    intensity = 0;
    ring = 0;
    azimuth = 0;
    distance = 0;
    return_type = 0;
    time_stamp = 0;
  }
  inline PointXYZIRADT(const PointXYZIRADT& p) {
    x = p.x;
    y = p.y;
    z = p.z;
    intensity = p.intensity;
    ring = p.ring;
    azimuth = p.azimuth;
    distance = p.distance;
    return_type = p.return_type;
    time_stamp = p.time_stamp;
  }
} EIGEN_ALIGN16;

// Register the custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRADT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        uint16_t, ring, ring)(float, azimuth, azimuth)(
        float, distance, distance)(uint8_t, return_type,
                                   return_type)(double, time_stamp, time_stamp))

#endif
