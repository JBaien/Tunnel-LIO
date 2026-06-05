#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "lio_preprocess/filtering.h"

namespace lio_preprocess {

struct PointCloud2ReadResult {
  bool ok = false;
  std::string reason;
  std::vector<std::string> field_names;
  std::vector<PointTuple> points;
  std::vector<std::vector<uint8_t> > point_bytes;
  int x_index = -1;
  int y_index = -1;
  int z_index = -1;
  FilterStats stats;
};

bool readFilteredPointCloud2(
    const sensor_msgs::PointCloud2& cloud,
    const PointFilterConfig& config,
    PointCloud2ReadResult* result);

bool writeDoubleToPointField(uint8_t* point, const sensor_msgs::PointField& field, double value);

}  // namespace lio_preprocess
