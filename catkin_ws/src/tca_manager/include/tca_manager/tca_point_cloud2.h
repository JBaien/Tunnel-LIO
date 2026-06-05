#pragma once

#include <string>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

#include "tca_manager/tca_detection.h"

namespace tca_manager {

struct TcaPointCloudReadResult {
  bool valid = false;
  std::string error;
  std::vector<PointXYZI> points;
};

TcaPointCloudReadResult readTcaPointCloud2(const sensor_msgs::PointCloud2& cloud);

}  // namespace tca_manager
