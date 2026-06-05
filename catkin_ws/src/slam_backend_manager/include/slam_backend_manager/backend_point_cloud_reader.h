#pragma once

#include <string>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

#include "slam_backend_manager/backend_candidates.h"

namespace slam_backend_manager {

struct BackendPointCloudReadResult {
  bool ok = false;
  std::string reason;
  std::vector<Point3> points;
};

BackendPointCloudReadResult readBackendPointCloud2(
    const sensor_msgs::PointCloud2& cloud);

}  // namespace slam_backend_manager
