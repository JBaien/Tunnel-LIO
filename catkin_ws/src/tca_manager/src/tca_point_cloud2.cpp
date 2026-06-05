#include "tca_manager/tca_point_cloud2.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include <sensor_msgs/PointField.h>

namespace tca_manager {
namespace {

int uniqueFieldIndex(const sensor_msgs::PointCloud2& cloud, const std::string& name) {
  int result = -1;
  std::size_t matches = 0;
  for (std::size_t index = 0; index < cloud.fields.size(); ++index) {
    if (cloud.fields[index].name == name) {
      result = static_cast<int>(index);
      ++matches;
    }
  }
  return matches == 1 ? result : -1;
}

std::size_t datatypeSize(const std::uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      return 4;
    case sensor_msgs::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

bool readFieldAsDouble(const std::uint8_t* point_data,
                       const sensor_msgs::PointField& field,
                       double* value) {
  const std::uint8_t* source = point_data + field.offset;
  switch (field.datatype) {
    case sensor_msgs::PointField::INT8: {
      int8_t raw = 0;
      std::memcpy(&raw, source, sizeof(raw));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::UINT8: {
      uint8_t raw = 0;
      std::memcpy(&raw, source, sizeof(raw));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::INT16: {
      int16_t raw = 0;
      std::memcpy(&raw, source, sizeof(raw));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::UINT16: {
      uint16_t raw = 0;
      std::memcpy(&raw, source, sizeof(raw));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::INT32: {
      int32_t raw = 0;
      std::memcpy(&raw, source, sizeof(raw));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::UINT32: {
      uint32_t raw = 0;
      std::memcpy(&raw, source, sizeof(raw));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::FLOAT32: {
      float raw = 0.0F;
      std::memcpy(&raw, source, sizeof(raw));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::FLOAT64: {
      double raw = 0.0;
      std::memcpy(&raw, source, sizeof(raw));
      *value = raw;
      return true;
    }
    default:
      return false;
  }
}

TcaPointCloudReadResult invalidResult(const std::string& error) {
  TcaPointCloudReadResult result;
  result.valid = false;
  result.error = error;
  return result;
}

bool validRequiredField(const sensor_msgs::PointField& field, const std::uint32_t point_step) {
  const std::size_t size = datatypeSize(field.datatype);
  return field.count == 1 &&
         size > 0 &&
         field.offset <= point_step &&
         size <= static_cast<std::size_t>(point_step - field.offset);
}

}  // namespace

TcaPointCloudReadResult readTcaPointCloud2(const sensor_msgs::PointCloud2& cloud) {
  const int x_index = uniqueFieldIndex(cloud, "x");
  const int y_index = uniqueFieldIndex(cloud, "y");
  const int z_index = uniqueFieldIndex(cloud, "z");
  const int intensity_index = uniqueFieldIndex(cloud, "intensity");
  if (x_index < 0 || y_index < 0 || z_index < 0 || intensity_index < 0) {
    return invalidResult("missing or duplicate required xyzi field");
  }

  const sensor_msgs::PointField& x_field = cloud.fields[x_index];
  const sensor_msgs::PointField& y_field = cloud.fields[y_index];
  const sensor_msgs::PointField& z_field = cloud.fields[z_index];
  const sensor_msgs::PointField& intensity_field = cloud.fields[intensity_index];
  if (!validRequiredField(x_field, cloud.point_step) ||
      !validRequiredField(y_field, cloud.point_step) ||
      !validRequiredField(z_field, cloud.point_step) ||
      !validRequiredField(intensity_field, cloud.point_step)) {
    return invalidResult("invalid required xyzi field layout");
  }

  if (cloud.point_step == 0 ||
      cloud.row_step < cloud.point_step * cloud.width) {
    return invalidResult("invalid point cloud stride");
  }

  if (cloud.height > 0 &&
      cloud.width > std::numeric_limits<std::uint32_t>::max() / cloud.height) {
    return invalidResult("point cloud dimensions overflow");
  }
  const std::size_t count = static_cast<std::size_t>(cloud.width) * cloud.height;
  if (count == 0) {
    TcaPointCloudReadResult result;
    result.valid = true;
    return result;
  }

  const std::size_t required_size =
      static_cast<std::size_t>(cloud.row_step) * (cloud.height - 1) +
      static_cast<std::size_t>(cloud.point_step) * cloud.width;
  if (cloud.data.size() < required_size) {
    return invalidResult("truncated point cloud data");
  }

  TcaPointCloudReadResult result;
  result.valid = true;
  result.points.reserve(count);
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    for (std::uint32_t column = 0; column < cloud.width; ++column) {
      const std::size_t offset =
          static_cast<std::size_t>(row) * cloud.row_step +
          static_cast<std::size_t>(column) * cloud.point_step;
      const std::uint8_t* point_data = &cloud.data[offset];
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double intensity = 0.0;
      if (!readFieldAsDouble(point_data, x_field, &x) ||
          !readFieldAsDouble(point_data, y_field, &y) ||
          !readFieldAsDouble(point_data, z_field, &z) ||
          !readFieldAsDouble(point_data, intensity_field, &intensity) ||
          !std::isfinite(x) ||
          !std::isfinite(y) ||
          !std::isfinite(z) ||
          !std::isfinite(intensity) ||
          intensity < 0.0) {
        return invalidResult("invalid point cloud xyzi sample");
      }
      result.points.push_back(PointXYZI{x, y, z, intensity});
    }
  }
  return result;
}

}  // namespace tca_manager
