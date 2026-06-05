#include "slam_backend_manager/backend_point_cloud_reader.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include <sensor_msgs/PointField.h>

namespace slam_backend_manager {
namespace {

BackendPointCloudReadResult fail(const std::string& reason) {
  BackendPointCloudReadResult result;
  result.ok = false;
  result.reason = reason;
  return result;
}

int datatypeSize(const std::uint8_t datatype) {
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

bool hasDuplicateFieldName(const sensor_msgs::PointCloud2& cloud) {
  for (std::size_t left = 0; left < cloud.fields.size(); ++left) {
    if (cloud.fields[left].name.empty()) {
      return true;
    }
    for (std::size_t right = left + 1; right < cloud.fields.size(); ++right) {
      if (cloud.fields[left].name == cloud.fields[right].name) {
        return true;
      }
    }
  }
  return false;
}

int fieldIndex(const sensor_msgs::PointCloud2& cloud, const std::string& name) {
  for (std::size_t index = 0; index < cloud.fields.size(); ++index) {
    if (cloud.fields[index].name == name) {
      return static_cast<int>(index);
    }
  }
  return -1;
}

bool safeMultiply(const std::size_t left,
                  const std::size_t right,
                  std::size_t* value) {
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {
    return false;
  }
  *value = left * right;
  return true;
}

bool validFieldLayout(const sensor_msgs::PointCloud2& cloud) {
  if (cloud.point_step == 0) {
    return false;
  }
  for (const sensor_msgs::PointField& field : cloud.fields) {
    const int scalar_size = datatypeSize(field.datatype);
    if (scalar_size <= 0 || field.count != 1) {
      return false;
    }
    const std::size_t offset = static_cast<std::size_t>(field.offset);
    if (offset > static_cast<std::size_t>(cloud.point_step) ||
        static_cast<std::size_t>(scalar_size) >
            static_cast<std::size_t>(cloud.point_step) - offset) {
      return false;
    }
  }
  return true;
}

bool validDataLayout(const sensor_msgs::PointCloud2& cloud) {
  std::size_t row_points_bytes = 0;
  if (!safeMultiply(static_cast<std::size_t>(cloud.width),
                    static_cast<std::size_t>(cloud.point_step),
                    &row_points_bytes)) {
    return false;
  }
  if (cloud.row_step < row_points_bytes) {
    return false;
  }
  std::size_t required_data_bytes = 0;
  if (!safeMultiply(static_cast<std::size_t>(cloud.row_step),
                    static_cast<std::size_t>(cloud.height),
                    &required_data_bytes)) {
    return false;
  }
  return cloud.data.size() >= required_data_bytes;
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

}  // namespace

BackendPointCloudReadResult readBackendPointCloud2(
    const sensor_msgs::PointCloud2& cloud) {
  if (hasDuplicateFieldName(cloud)) {
    return fail("duplicate_field");
  }
  const int x_index = fieldIndex(cloud, "x");
  const int y_index = fieldIndex(cloud, "y");
  const int z_index = fieldIndex(cloud, "z");
  const int intensity_index = fieldIndex(cloud, "intensity");
  if (x_index < 0 || y_index < 0 || z_index < 0) {
    return fail("missing_required_field");
  }
  if (!validFieldLayout(cloud)) {
    return fail("invalid_field_layout");
  }
  if (!validDataLayout(cloud)) {
    return fail("truncated_data");
  }
  const std::size_t point_count =
      static_cast<std::size_t>(cloud.width) * cloud.height;
  if (point_count == 0) {
    return fail("empty_cloud");
  }

  BackendPointCloudReadResult result;
  result.ok = true;
  result.points.reserve(point_count);
  for (std::size_t row = 0; row < cloud.height; ++row) {
    const std::size_t row_offset = row * static_cast<std::size_t>(cloud.row_step);
    for (std::size_t column = 0; column < cloud.width; ++column) {
      const std::size_t point_offset =
          row_offset + column * static_cast<std::size_t>(cloud.point_step);
      const std::uint8_t* point_data = &cloud.data[point_offset];
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double intensity = 0.0;
      if (!readFieldAsDouble(point_data, cloud.fields[x_index], &x) ||
          !readFieldAsDouble(point_data, cloud.fields[y_index], &y) ||
          !readFieldAsDouble(point_data, cloud.fields[z_index], &z) ||
          !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return fail("invalid_point");
      }
      if (intensity_index >= 0) {
        if (!readFieldAsDouble(point_data, cloud.fields[intensity_index],
                               &intensity) ||
            !std::isfinite(intensity) || intensity < 0.0) {
          return fail("invalid_intensity");
        }
      }
      result.points.push_back(Point3{x, y, z, intensity});
    }
  }
  return result;
}

}  // namespace slam_backend_manager
