#include "lio_preprocess/point_cloud2_reader.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <set>

namespace lio_preprocess {

namespace {

int fieldElementSize(uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::FLOAT64:
      return 8;
    case sensor_msgs::PointField::FLOAT32:
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
      return 4;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2;
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1;
    default:
      return 0;
  }
}

bool readFieldAsDouble(const uint8_t* point, const sensor_msgs::PointField& field, double* value) {
  const uint8_t* ptr = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::PointField::FLOAT32: {
      float raw = 0.0f;
      std::memcpy(&raw, ptr, sizeof(float));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::FLOAT64: {
      double raw = 0.0;
      std::memcpy(&raw, ptr, sizeof(double));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::INT32: {
      int32_t raw = 0;
      std::memcpy(&raw, ptr, sizeof(int32_t));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::UINT32: {
      uint32_t raw = 0;
      std::memcpy(&raw, ptr, sizeof(uint32_t));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::INT16: {
      int16_t raw = 0;
      std::memcpy(&raw, ptr, sizeof(int16_t));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::UINT16: {
      uint16_t raw = 0;
      std::memcpy(&raw, ptr, sizeof(uint16_t));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::INT8: {
      int8_t raw = 0;
      std::memcpy(&raw, ptr, sizeof(int8_t));
      *value = raw;
      return true;
    }
    case sensor_msgs::PointField::UINT8: {
      uint8_t raw = 0;
      std::memcpy(&raw, ptr, sizeof(uint8_t));
      *value = raw;
      return true;
    }
    default:
      return false;
  }
}

bool fail(PointCloud2ReadResult* result, const std::string& reason) {
  if (result != nullptr) {
    result->ok = false;
    result->reason = reason;
    result->points.clear();
    result->point_bytes.clear();
    result->stats = FilterStats();
  }
  return false;
}

bool validFieldLayout(const sensor_msgs::PointField& field, uint32_t point_step) {
  const int size = fieldElementSize(field.datatype);
  if (field.name.empty() || field.count != 1 || size <= 0) {
    return false;
  }
  const uint64_t end = static_cast<uint64_t>(field.offset) + static_cast<uint64_t>(size);
  return end <= static_cast<uint64_t>(point_step);
}

bool writableCoordinateField(const sensor_msgs::PointField& field) {
  return field.datatype == sensor_msgs::PointField::FLOAT32 ||
         field.datatype == sensor_msgs::PointField::FLOAT64;
}

int requiredFieldIndex(const sensor_msgs::PointCloud2& cloud, const std::string& name, bool* duplicate) {
  int result = -1;
  int matches = 0;
  for (std::size_t i = 0; i < cloud.fields.size(); ++i) {
    if (cloud.fields[i].name == name) {
      result = static_cast<int>(i);
      ++matches;
    }
  }
  *duplicate = matches > 1;
  return matches == 1 ? result : -1;
}

std::vector<std::string> fieldNames(const sensor_msgs::PointCloud2& cloud) {
  std::vector<std::string> names;
  for (std::vector<sensor_msgs::PointField>::const_iterator it = cloud.fields.begin(); it != cloud.fields.end(); ++it) {
    names.push_back(it->name);
  }
  return names;
}

bool hasDuplicateFieldName(const sensor_msgs::PointCloud2& cloud) {
  std::set<std::string> names;
  for (std::vector<sensor_msgs::PointField>::const_iterator it = cloud.fields.begin(); it != cloud.fields.end(); ++it) {
    if (!names.insert(it->name).second) {
      return true;
    }
  }
  return false;
}

bool dataLayoutCoversCloud(const sensor_msgs::PointCloud2& cloud) {
  if (cloud.height == 0 || cloud.width == 0) {
    return true;
  }
  if (cloud.point_step == 0) {
    return false;
  }
  const uint64_t min_row_step = static_cast<uint64_t>(cloud.point_step) * static_cast<uint64_t>(cloud.width);
  if (static_cast<uint64_t>(cloud.row_step) < min_row_step) {
    return false;
  }
  const uint64_t required_bytes = static_cast<uint64_t>(cloud.row_step) * static_cast<uint64_t>(cloud.height);
  return required_bytes <= static_cast<uint64_t>(cloud.data.size());
}

}  // namespace

bool readFilteredPointCloud2(
    const sensor_msgs::PointCloud2& cloud,
    const PointFilterConfig& config,
    PointCloud2ReadResult* result) {
  if (result == nullptr) {
    return false;
  }
  *result = PointCloud2ReadResult();
  result->field_names = fieldNames(cloud);

  bool duplicate = false;
  const int x_index = requiredFieldIndex(cloud, "x", &duplicate);
  if (duplicate) {
    return fail(result, "duplicate_required_field");
  }
  const int y_index = requiredFieldIndex(cloud, "y", &duplicate);
  if (duplicate) {
    return fail(result, "duplicate_required_field");
  }
  const int z_index = requiredFieldIndex(cloud, "z", &duplicate);
  if (duplicate) {
    return fail(result, "duplicate_required_field");
  }
  if (x_index < 0 || y_index < 0 || z_index < 0) {
    return fail(result, "missing_required_field");
  }

  for (std::vector<sensor_msgs::PointField>::const_iterator it = cloud.fields.begin(); it != cloud.fields.end(); ++it) {
    if (!validFieldLayout(*it, cloud.point_step)) {
      return fail(result, "unreadable_field");
    }
  }
  if (hasDuplicateFieldName(cloud)) {
    return fail(result, "duplicate_field");
  }
  if (!writableCoordinateField(cloud.fields[x_index]) ||
      !writableCoordinateField(cloud.fields[y_index]) ||
      !writableCoordinateField(cloud.fields[z_index])) {
    return fail(result, "unwritable_required_field");
  }
  if (!dataLayoutCoversCloud(cloud)) {
    return fail(result, "truncated_data");
  }

  result->ok = true;
  result->x_index = x_index;
  result->y_index = y_index;
  result->z_index = z_index;
  result->points.reserve(static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height));
  result->point_bytes.reserve(result->points.capacity());

  for (uint32_t row = 0; row < cloud.height; ++row) {
    for (uint32_t col = 0; col < cloud.width; ++col) {
      const std::size_t point_offset =
          static_cast<std::size_t>(row) * cloud.row_step + static_cast<std::size_t>(col) * cloud.point_step;
      const uint8_t* point = &cloud.data[point_offset];
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      readFieldAsDouble(point, cloud.fields[x_index], &x);
      readFieldAsDouble(point, cloud.fields[y_index], &y);
      readFieldAsDouble(point, cloud.fields[z_index], &z);
      ++result->stats.input_points;
      const KeepDecision keep = shouldKeepXYZ(x, y, z, config);
      if (!keep.keep) {
        if (keep.reason == "nan") {
          ++result->stats.dropped_nan;
        } else if (keep.reason == "range") {
          ++result->stats.dropped_range;
        } else if (keep.reason == "body") {
          ++result->stats.dropped_body;
        }
        continue;
      }

      PointTuple tuple;
      tuple.reserve(cloud.fields.size());
      for (std::vector<sensor_msgs::PointField>::const_iterator field = cloud.fields.begin(); field != cloud.fields.end(); ++field) {
        double value = 0.0;
        readFieldAsDouble(point, *field, &value);
        if (!std::isfinite(value)) {
          return fail(result, "nonfinite_field");
        }
        tuple.push_back(value);
      }
      result->points.push_back(tuple);
      result->point_bytes.push_back(std::vector<uint8_t>(point, point + cloud.point_step));
      ++result->stats.output_points;
    }
  }
  return true;
}

bool writeDoubleToPointField(uint8_t* point, const sensor_msgs::PointField& field, double value) {
  if (point == nullptr || !std::isfinite(value)) {
    return false;
  }
  uint8_t* ptr = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::PointField::FLOAT32: {
      float raw = static_cast<float>(value);
      std::memcpy(ptr, &raw, sizeof(float));
      return true;
    }
    case sensor_msgs::PointField::FLOAT64: {
      std::memcpy(ptr, &value, sizeof(double));
      return true;
    }
    default:
      return false;
  }
}

}  // namespace lio_preprocess
