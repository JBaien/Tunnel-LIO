#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <limits>

#include <boost/filesystem.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "tca_manager/tca_detection.h"
#include "tca_manager/tca_point_cloud2.h"

namespace tca_manager {
namespace {

sensor_msgs::PointField field(const std::string& name,
                              const std::uint32_t offset,
                              const std::uint8_t datatype) {
  sensor_msgs::PointField result;
  result.name = name;
  result.offset = offset;
  result.datatype = datatype;
  result.count = 1;
  return result;
}

void writeFloat(std::vector<std::uint8_t>* data, const std::size_t offset, const float value) {
  std::memcpy(&(*data)[offset], &value, sizeof(value));
}

sensor_msgs::PointCloud2 xyziCloud(const std::vector<std::array<float, 4> >& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields = {
      field("x", 0, sensor_msgs::PointField::FLOAT32),
      field("y", 4, sensor_msgs::PointField::FLOAT32),
      field("z", 8, sensor_msgs::PointField::FLOAT32),
      field("intensity", 12, sensor_msgs::PointField::FLOAT32),
  };
  cloud.data.assign(cloud.row_step, 0);

  for (std::size_t index = 0; index < points.size(); ++index) {
    const std::size_t offset = index * cloud.point_step;
    writeFloat(&cloud.data, offset + 0, points[index][0]);
    writeFloat(&cloud.data, offset + 4, points[index][1]);
    writeFloat(&cloud.data, offset + 8, points[index][2]);
    writeFloat(&cloud.data, offset + 12, points[index][3]);
  }
  return cloud;
}

TEST(TcaDetection, DetectsReflectiveTargetCentroid) {
  const std::vector<PointXYZI> points = {
      {0.0, 0.0, 0.0, 10.0},
      {1.0, 0.0, 0.0, 230.0},
      {1.1, 0.0, 0.0, 240.0},
      {1.0, 0.1, 0.0, 250.0},
  };
  TcaDetectionConfig config;
  config.min_reflective_points = 3;
  config.intensity_threshold = 200.0;

  const std::vector<TcaDetection> detections = detectReflectiveTargets(points, config);
  ASSERT_EQ(1u, detections.size());
  EXPECT_NEAR(1.0333333333, detections[0].center.x, 1e-9);
  EXPECT_NEAR(0.0333333333, detections[0].center.y, 1e-9);
}

TEST(TcaDetection, BuildsContextSignatureFromNearbyGeometry) {
  const std::vector<PointXYZI> points = {
      {0.0, 0.0, 0.0, 250.0},
      {1.0, 0.0, 0.0, 20.0},
      {3.0, 0.0, 0.0, 20.0},
      {9.0, 0.0, 0.0, 20.0},
  };

  const ContextSignature signature =
      buildContextSignature(points, Point3{0.0, 0.0, 0.0}, std::vector<double>{0.0, 2.0, 4.0, 8.0});

  ASSERT_EQ(3u, signature.ring_counts.size());
  EXPECT_EQ(1, signature.ring_counts[0]);
  EXPECT_EQ(1, signature.ring_counts[1]);
  EXPECT_EQ(0, signature.ring_counts[2]);
}

TEST(TcaDetection, InvalidContextConfigFailsClosed) {
  const std::vector<PointXYZI> points = {
      {0.0, 0.0, 0.0, 250.0},
      {1.0, 0.0, 0.0, 230.0},
      {1.1, 0.0, 0.0, 240.0},
      {1.0, 0.1, 0.0, 250.0},
      {3.0, 0.0, 0.0, 20.0},
  };

  EXPECT_TRUE(buildContextSignature(points, Point3{0.0, 0.0, 0.0},
                                    std::vector<double>{0.0, 3.0, 2.0})
                  .ring_counts.empty());
  EXPECT_TRUE(buildContextSignature(points, Point3{0.0, 0.0, 0.0},
                                    std::vector<double>{0.0,
                                                        std::numeric_limits<double>::quiet_NaN(),
                                                        4.0})
                  .ring_counts.empty());

  TcaDetectionConfig config;
  config.min_reflective_points = 3;
  config.intensity_threshold = 200.0;
  config.context_ring_edges = {0.0, 3.0, 2.0};

  EXPECT_TRUE(detectReflectiveTargets(points, config).empty());
}

TEST(TcaDetection, DetectRejectsNonFinitePointCoordinates) {
  const std::vector<PointXYZI> points = {
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 250.0},
      {1.0, 0.0, 0.0, 230.0},
      {1.1, 0.0, 0.0, 240.0},
      {1.0, 0.1, 0.0, 250.0},
  };
  TcaDetectionConfig config;
  config.min_reflective_points = 3;
  config.intensity_threshold = 200.0;

  EXPECT_TRUE(detectReflectiveTargets(points, config).empty());
}

TEST(TcaDetection, DetectRejectsNonFinitePointIntensity) {
  const std::vector<PointXYZI> points = {
      {0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN()},
      {1.0, 0.0, 0.0, 230.0},
      {1.1, 0.0, 0.0, 240.0},
      {1.0, 0.1, 0.0, 250.0},
  };
  TcaDetectionConfig config;
  config.min_reflective_points = 3;
  config.intensity_threshold = 200.0;

  EXPECT_TRUE(detectReflectiveTargets(points, config).empty());
}

TEST(TcaDetection, DetectRejectsNegativePointIntensity) {
  const std::vector<PointXYZI> points = {
      {0.0, 0.0, 0.0, -1.0},
      {1.0, 0.0, 0.0, 230.0},
      {1.1, 0.0, 0.0, 240.0},
      {1.0, 0.1, 0.0, 250.0},
  };
  TcaDetectionConfig config;
  config.min_reflective_points = 3;
  config.intensity_threshold = 200.0;

  EXPECT_TRUE(detectReflectiveTargets(points, config).empty());
}

TEST(TcaDetection, DetectRejectsNegativeIntensityThreshold) {
  const std::vector<PointXYZI> points = {
      {0.0, 0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 1.0},
      {1.1, 0.0, 0.0, 2.0},
      {1.0, 0.1, 0.0, 3.0},
  };
  TcaDetectionConfig config;
  config.min_reflective_points = 3;
  config.intensity_threshold = -1.0;

  EXPECT_TRUE(detectReflectiveTargets(points, config).empty());
}

TEST(TcaDetection, ContextSignatureRejectsNonFiniteInputPoint) {
  const std::vector<PointXYZI> points = {
      {0.0, 0.0, 0.0, 250.0},
      {std::numeric_limits<double>::infinity(), 0.0, 0.0, 20.0},
      {1.0, 0.0, 0.0, 20.0},
  };

  const ContextSignature signature =
      buildContextSignature(points, Point3{0.0, 0.0, 0.0}, std::vector<double>{0.0, 2.0, 4.0});

  EXPECT_TRUE(signature.ring_counts.empty());
}

TEST(TcaDetection, ContextSignatureRejectsNegativeInputIntensity) {
  const std::vector<PointXYZI> points = {
      {0.0, 0.0, 0.0, 250.0},
      {1.0, 0.0, 0.0, -1.0},
      {2.0, 0.0, 0.0, 20.0},
  };

  const ContextSignature signature =
      buildContextSignature(points, Point3{0.0, 0.0, 0.0}, std::vector<double>{0.0, 2.0, 4.0});

  EXPECT_TRUE(signature.ring_counts.empty());
}

TEST(TcaDetection, ContextSignatureRejectsEmptyPointCloud) {
  const ContextSignature signature =
      buildContextSignature(std::vector<PointXYZI>{},
                            Point3{0.0, 0.0, 0.0},
                            std::vector<double>{0.0, 2.0, 4.0});

  EXPECT_TRUE(signature.ring_counts.empty());
}

TEST(TcaDetection, ContextSignatureRejectsAllZeroRingCounts) {
  const std::vector<PointXYZI> points = {
      {0.0, 0.0, 0.0, 250.0},
  };

  const ContextSignature signature =
      buildContextSignature(points, Point3{0.0, 0.0, 0.0}, std::vector<double>{0.0, 2.0, 4.0});

  EXPECT_TRUE(signature.ring_counts.empty());
}

TEST(TcaDetection, DetectRejectsCandidateWithoutContextSignature) {
  const std::vector<PointXYZI> points = {
      {1.0, 1.0, 1.0, 250.0},
      {1.0, 1.0, 1.0, 240.0},
      {1.0, 1.0, 1.0, 230.0},
  };
  TcaDetectionConfig config;
  config.min_reflective_points = 3;
  config.intensity_threshold = 200.0;
  config.context_ring_edges = {0.0, 0.5, 1.0};

  EXPECT_TRUE(detectReflectiveTargets(points, config).empty());
}

TEST(TcaPointCloud2Reader, ReadsFiniteXyziCloud) {
  const sensor_msgs::PointCloud2 cloud =
      xyziCloud(std::vector<std::array<float, 4> >{
          {{0.0F, 0.0F, 0.0F, 10.0F}},
          {{1.0F, 2.0F, 3.0F, 240.0F}},
      });

  const TcaPointCloudReadResult result = readTcaPointCloud2(cloud);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(2u, result.points.size());
  EXPECT_DOUBLE_EQ(1.0, result.points[1].x);
  EXPECT_DOUBLE_EQ(240.0, result.points[1].intensity);
}

TEST(TcaPointCloud2Reader, RejectsNonFinitePointInWholeCloud) {
  const sensor_msgs::PointCloud2 cloud =
      xyziCloud(std::vector<std::array<float, 4> >{
          {{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 250.0F}},
          {{1.0F, 0.0F, 0.0F, 240.0F}},
      });

  const TcaPointCloudReadResult result = readTcaPointCloud2(cloud);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.points.empty());
}

TEST(TcaPointCloud2Reader, RejectsNegativeIntensityInWholeCloud) {
  const sensor_msgs::PointCloud2 cloud =
      xyziCloud(std::vector<std::array<float, 4> >{
          {{0.0F, 0.0F, 0.0F, -1.0F}},
          {{1.0F, 0.0F, 0.0F, 240.0F}},
      });

  const TcaPointCloudReadResult result = readTcaPointCloud2(cloud);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.points.empty());
}

TEST(TcaPointCloud2Reader, RejectsUnreadableRequiredField) {
  sensor_msgs::PointCloud2 cloud =
      xyziCloud(std::vector<std::array<float, 4> >{
          {{0.0F, 0.0F, 0.0F, 10.0F}},
      });
  cloud.fields[0].datatype = 0;

  const TcaPointCloudReadResult result = readTcaPointCloud2(cloud);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.points.empty());
}

TEST(TcaPointCloud2Reader, RejectsDuplicateRequiredFieldNames) {
  sensor_msgs::PointCloud2 cloud =
      xyziCloud(std::vector<std::array<float, 4> >{
          {{0.0F, 0.0F, 0.0F, 10.0F}},
      });
  cloud.fields.push_back(field("intensity", 12, sensor_msgs::PointField::FLOAT32));

  const TcaPointCloudReadResult result = readTcaPointCloud2(cloud);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.points.empty());
}

TEST(TcaPointCloud2Reader, RejectsTruncatedPointData) {
  sensor_msgs::PointCloud2 cloud =
      xyziCloud(std::vector<std::array<float, 4> >{
          {{0.0F, 0.0F, 0.0F, 10.0F}},
          {{1.0F, 0.0F, 0.0F, 240.0F}},
      });
  cloud.data.resize(cloud.data.size() - 1);

  const TcaPointCloudReadResult result = readTcaPointCloud2(cloud);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.points.empty());
}

TEST(TcaDetectionCache, EmptyUpdateClearsStaleDetection) {
  TcaDetectionCache cache;
  TcaDetection detection;
  detection.center = Point3{10.0, 0.0, 0.0};
  detection.reflective_points = 3;
  detection.context_signature.ring_counts = {3, 2, 1};

  cache.update(std::vector<TcaDetection>{detection});
  ASSERT_TRUE(cache.hasDetection());
  EXPECT_EQ(1, cache.lastDetectionCount());

  cache.update(std::vector<TcaDetection>{});

  EXPECT_FALSE(cache.hasDetection());
  EXPECT_EQ(0, cache.lastDetectionCount());
}

TEST(TcaDetectionCache, BuildAnchorRequiresCurrentDetection) {
  TcaDetectionCache cache;
  TcaDetection detection;
  detection.center = Point3{10.0, 0.0, 0.0};
  detection.reflective_points = 3;
  detection.context_signature.ring_counts = {3, 2, 1};
  TcaAnchor anchor;

  EXPECT_FALSE(cache.buildAnchor("tca_1", &anchor));

  cache.update(std::vector<TcaDetection>{detection});
  ASSERT_TRUE(cache.buildAnchor("tca_1", &anchor));
  EXPECT_EQ("tca_1", anchor.anchor_id);
  EXPECT_DOUBLE_EQ(10.0, anchor.chainage_m);
  EXPECT_EQ(std::vector<int>({3, 2, 1}), anchor.context_signature);

  cache.clear();

  EXPECT_FALSE(cache.buildAnchor("tca_2", &anchor));
}

TEST(TcaLedger, RejectsAmbiguousAnchorMatch) {
  TcaLedger ledger(0.5, 1.5);
  ledger.addAnchor(TcaAnchor{"a1", 10.0, Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}});
  ledger.addAnchor(TcaAnchor{"a2", 12.0, Point3{0.1, 0.0, 0.0}, std::vector<int>{3, 2, 1}});
  EXPECT_FALSE(ledger.match(Point3{0.02, 0.0, 0.0}, std::vector<int>{3, 2, 1}).has_value);

  TcaLedger separated(0.5, 1.5);
  separated.addAnchor(TcaAnchor{"a1", 10.0, Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}});
  separated.addAnchor(TcaAnchor{"a2", 30.0, Point3{5.0, 0.0, 0.0}, std::vector<int>{0, 1, 3}});
  const OptionalTcaMatch match = separated.match(Point3{0.02, 0.0, 0.0}, std::vector<int>{3, 2, 1});
  ASSERT_TRUE(match.has_value);
  EXPECT_EQ("a1", match.value.anchor_id);
}

TEST(TcaLedger, InvalidLedgerInputsCannotMatch) {
  const TcaAnchor valid_anchor{"a1", 10.0, Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}};

  TcaLedger invalid_score(-0.1, 1.5);
  invalid_score.addAnchor(valid_anchor);
  EXPECT_FALSE(invalid_score.match(Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}).has_value);

  TcaLedger invalid_ratio(0.5, 0.5);
  invalid_ratio.addAnchor(valid_anchor);
  EXPECT_FALSE(invalid_ratio.match(Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}).has_value);

  TcaLedger invalid_anchor_ledger(0.1, 1.5);
  invalid_anchor_ledger.addAnchor(TcaAnchor{"", 10.0, Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}});
  invalid_anchor_ledger.addAnchor(TcaAnchor{"empty_context", 10.0, Point3{0.0, 0.0, 0.0}, std::vector<int>{}});
  EXPECT_FALSE(invalid_anchor_ledger.match(Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}).has_value);
  EXPECT_FALSE(invalid_anchor_ledger.match(Point3{0.0, 0.0, 0.0}, std::vector<int>{}).has_value);
}

TEST(TcaLedger, AllZeroContextCannotMatch) {
  TcaLedger ledger(0.1, 1.5);
  ledger.addAnchor(TcaAnchor{"zero_context",
                             10.0,
                             Point3{0.0, 0.0, 0.0},
                             std::vector<int>{0, 0, 0}});

  EXPECT_FALSE(ledger.match(Point3{0.0, 0.0, 0.0}, std::vector<int>{0, 0, 0}).has_value);
}

TEST(TcaLedger, NegativeChainageAnchorCannotMatch) {
  TcaLedger ledger(0.1, 1.5);
  ledger.addAnchor(TcaAnchor{"negative_chainage",
                             -1.0,
                             Point3{0.0, 0.0, 0.0},
                             std::vector<int>{3, 2, 1}});

  EXPECT_FALSE(ledger.match(Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}).has_value);
}

TEST(TcaLedger, NegativeHeightAnchorCannotMatch) {
  TcaLedger ledger(0.1, 1.5);
  ledger.addAnchor(TcaAnchor{"negative_height",
                             10.0,
                             Point3{0.0, 0.0, 0.0},
                             std::vector<int>{3, 2, 1},
                             "left",
                             -0.1});

  EXPECT_FALSE(ledger.match(Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}).has_value);
}

TEST(TcaLedger, UnsafeAnchorIdCannotMatch) {
  const std::vector<std::string> unsafe_ids = {
      "bad id",
      "bad;score=1",
      "bad\nid",
      ".",
      "..",
  };

  for (const std::string& unsafe_id : unsafe_ids) {
    TcaLedger ledger(0.1, 1.5);
    ledger.addAnchor(TcaAnchor{unsafe_id,
                               10.0,
                               Point3{0.0, 0.0, 0.0},
                               std::vector<int>{3, 2, 1}});

    EXPECT_FALSE(ledger.match(Point3{0.0, 0.0, 0.0}, std::vector<int>{3, 2, 1}).has_value)
        << unsafe_id;
  }
}

TEST(TcaLedger, LoadMalformedJsonFailsClosed) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("tca_ledger_bad_%%%%%%");
  boost::filesystem::create_directories(root);
  const boost::filesystem::path path = root / "ledger.json";

  std::ofstream stream(path.string());
  stream << "{not-json";
  stream.close();

  TcaLedger ledger;
  ledger.addAnchor(TcaAnchor{"existing",
                             1.0,
                             Point3{0.0, 0.0, 0.0},
                             std::vector<int>{1}});

  EXPECT_NO_THROW(ledger.load(path.string()));
  EXPECT_TRUE(ledger.anchors().empty());

  boost::filesystem::remove_all(root);
}

TEST(TcaLedger, LoadSkipsMalformedAnchorAndKeepsValidAnchors) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("tca_ledger_mixed_%%%%%%");
  boost::filesystem::create_directories(root);
  const boost::filesystem::path path = root / "ledger.json";

  std::ofstream stream(path.string());
  stream << "{\n"
         << "  \"anchors\": [\n"
         << "    {\n"
         << "      \"anchor_id\": \"bad\",\n"
         << "      \"chainage_m\": 1.0,\n"
         << "      \"center\": [\"not-a-number\", 0.0, 0.0],\n"
         << "      \"context_signature\": [1],\n"
         << "      \"height_m\": 1.0\n"
         << "    },\n"
         << "    {\n"
         << "      \"anchor_id\": \"good\",\n"
         << "      \"chainage_m\": 12.0,\n"
         << "      \"center\": [1.0, 2.0, 3.0],\n"
         << "      \"context_signature\": [2, 1],\n"
         << "      \"height_m\": 1.2\n"
         << "    }\n"
         << "  ]\n"
         << "}\n";
  stream.close();

  TcaLedger ledger;

  EXPECT_NO_THROW(ledger.load(path.string()));
  ASSERT_EQ(1u, ledger.anchors().size());
  EXPECT_EQ("good", ledger.anchors()[0].anchor_id);

  boost::filesystem::remove_all(root);
}

TEST(TcaLedger, LoadSkipsUnsafeAnchorIds) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("tca_ledger_unsafe_id_%%%%%%");
  boost::filesystem::create_directories(root);
  const boost::filesystem::path path = root / "ledger.json";

  std::ofstream stream(path.string());
  stream << "{\n"
         << "  \"anchors\": [\n"
         << "    {\n"
         << "      \"anchor_id\": \"bad;score=1\",\n"
         << "      \"chainage_m\": 1.0,\n"
         << "      \"center\": [0.0, 0.0, 0.0],\n"
         << "      \"context_signature\": [1],\n"
         << "      \"height_m\": 1.0\n"
         << "    },\n"
         << "    {\n"
         << "      \"anchor_id\": \"..\",\n"
         << "      \"chainage_m\": 2.0,\n"
         << "      \"center\": [0.0, 0.0, 0.0],\n"
         << "      \"context_signature\": [1],\n"
         << "      \"height_m\": 1.0\n"
         << "    },\n"
         << "    {\n"
         << "      \"anchor_id\": \"good_anchor-1.2\",\n"
         << "      \"chainage_m\": 14.0,\n"
         << "      \"center\": [1.0, 2.0, 3.0],\n"
         << "      \"context_signature\": [2, 1],\n"
         << "      \"height_m\": 1.2\n"
         << "    }\n"
         << "  ]\n"
         << "}\n";
  stream.close();

  TcaLedger ledger;

  EXPECT_NO_THROW(ledger.load(path.string()));
  ASSERT_EQ(1u, ledger.anchors().size());
  EXPECT_EQ("good_anchor-1.2", ledger.anchors()[0].anchor_id);

  boost::filesystem::remove_all(root);
}

TEST(TcaLedger, LoadSkipsNegativePhysicalFieldAnchors) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("tca_ledger_negative_fields_%%%%%%");
  boost::filesystem::create_directories(root);
  const boost::filesystem::path path = root / "ledger.json";

  std::ofstream stream(path.string());
  stream << "{\n"
         << "  \"anchors\": [\n"
         << "    {\n"
         << "      \"anchor_id\": \"bad_chainage\",\n"
         << "      \"chainage_m\": -1.0,\n"
         << "      \"center\": [0.0, 0.0, 0.0],\n"
         << "      \"context_signature\": [1],\n"
         << "      \"height_m\": 1.0\n"
         << "    },\n"
         << "    {\n"
         << "      \"anchor_id\": \"bad_height\",\n"
         << "      \"chainage_m\": 1.0,\n"
         << "      \"center\": [0.0, 0.0, 0.0],\n"
         << "      \"context_signature\": [1],\n"
         << "      \"height_m\": -0.1\n"
         << "    },\n"
         << "    {\n"
         << "      \"anchor_id\": \"good\",\n"
         << "      \"chainage_m\": 14.0,\n"
         << "      \"center\": [1.0, 2.0, 3.0],\n"
         << "      \"context_signature\": [2, 1],\n"
         << "      \"height_m\": 1.2\n"
         << "    }\n"
         << "  ]\n"
         << "}\n";
  stream.close();

  TcaLedger ledger;

  EXPECT_NO_THROW(ledger.load(path.string()));
  ASSERT_EQ(1u, ledger.anchors().size());
  EXPECT_EQ("good", ledger.anchors()[0].anchor_id);

  boost::filesystem::remove_all(root);
}

TEST(TcaLedger, LoadSkipsNonnumericChainageAnchor) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("tca_ledger_bad_chainage_%%%%%%");
  boost::filesystem::create_directories(root);
  const boost::filesystem::path path = root / "ledger.json";

  std::ofstream stream(path.string());
  stream << "{\n"
         << "  \"anchors\": [\n"
         << "    {\n"
         << "      \"anchor_id\": \"bad_chainage\",\n"
         << "      \"chainage_m\": \"not-a-number\",\n"
         << "      \"center\": [0.0, 0.0, 0.0],\n"
         << "      \"context_signature\": [1],\n"
         << "      \"height_m\": 1.0\n"
         << "    },\n"
         << "    {\n"
         << "      \"anchor_id\": \"good\",\n"
         << "      \"chainage_m\": 14.0,\n"
         << "      \"center\": [1.0, 2.0, 3.0],\n"
         << "      \"context_signature\": [2, 1],\n"
         << "      \"height_m\": 1.2\n"
         << "    }\n"
         << "  ]\n"
         << "}\n";
  stream.close();

  TcaLedger ledger;

  EXPECT_NO_THROW(ledger.load(path.string()));
  ASSERT_EQ(1u, ledger.anchors().size());
  EXPECT_EQ("good", ledger.anchors()[0].anchor_id);

  boost::filesystem::remove_all(root);
}

TEST(TcaLedger, LoadSkipsNonnumericHeightAnchor) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("tca_ledger_bad_height_%%%%%%");
  boost::filesystem::create_directories(root);
  const boost::filesystem::path path = root / "ledger.json";

  std::ofstream stream(path.string());
  stream << "{\n"
         << "  \"anchors\": [\n"
         << "    {\n"
         << "      \"anchor_id\": \"bad_height\",\n"
         << "      \"chainage_m\": 1.0,\n"
         << "      \"center\": [0.0, 0.0, 0.0],\n"
         << "      \"context_signature\": [1],\n"
         << "      \"height_m\": \"not-a-number\"\n"
         << "    },\n"
         << "    {\n"
         << "      \"anchor_id\": \"good\",\n"
         << "      \"chainage_m\": 14.0,\n"
         << "      \"center\": [1.0, 2.0, 3.0],\n"
         << "      \"context_signature\": [2, 1],\n"
         << "      \"height_m\": 1.2\n"
         << "    }\n"
         << "  ]\n"
         << "}\n";
  stream.close();

  TcaLedger ledger;

  EXPECT_NO_THROW(ledger.load(path.string()));
  ASSERT_EQ(1u, ledger.anchors().size());
  EXPECT_EQ("good", ledger.anchors()[0].anchor_id);

  boost::filesystem::remove_all(root);
}

}  // namespace
}  // namespace tca_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
