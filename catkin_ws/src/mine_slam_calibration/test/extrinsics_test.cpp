#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "mine_slam_calibration/extrinsics.h"

namespace mine_slam_calibration {
namespace {

std::string readFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

void writeFile(const std::string& path, const std::string& content) {
  std::ofstream output(path.c_str());
  output << content;
}

TEST(ExtrinsicsAudit, DefaultConfigPassesAudit) {
  const ExtrinsicsData data =
      loadExtrinsics("/home/bai/Desktop/Tunnel-LIO/catkin_ws/src/mine_slam_calibration/config/extrinsics.yaml");
  EXPECT_TRUE(auditExtrinsics(data).empty());
}

TEST(ExtrinsicsAudit, DefaultConfigIncludesExplicitImuNativeAxisAdapter) {
  const ExtrinsicsData data =
      loadExtrinsics("/home/bai/Desktop/Tunnel-LIO/catkin_ws/src/mine_slam_calibration/config/extrinsics.yaml");

  const auto transform = std::find_if(
      data.transforms.begin(), data.transforms.end(), [](const TransformSpec& item) {
        return item.parent == "imu_native" && item.child == "imu_link";
      });

  ASSERT_NE(data.transforms.end(), transform);
  EXPECT_NEAR(0.0, transform->translation[0], 1e-12);
  EXPECT_NEAR(0.0, transform->translation[1], 1e-12);
  EXPECT_NEAR(0.0, transform->translation[2], 1e-12);
  EXPECT_NEAR(0.0, transform->rotation_rpy[0], 1e-12);
  EXPECT_NEAR(0.0, transform->rotation_rpy[1], 1e-12);
  EXPECT_NEAR(1.5707963267948966, transform->rotation_rpy[2], 1e-9);
}

TEST(ExtrinsicsAudit, DuplicateChildIsReported) {
  ExtrinsicsData data;
  data.reference_frame = "base_link";
  data.frames.insert("base_link");
  data.frames.insert("lidar_center");
  data.frames.insert("lidar_left");
  data.transforms.push_back(TransformSpec{"base_link", "lidar_center", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  data.transforms.push_back(TransformSpec{"lidar_left", "lidar_center", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});

  const std::vector<std::string> issues = auditExtrinsics(data);
  EXPECT_NE(issues.end(), std::find_if(issues.begin(), issues.end(), [](const std::string& issue) {
              return issue.find("multiple parents") != std::string::npos;
            }));
}

TEST(ExtrinsicsAudit, MissingRequiredFrameIsReported) {
  ExtrinsicsData data;
  data.reference_frame = "base_link";
  data.frames.insert("base_link");
  data.frames.insert("lidar_center");
  data.required_frames.push_back("base_link");
  data.required_frames.push_back("lidar_center");
  data.required_frames.push_back("imu_link");
  data.transforms.push_back(TransformSpec{"base_link", "lidar_center", {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});

  const std::vector<std::string> issues = auditExtrinsics(data);
  EXPECT_NE(issues.end(), std::find(issues.begin(), issues.end(),
                                    "required frame 'imu_link' is not declared"));
}

TEST(ExtrinsicsAudit, ImplausibleTransformMagnitudeIsReported) {
  ExtrinsicsData data;
  data.reference_frame = "base_link";
  data.frames.insert("base_link");
  data.frames.insert("lidar_center");
  data.transforms.push_back(TransformSpec{"base_link", "lidar_center", {30.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});

  const std::vector<std::string> issues = auditExtrinsics(data);
  EXPECT_NE(issues.end(), std::find(issues.begin(), issues.end(),
                                    "transform[0].translation norm 30 exceeds max 20 m"));
}

TEST(ExtrinsicsAudit, AmbiguousRotationWrapIsReported) {
  ExtrinsicsData data;
  data.reference_frame = "base_link";
  data.frames.insert("base_link");
  data.frames.insert("imu_link");
  data.transforms.push_back(TransformSpec{"base_link", "imu_link", {0.0, 0.0, 0.0}, {0.0, 0.0, 7.0}});

  const std::vector<std::string> issues = auditExtrinsics(data);
  EXPECT_NE(issues.end(), std::find(issues.begin(), issues.end(),
                                    "transform[0].rotation_rpy[2] exceeds +/-2pi and is ambiguous"));
}

TEST(ExtrinsicsAuditReport, BuildsServiceFriendlyReportFromFile) {
  char root_template[] = "/tmp/tunnel_lio_extrinsics_audit_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string path = std::string(root) + "/bad_extrinsics.yaml";
  writeFile(path,
            "frames:\n"
            "  base_link: {}\n"
            "  lidar_center: {}\n"
            "required_frames:\n"
            "  - base_link\n"
            "  - lidar_center\n"
            "  - imu_link\n"
            "calibration:\n"
            "  reference_frame: base_link\n"
            "transforms:\n"
            "  - parent: base_link\n"
            "    child: lidar_center\n"
            "    translation: [30.0, 0.0, 0.0]\n"
            "    rotation_rpy: [0.0, 0.0, 0.0]\n");

  const AuditReport report = auditExtrinsicsFile(path);

  EXPECT_FALSE(report.ok);
  EXPECT_EQ(path, report.path);
  ASSERT_GE(report.issues.size(), 2u);
  EXPECT_NE(report.text.find("ok=false"), std::string::npos);
  EXPECT_NE(report.text.find("issue_count="), std::string::npos);
  EXPECT_NE(report.text.find("required frame 'imu_link' is not declared"), std::string::npos);
  EXPECT_NE(report.text.find("translation norm 30 exceeds max 20 m"), std::string::npos);
}

}  // namespace
}  // namespace mine_slam_calibration

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
