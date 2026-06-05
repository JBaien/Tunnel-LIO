#include <gtest/gtest.h>

#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "mine_slam_bringup/event_dump.h"

namespace {

bool exists(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

bool isDirectory(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

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

void writeValidMetadata(const std::string& session_dir) {
  writeFile(session_dir + "/metadata.env",
            "session_name=session_a\n"
            "bag_path=" + session_dir + "/bags/tunnel_lio.bag\n"
            "pcap_path=" + session_dir + "/pcap/tunnel_lio.pcap\n"
            "topics=/points_raw /tf /diagnostics/lidar_fusion\n");
}

mine_slam_bringup::EventDumpRequest validRequest(const std::string& root,
                                                  const std::string& session_dir) {
  mine_slam_bringup::EventDumpRequest request;
  request.session_dir = session_dir;
  request.output_root = root + "/events";
  request.event_id = "power_loss_001";
  request.event_time_s = 120.0;
  request.window_before_s = 10.0;
  request.window_after_s = 15.0;
  request.reason = "power_loss_recovery";
  return request;
}

}  // namespace

TEST(EventDump, CreatesReplayFriendlyEvidenceSlicePlan) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeValidMetadata(session_dir);

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_TRUE(result.success);
  EXPECT_EQ("power_loss_001", result.event_id);
  EXPECT_TRUE(isDirectory(result.event_dir));
  EXPECT_TRUE(isDirectory(result.event_dir + "/commands"));
  EXPECT_TRUE(exists(result.event_dir + "/event_manifest.env"));
  EXPECT_TRUE(exists(result.event_dir + "/commands/filter_rosbag.sh"));
  EXPECT_TRUE(exists(result.event_dir + "/commands/extract_pcap_window.sh"));

  const std::string manifest = readFile(result.event_dir + "/event_manifest.env");
  EXPECT_NE(std::string::npos, manifest.find("event_id=power_loss_001"));
  EXPECT_NE(std::string::npos, manifest.find("reason=power_loss_recovery"));
  EXPECT_NE(std::string::npos, manifest.find("start_time_s=110.000"));
  EXPECT_NE(std::string::npos, manifest.find("end_time_s=135.000"));
  EXPECT_NE(std::string::npos, manifest.find("bag_path=" + session_dir + "/bags/tunnel_lio.bag"));

  const std::string bag_command = readFile(result.event_dir + "/commands/filter_rosbag.sh");
  EXPECT_NE(std::string::npos, bag_command.find("rosbag filter"));
  EXPECT_NE(std::string::npos, bag_command.find("t.to_sec() >= 110.000"));
  EXPECT_NE(std::string::npos, bag_command.find("t.to_sec() <= 135.000"));

  const std::string pcap_command =
      readFile(result.event_dir + "/commands/extract_pcap_window.sh");
  EXPECT_NE(std::string::npos, pcap_command.find("editcap"));
  EXPECT_NE(std::string::npos, pcap_command.find("tunnel_lio.pcap"));
}

TEST(EventDump, RejectsWhitespacePollutedMetadataValues) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_bad_metadata_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  writeFile(session_dir + "/metadata.env",
            "session_name=session_a\n"
            "bag_path=" + session_dir + "/bags/tunnel_lio.bag \n"
            "pcap_path=" + session_dir + "/pcap/tunnel_lio.pcap\n"
            "topics=/points_raw /tf /diagnostics/lidar_fusion\n");

  mine_slam_bringup::EventDumpRequest request;
  request.session_dir = session_dir;
  request.output_root = std::string(root) + "/events";
  request.event_id = "power_loss_001";
  request.event_time_s = 120.0;
  request.window_before_s = 10.0;
  request.window_after_s = 15.0;
  request.reason = "power_loss_recovery";

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("bag_path"));
}

TEST(EventDump, RejectsRelativeMetadataArtifactPaths) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_relative_metadata_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  writeFile(session_dir + "/metadata.env",
            "session_name=session_a\n"
            "bag_path=bags/tunnel_lio.bag\n"
            "pcap_path=" + session_dir + "/pcap/tunnel_lio.pcap\n"
            "topics=/points_raw /tf /diagnostics/lidar_fusion\n");

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("bag_path"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

TEST(EventDump, RejectsMetadataArtifactPathsWithDotSegments) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_metadata_dot_path_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  writeFile(session_dir + "/metadata.env",
            "session_name=session_a\n"
            "bag_path=" + session_dir + "/bags/../bags/tunnel_lio.bag\n"
            "pcap_path=" + session_dir + "/pcap/tunnel_lio.pcap\n"
            "topics=/points_raw /tf /diagnostics/lidar_fusion\n");

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("bag_path"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

TEST(EventDump, RejectsMetadataArtifactRootPaths) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_metadata_root_path_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  writeFile(session_dir + "/metadata.env",
            "session_name=session_a\n"
            "bag_path=/\n"
            "pcap_path=" + session_dir + "/pcap/tunnel_lio.pcap\n"
            "topics=/points_raw /tf /diagnostics/lidar_fusion\n");

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("bag_path"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

TEST(EventDump, RejectsMetadataSessionNameOutsideSafeToken) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_bad_session_name_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeFile(session_dir + "/metadata.env",
            "session_name=session a\n"
            "bag_path=" + session_dir + "/bags/tunnel_lio.bag\n"
            "pcap_path=" + session_dir + "/pcap/tunnel_lio.pcap\n"
            "topics=/points_raw /tf /diagnostics/lidar_fusion\n");

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("session_name"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

TEST(EventDump, RejectsMetadataSessionNameDotSegments) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_session_dot_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeFile(session_dir + "/metadata.env",
            "session_name=..\n"
            "bag_path=" + session_dir + "/bags/tunnel_lio.bag\n"
            "pcap_path=" + session_dir + "/pcap/tunnel_lio.pcap\n"
            "topics=/points_raw /tf /diagnostics/lidar_fusion\n");

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("session_name"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

TEST(EventDump, RejectsRelativeRequestPaths) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_relative_request_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeValidMetadata(session_dir);

  char previous_cwd[4096];
  ASSERT_NE(nullptr, getcwd(previous_cwd, sizeof(previous_cwd)));
  ASSERT_EQ(0, chdir(root));

  mine_slam_bringup::EventDumpRequest output_root_request = validRequest(root, session_dir);
  output_root_request.output_root = "events";
  const mine_slam_bringup::EventDumpResult output_root_result =
      mine_slam_bringup::createEventDump(output_root_request);
  EXPECT_FALSE(output_root_result.success);
  EXPECT_NE(std::string::npos, output_root_result.message.find("output_root"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));

  mine_slam_bringup::EventDumpRequest session_dir_request = validRequest(root, session_dir);
  session_dir_request.session_dir = "session_a";
  const mine_slam_bringup::EventDumpResult session_dir_result =
      mine_slam_bringup::createEventDump(session_dir_request);
  EXPECT_FALSE(session_dir_result.success);
  EXPECT_NE(std::string::npos, session_dir_result.message.find("session_dir"));

  EXPECT_EQ(0, chdir(previous_cwd));
}

TEST(EventDump, RejectsRequestPathsWithDotSegments) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_request_dot_path_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeValidMetadata(session_dir);

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);
  request.output_root = std::string(root) + "/events/../escaped_events";

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("output_root"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
  EXPECT_FALSE(exists(std::string(root) + "/escaped_events"));
}

TEST(EventDump, RejectsRequestReasonWithManifestPollution) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_bad_reason_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeValidMetadata(session_dir);

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);
  request.reason = "power_loss_recovery\nsource=manual";

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("reason"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

TEST(EventDump, RejectsRequestEventIdBeforeSanitizing) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_bad_event_id_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeValidMetadata(session_dir);

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);
  request.event_id = "power_loss;touch";

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("event_id"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

TEST(EventDump, RejectsRequestEventIdDotSegments) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_event_dot_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeValidMetadata(session_dir);

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);
  request.event_id = "..";

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("event_id"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

TEST(EventDump, RejectsRequestOutputRootWithShellPollution) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_bad_output_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeValidMetadata(session_dir);

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);
  request.output_root = std::string(root) + "/events\";touch /tmp/bad;\"";

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("output_root"));
}

TEST(EventDump, RejectsNonFiniteEventTimes) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_bad_time_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeValidMetadata(session_dir);

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);
  request.event_time_s = std::numeric_limits<double>::infinity();

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("event_time_s"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

TEST(EventDump, RejectsNegativeEventTime) {
  char root_template[] = "/tmp/tunnel_lio_event_dump_negative_time_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string session_dir = std::string(root) + "/session_a";
  ASSERT_EQ(0, mkdir(session_dir.c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/bags").c_str(), 0755));
  ASSERT_EQ(0, mkdir((session_dir + "/pcap").c_str(), 0755));
  writeValidMetadata(session_dir);

  mine_slam_bringup::EventDumpRequest request = validRequest(root, session_dir);
  request.event_time_s = -1.0;

  const mine_slam_bringup::EventDumpResult result =
      mine_slam_bringup::createEventDump(request);

  EXPECT_FALSE(result.success);
  EXPECT_NE(std::string::npos, result.message.find("event_time_s"));
  EXPECT_FALSE(exists(std::string(root) + "/events"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
