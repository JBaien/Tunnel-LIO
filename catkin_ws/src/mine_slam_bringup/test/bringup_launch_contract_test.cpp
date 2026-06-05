#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string readFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string bringupLaunchDir() {
  return std::string(BRINGUP_LAUNCH_DIR);
}

}  // namespace

TEST(BringupLaunchContract, SectionManagerLaunchAcceptsSessionIdOverride) {
  const std::string launch =
      readFile(std::string(SECTION_MANAGER_LAUNCH_PATH));

  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"session_id\" default=\"unassigned\""));
  EXPECT_NE(std::string::npos,
            launch.find("<param name=\"session_id\" value=\"$(arg session_id)\""));
}

TEST(BringupLaunchContract, SessionManagerLaunchAcceptsSessionRootOverride) {
  const std::string launch =
      readFile(std::string(CATKIN_WORKSPACE_SRC_DIR) +
               "/lio_session_manager/launch/session_manager.launch");

  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"session_root\" default=\"/tmp/tunnel_lio_sessions\""));
  EXPECT_NE(std::string::npos,
            launch.find("<param name=\"session_root\" value=\"$(arg session_root)\""));
}

TEST(BringupLaunchContract, FusionBringupPassesSectionSessionIdToSectionManager) {
  const std::string timoo =
      readFile(bringupLaunchDir() + "/bringup_fusion_timoo.launch");
  const std::string tmlidar =
      readFile(bringupLaunchDir() + "/bringup_fusion_tmlidar.launch");

  EXPECT_NE(std::string::npos,
            timoo.find("<arg name=\"section_session_id\" default=\"unassigned\""));
  EXPECT_NE(std::string::npos,
            timoo.find("<arg name=\"session_id\" value=\"$(arg section_session_id)\""));
  EXPECT_NE(std::string::npos,
            tmlidar.find("<arg name=\"section_session_id\" default=\"unassigned\""));
  EXPECT_NE(std::string::npos,
            tmlidar.find("<arg name=\"session_id\" value=\"$(arg section_session_id)\""));
}

TEST(BringupLaunchContract, TunnelBagReplayLaunchUsesActualBagProfile) {
  const std::string launch =
      readFile(bringupLaunchDir() + "/bringup_replay_tunnel_bag.launch");

  EXPECT_NE(std::string::npos,
            launch.find("multi_lidar_fusion_tunnel_bag.yaml"));
  EXPECT_NE(std::string::npos,
            launch.find("preprocess_tunnel_bag.yaml"));
  EXPECT_NE(std::string::npos,
            launch.find("state_estimator_tunnel_bag.yaml"));
  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"local_odometry_config\""));
  EXPECT_NE(std::string::npos,
            launch.find("local_icp_odometry_tunnel_bag.yaml"));
  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"config\" value=\"$(arg local_odometry_config)\""));
  EXPECT_NE(std::string::npos,
            launch.find("extrinsics_tunnel_bag_identity.yaml"));
  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"start_plc_modbus\" value=\"false\""));
  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"imu_topic\" value=\"/imu/data\""));
  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"enable_pps\" value=\"false\""));
  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"session_root\" value=\"$(arg session_root)\""));
}

TEST(BringupLaunchContract, TunnelBagLocalOdometryProfilePreservesRegistrationGate) {
  const std::string config =
      readFile(std::string(CATKIN_WORKSPACE_SRC_DIR) +
               "/lio_local_odometry/config/local_icp_odometry_tunnel_bag.yaml");

  EXPECT_NE(std::string::npos, config.find("voxel_leaf_size: 0.10"));
  EXPECT_NE(std::string::npos,
            config.find("registration_voxel_leaf_sizes: [0.30, 0.15, 0.0]"));
  EXPECT_NE(std::string::npos,
            config.find("max_fitness_score: 0.5"));
  EXPECT_NE(std::string::npos,
            config.find("max_correspondence_distance: 1.0"));
  EXPECT_NE(std::string::npos, config.find("max_iterations: 50"));
  EXPECT_NE(std::string::npos, config.find("cloud_queue_size: 50"));
  EXPECT_NE(std::string::npos,
            config.find("reseed_keyframe_after_consecutive_rejections: 2"));
  EXPECT_NE(std::string::npos,
            config.find("submap_voxel_leaf_size: 0.20"));
  EXPECT_NE(std::string::npos, config.find("max_submap_points: 200000"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
