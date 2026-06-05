#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

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

std::size_t countOccurrences(const std::string& text,
                             const std::string& pattern) {
  if (pattern.empty()) {
    return 0;
  }
  std::size_t count = 0;
  std::size_t position = 0;
  while ((position = text.find(pattern, position)) != std::string::npos) {
    ++count;
    position += pattern.size();
  }
  return count;
}

void writeFile(const std::string& path, const std::string& contents) {
  std::ofstream output(path.c_str());
  output << contents;
}

std::string fakeRuntimeHealthScript(const std::string& runtime_dir,
                                    const std::string& systemd_active,
                                    const std::string& docker_status,
                                    const std::string& report_name =
                                        "runtime_health_fake.txt",
                                    const std::string& extra_report_lines = "") {
  std::string script =
      "#!/usr/bin/env bash\n"
      "set -euo pipefail\n"
      "log_dir=\"" + runtime_dir + "/logs\"\n"
      "report_path=\"${log_dir}/" + report_name + "\"\n"
      "mkdir -p \"${log_dir}\"\n"
      "{\n"
      "  echo \"timestamp=2026-06-03T00:00:00+08:00\"\n"
      "  echo \"runtime_dir=" + runtime_dir + "\"\n"
      "  echo \"log_dir=" + runtime_dir + "/logs\"\n"
      "  echo \"state_dir=" + runtime_dir + "/state\"\n"
      "  echo \"disk_available_gb=120\"\n"
      "  echo \"runtime_pid=1234\"\n"
      "  echo \"systemd_unit=tunnel-lio.service\"\n"
      "  echo \"systemd_active=" + systemd_active + "\"\n"
      "  echo \"systemd_active_source=systemctl\"\n"
      "  echo \"docker_container=tunnel-lio-runtime\"\n"
      "  echo \"docker_container_status=" + docker_status + "\"\n"
      "  echo \"docker_container_status_source=docker_inspect\"\n"
      "} > \"${report_path}\"\n";
  if (!extra_report_lines.empty()) {
    script += "cat >> \"${report_path}\" <<'RUNTIME_HEALTH_EXTRA'\n" +
              extra_report_lines + "RUNTIME_HEALTH_EXTRA\n";
  }
  script +=
      "ln -sfn \"${report_path}\" \"${log_dir}/runtime_health_latest.txt\"\n"
      "echo \"${report_path}\"\n";
  return script;
}

}  // namespace

TEST(RuntimeOpsScript, DryRunCreatesBoardRuntimePlan) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 12 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --cpu-set 2-5 --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  EXPECT_TRUE(isDirectory(runtime_dir + "/logs"));
  EXPECT_TRUE(isDirectory(runtime_dir + "/state"));
  EXPECT_TRUE(isDirectory(runtime_dir + "/commands"));
  EXPECT_TRUE(isDirectory(runtime_dir + "/systemd"));
  EXPECT_TRUE(isDirectory(runtime_dir + "/docker"));
  EXPECT_TRUE(exists(runtime_dir + "/runtime.env"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/start_runtime.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/disk_guard.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/watchdog_check.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/runtime_health.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/runtime_stability_check.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/runtime_deployment_check.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/systemd/tunnel-lio.service"));
  EXPECT_TRUE(exists(runtime_dir + "/systemd/tunnel-lio.env"));
  EXPECT_TRUE(exists(runtime_dir + "/docker/Dockerfile"));
  EXPECT_TRUE(exists(runtime_dir + "/docker/docker-compose.yaml"));
  EXPECT_TRUE(exists(runtime_dir + "/docker/tunnel-lio.env"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/install_systemd.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/docker_build.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/docker_up.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/docker_down.sh"));
  EXPECT_TRUE(exists(runtime_dir + "/commands/docker_logs.sh"));

  const std::string metadata = readFile(runtime_dir + "/runtime.env");
  EXPECT_NE(std::string::npos, metadata.find("runtime_name=board_alpha"));
  EXPECT_NE(std::string::npos,
            metadata.find("workspace=/opt/tunnel_lio/catkin_ws"));
  EXPECT_NE(std::string::npos,
            metadata.find("launch=mine_slam_bringup bringup_sensors.launch"));
  EXPECT_NE(std::string::npos, metadata.find("min_free_gb=12"));
  EXPECT_NE(std::string::npos, metadata.find("log_retention_days=5"));
  EXPECT_NE(std::string::npos, metadata.find("watchdog_topic=/time/status"));
  EXPECT_NE(std::string::npos, metadata.find("cpu_set=2-5"));
  EXPECT_NE(std::string::npos,
            metadata.find("systemd_unit=" + runtime_dir + "/systemd/tunnel-lio.service"));
  EXPECT_NE(std::string::npos,
            metadata.find("docker_compose=" + runtime_dir + "/docker/docker-compose.yaml"));

  const std::string start_command =
      readFile(runtime_dir + "/commands/start_runtime.sh");
  EXPECT_NE(std::string::npos, start_command.find("ROS_LOG_DIR"));
  EXPECT_NE(std::string::npos,
            start_command.find("source \"/opt/tunnel_lio/catkin_ws/devel/setup.bash\""));
  EXPECT_NE(std::string::npos,
            start_command.find("exec taskset -c \"2-5\" roslaunch mine_slam_bringup bringup_sensors.launch"));
  EXPECT_NE(std::string::npos,
            start_command.find("roslaunch mine_slam_bringup bringup_sensors.launch"));

  const std::string disk_guard =
      readFile(runtime_dir + "/commands/disk_guard.sh");
  EXPECT_NE(std::string::npos, disk_guard.find("min_free_gb=12"));
  EXPECT_NE(std::string::npos, disk_guard.find("find \"${runtime_dir}/logs\""));

  const std::string watchdog =
      readFile(runtime_dir + "/commands/watchdog_check.sh");
  EXPECT_NE(std::string::npos, watchdog.find("rostopic echo -n1"));
  EXPECT_NE(std::string::npos, watchdog.find("/time/status"));
  EXPECT_NE(std::string::npos, watchdog.find("timeout_s=7"));

  const std::string health =
      readFile(runtime_dir + "/commands/runtime_health.sh");
  EXPECT_NE(std::string::npos, health.find("runtime_health_latest.txt"));
  EXPECT_NE(std::string::npos, health.find("systemd_active"));
  EXPECT_NE(std::string::npos, health.find("docker_container_status"));
  EXPECT_EQ(0, std::system((runtime_dir + "/commands/runtime_health.sh").c_str()));
  const std::string health_report =
      readFile(runtime_dir + "/logs/runtime_health_latest.txt");
  EXPECT_NE(std::string::npos, health_report.find("runtime_dir=" + runtime_dir));
  EXPECT_NE(std::string::npos, health_report.find("disk_available_gb="));
  EXPECT_NE(std::string::npos, health_report.find("runtime_pid="));
  EXPECT_NE(std::string::npos, health_report.find("systemd_active="));
  EXPECT_NE(std::string::npos, health_report.find("systemd_active_source="));
  EXPECT_NE(std::string::npos,
            health_report.find("docker_container_status_source="));
  const std::size_t docker_status_key =
      health_report.find("docker_container_status=");
  ASSERT_NE(std::string::npos, docker_status_key);
  EXPECT_EQ(std::string::npos,
            health_report.find("docker_container_status=",
                               docker_status_key + 1));

  const std::string deployment =
      readFile(runtime_dir + "/commands/runtime_deployment_check.sh");
  EXPECT_NE(std::string::npos, deployment.find("runtime_deployment_check.txt"));
  EXPECT_NE(std::string::npos, deployment.find("systemd_unit_file="));
  EXPECT_NE(std::string::npos, deployment.find("docker_compose_file="));
  EXPECT_NE(std::string::npos, deployment.find("runtime_process_status="));
  EXPECT_NE(std::string::npos, deployment.find("TUNNEL_LIO_SYSTEMD_ACTIVE"));
  EXPECT_NE(std::string::npos, deployment.find("TUNNEL_LIO_DOCKER_STATUS"));
  EXPECT_NE(std::string::npos, deployment.find("deployment_status="));
  const std::string deployment_command =
      "TUNNEL_LIO_SYSTEMD_ACTIVE=active TUNNEL_LIO_DOCKER_STATUS=running " +
      runtime_dir + "/commands/runtime_deployment_check.sh";
  EXPECT_NE(0, std::system(deployment_command.c_str()));
  const std::string deployment_report =
      readFile(runtime_dir + "/logs/runtime_deployment_check.txt");
  EXPECT_NE(std::string::npos,
            deployment_report.find("runtime_dir=" + runtime_dir));
  EXPECT_NE(std::string::npos, deployment_report.find("systemd_unit_file=PASS"));
  EXPECT_NE(std::string::npos, deployment_report.find("systemd_env_file=PASS"));
  EXPECT_NE(std::string::npos, deployment_report.find("docker_compose_file=PASS"));
  EXPECT_NE(std::string::npos, deployment_report.find("docker_env_file=PASS"));
  EXPECT_NE(std::string::npos, deployment_report.find("systemd_active=active"));
  EXPECT_NE(std::string::npos,
            deployment_report.find("systemd_active_source=env_override"));
  EXPECT_NE(std::string::npos,
            deployment_report.find("docker_container_status=running"));
  EXPECT_NE(std::string::npos,
            deployment_report.find("docker_container_status_source=env_override"));
  EXPECT_NE(std::string::npos,
            deployment_report.find("runtime_process_status=FAIL"));
  EXPECT_NE(std::string::npos, deployment_report.find("start_command=PASS"));
  EXPECT_NE(std::string::npos, deployment_report.find("deployment_status=FAIL"));

  const std::string stability =
      readFile(runtime_dir + "/commands/runtime_stability_check.sh");
  EXPECT_NE(std::string::npos, stability.find("runtime_stability.csv"));
  EXPECT_NE(std::string::npos, stability.find("runtime_stability_summary.txt"));
  EXPECT_NE(std::string::npos, stability.find("TUNNEL_LIO_SKIP_WATCHDOG"));
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "active", "running"));
  ASSERT_EQ(0, chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));
  const std::string stability_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + runtime_dir +
      "/commands/runtime_stability_check.sh --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(stability_command.c_str()));
  const std::string stability_csv =
      readFile(runtime_dir + "/logs/runtime_stability.csv");
  EXPECT_NE(std::string::npos,
            stability_csv.find("sample,timestamp,disk_guard_status,watchdog_status,health_report"));
  EXPECT_NE(std::string::npos, stability_csv.find("1,"));
  EXPECT_NE(std::string::npos, stability_csv.find(",PASS,SKIP,"));
  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("samples=1"));
  EXPECT_NE(std::string::npos, stability_summary.find("watchdog_skipped=1"));

  const std::string service =
      readFile(runtime_dir + "/systemd/tunnel-lio.service");
  EXPECT_NE(std::string::npos, service.find("Description=Tunnel-LIO runtime"));
  EXPECT_NE(std::string::npos, service.find("EnvironmentFile=" + runtime_dir + "/systemd/tunnel-lio.env"));
  EXPECT_NE(std::string::npos, service.find("ExecStart=" + runtime_dir + "/commands/start_runtime.sh"));
  EXPECT_NE(std::string::npos, service.find("Restart=always"));

  const std::string systemd_env =
      readFile(runtime_dir + "/systemd/tunnel-lio.env");
  EXPECT_NE(std::string::npos, systemd_env.find("TUNNEL_LIO_RUNTIME_DIR=" + runtime_dir));
  EXPECT_NE(std::string::npos, systemd_env.find("ROS_LOG_DIR=" + runtime_dir + "/logs"));
  EXPECT_NE(std::string::npos, systemd_env.find("TUNNEL_LIO_CPU_SET=2-5"));

  const std::string install_systemd =
      readFile(runtime_dir + "/commands/install_systemd.sh");
  EXPECT_NE(std::string::npos, install_systemd.find("systemctl daemon-reload"));
  EXPECT_NE(std::string::npos, install_systemd.find("systemctl enable tunnel-lio.service"));
  EXPECT_NE(std::string::npos, install_systemd.find("tunnel-lio.service"));

  const std::string dockerfile = readFile(runtime_dir + "/docker/Dockerfile");
  EXPECT_NE(std::string::npos, dockerfile.find("FROM ros:noetic-ros-core"));
  EXPECT_NE(std::string::npos, dockerfile.find("WORKDIR /opt/tunnel_lio/catkin_ws"));
  EXPECT_NE(std::string::npos, dockerfile.find("ENTRYPOINT"));

  const std::string compose =
      readFile(runtime_dir + "/docker/docker-compose.yaml");
  EXPECT_NE(std::string::npos, compose.find("tunnel-lio-runtime"));
  EXPECT_NE(std::string::npos, compose.find("network_mode: host"));
  EXPECT_NE(std::string::npos, compose.find("privileged: true"));
  EXPECT_NE(std::string::npos, compose.find(runtime_dir + ":/runtime"));
  EXPECT_NE(std::string::npos, compose.find("command: /runtime/commands/start_runtime.sh"));
  EXPECT_NE(std::string::npos, compose.find("env_file:"));

  const std::string docker_env =
      readFile(runtime_dir + "/docker/tunnel-lio.env");
  EXPECT_NE(std::string::npos, docker_env.find("TUNNEL_LIO_RUNTIME_DIR=/runtime"));
  EXPECT_NE(std::string::npos, docker_env.find("ROS_LOG_DIR=/runtime/logs"));

  const std::string docker_build =
      readFile(runtime_dir + "/commands/docker_build.sh");
  EXPECT_NE(std::string::npos, docker_build.find("docker compose"));
  EXPECT_NE(std::string::npos, docker_build.find("build"));
  EXPECT_NE(std::string::npos, docker_build.find(runtime_dir + "/docker/docker-compose.yaml"));

  const std::string docker_up = readFile(runtime_dir + "/commands/docker_up.sh");
  EXPECT_NE(std::string::npos, docker_up.find("docker compose"));
  EXPECT_NE(std::string::npos, docker_up.find("up -d"));
  EXPECT_NE(std::string::npos, docker_up.find(runtime_dir + "/docker/docker-compose.yaml"));

  const std::string docker_down =
      readFile(runtime_dir + "/commands/docker_down.sh");
  EXPECT_NE(std::string::npos, docker_down.find("docker compose"));
  EXPECT_NE(std::string::npos, docker_down.find("down"));
  EXPECT_NE(std::string::npos, docker_down.find(runtime_dir + "/docker/docker-compose.yaml"));

  const std::string docker_logs =
      readFile(runtime_dir + "/commands/docker_logs.sh");
  EXPECT_NE(std::string::npos, docker_logs.find("docker compose"));
  EXPECT_NE(std::string::npos, docker_logs.find("logs --tail=200"));
  EXPECT_NE(std::string::npos, docker_logs.find("tunnel-lio-runtime"));
  EXPECT_NE(std::string::npos, docker_logs.find(runtime_dir + "/docker/docker-compose.yaml"));
}

TEST(RuntimeOpsScript, StabilityRejectsInactiveRuntimeHealthReport) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_health_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 1 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "inactive", "exited"));
  ASSERT_EQ(0, chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));

  const std::string stability_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + runtime_dir +
      "/commands/runtime_stability_check.sh --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(stability_command.c_str()));

  const std::string health_report =
      readFile(runtime_dir + "/logs/runtime_health_latest.txt");
  EXPECT_NE(std::string::npos, health_report.find("systemd_active=inactive"));
  EXPECT_NE(std::string::npos,
            health_report.find("docker_container_status=exited"));

  const std::string stability_csv =
      readFile(runtime_dir + "/logs/runtime_stability.csv");
  EXPECT_NE(std::string::npos, stability_csv.find("runtime_health_fake.txt"));

  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("health_failures=1"));
  EXPECT_NE(std::string::npos, stability_summary.find("watchdog_skipped=1"));
}

TEST(RuntimeOpsScript, StabilityMarksSkippedWatchdogAsOverallFail) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_watchdog_skip_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 1 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "active", "running"));
  ASSERT_EQ(0,
            chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));

  const std::string stability_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + runtime_dir +
      "/commands/runtime_stability_check.sh --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(stability_command.c_str()));

  const std::string stability_csv =
      readFile(runtime_dir + "/logs/runtime_stability.csv");
  EXPECT_NE(std::string::npos, stability_csv.find(",PASS,SKIP,"));

  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("watchdog_failures=0"));
  EXPECT_NE(std::string::npos, stability_summary.find("watchdog_skipped=1"));
  EXPECT_NE(std::string::npos, stability_summary.find("health_failures=0"));
}

TEST(RuntimeOpsScript, StabilityRejectsMalformedRuntimeHealthKeyValueLine) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_health_malformed_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 1 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "active", "running",
                                    "runtime_health_fake.txt",
                                    "not_a_key_value_pair\n"));
  ASSERT_EQ(0, chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));

  const std::string stability_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + runtime_dir +
      "/commands/runtime_stability_check.sh --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(stability_command.c_str()));

  const std::string stability_csv =
      readFile(runtime_dir + "/logs/runtime_stability.csv");
  EXPECT_NE(std::string::npos, stability_csv.find("runtime_health_fake.txt"));

  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("health_failures=1"));
}

TEST(RuntimeOpsScript, StabilityRejectsRuntimeHealthUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_health_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 1 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "active", "running",
                                    "runtime_health_fake.txt",
                                    "operator=qa\noperator=qa\n"));
  ASSERT_EQ(0, chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));

  const std::string stability_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + runtime_dir +
      "/commands/runtime_stability_check.sh --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(stability_command.c_str()));

  const std::string stability_csv =
      readFile(runtime_dir + "/logs/runtime_stability.csv");
  EXPECT_NE(std::string::npos, stability_csv.find("runtime_health_fake.txt"));

  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("health_failures=1"));
}

TEST(RuntimeOpsScript, StabilityRejectsCsvSeparatorHealthReportPath) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_health_csv_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 1 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "active", "running",
                                    "runtime_health_fake,bad.txt"));
  ASSERT_EQ(0, chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));

  const std::string stability_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + runtime_dir +
      "/commands/runtime_stability_check.sh --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(stability_command.c_str()));

  const std::string stability_csv =
      readFile(runtime_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, stability_csv.find("runtime_health_fake,bad.txt"));
  EXPECT_NE(std::string::npos, stability_csv.find(",PASS,SKIP,missing"));

  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("health_failures=1"));
}

TEST(RuntimeOpsScript, StabilityRejectsManifestSeparatorHealthReportPath) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_runtime_ops_health_separator_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 1 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "active", "running",
                                    "runtime_health_fake;bad.txt"));
  ASSERT_EQ(0, chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));

  const std::string stability_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + runtime_dir +
      "/commands/runtime_stability_check.sh --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(stability_command.c_str()));

  const std::string stability_csv =
      readFile(runtime_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, stability_csv.find("runtime_health_fake;bad.txt"));
  EXPECT_NE(std::string::npos, stability_csv.find(",PASS,SKIP,missing"));

  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("health_failures=1"));
}

TEST(RuntimeOpsScript, StabilityRejectsLineBreakHealthReportPath) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_health_line_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 1 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "active", "running",
                                    "runtime_health_fake\nbad.txt"));
  ASSERT_EQ(0, chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));

  const std::string stability_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + runtime_dir +
      "/commands/runtime_stability_check.sh --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(stability_command.c_str()));

  const std::string stability_csv =
      readFile(runtime_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos,
            stability_csv.find("runtime_health_fake\nbad.txt"));
  EXPECT_NE(std::string::npos, stability_csv.find(",PASS,SKIP,missing"));

  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("health_failures=1"));
}

TEST(RuntimeOpsScript, StabilityRejectsMalformedSamplingArguments) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_sampling_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 1 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "active", "running"));
  ASSERT_EQ(0, chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));

  const std::string stability_script =
      runtime_dir + "/commands/runtime_stability_check.sh";
  EXPECT_NE(0, std::system((stability_script +
                            " --samples 0 --interval 0 >/dev/null 2>&1")
                               .c_str()));
  EXPECT_NE(0, std::system((stability_script +
                            " --samples abc --interval 0 >/dev/null 2>&1")
                               .c_str()));
  EXPECT_NE(0, std::system((stability_script +
                            " --samples 1 --interval -1 >/dev/null 2>&1")
                               .c_str()));

  const std::string valid_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + stability_script +
      " --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(valid_command.c_str()));
  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("samples=1"));
  EXPECT_NE(std::string::npos, stability_summary.find("interval_s=0"));
}

TEST(RuntimeOpsScript, StabilityCheckStartsFreshCsvForEachRun) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_fresh_csv_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb 1 --log-retention-days 5"
      " --watchdog-topic /time/status --watchdog-timeout 7"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_alpha";
  writeFile(runtime_dir + "/commands/runtime_health.sh",
            fakeRuntimeHealthScript(runtime_dir, "active", "running"));
  ASSERT_EQ(0, chmod((runtime_dir + "/commands/runtime_health.sh").c_str(), 0755));

  const std::string stability_command =
      "TUNNEL_LIO_SKIP_WATCHDOG=1 " + runtime_dir +
      "/commands/runtime_stability_check.sh --samples 1 --interval 0";
  EXPECT_EQ(0, std::system(stability_command.c_str()));
  EXPECT_EQ(0, std::system(stability_command.c_str()));

  const std::string stability_csv =
      readFile(runtime_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(1u, countOccurrences(stability_csv, ",PASS,SKIP,"));

  const std::string stability_summary =
      readFile(runtime_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, stability_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos, stability_summary.find("samples=1"));
}

TEST(RuntimeOpsScript, RejectsManifestSentinelRuntimeArguments) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_sentinel_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_name_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name __DUPLICATE_KEY__ --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_name_command.c_str()));

  const std::string bad_session_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_fusion_timoo.launch"
      " --section-session-id missing --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_session_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsManifestSeparatorRuntimeArguments) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_separator_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_name_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name 'board;section_session_id=evil'"
      " --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_name_command.c_str()));

  const std::string bad_session_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_fusion_timoo.launch"
      " --section-session-id 'session;metrics_report=evil'"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_session_command.c_str()));

  const std::string bad_root_command =
      "bash \"" + script + "\" --root '" + std::string(root) +
      ";runtime_dir=evil'"
      " --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_root_command.c_str()));

  const std::string bad_workspace_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace '/opt/tunnel_lio/catkin_ws;runtime_dir=evil'"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_workspace_command.c_str()));

  const std::string bad_launch_package_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package 'mine_slam_bringup;runtime_dir=evil'"
      " --launch-file bringup_sensors.launch --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_launch_package_command.c_str()));

  const std::string bad_launch_file_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup"
      " --launch-file 'bringup_sensors.launch;runtime_dir=evil'"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_launch_file_command.c_str()));

  const std::string bad_watchdog_topic_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --watchdog-topic '/time/status;runtime_dir=evil'"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_watchdog_topic_command.c_str()));

  const std::string bad_cpu_set_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --cpu-set '2-5;runtime_dir=evil'"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_cpu_set_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsCsvSeparatorRuntimePathArguments) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_csv_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_root_command =
      "bash \"" + script + "\" --root \"" + std::string(root) +
      ",bad\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_root_command.c_str()));

  const std::string bad_name_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board,alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_name_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsManifestWhitespacePollutedRuntimeArguments) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_runtime_ops_manifest_ws_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_workspace_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace '/opt/tunnel_lio/catkin_ws '"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_workspace_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsPathTraversalRuntimeName) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_runtime_ops_path_segment_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_name_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name ../board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
	      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_name_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsAbsolutePathArgumentsWithDotSegments) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_runtime_ops_dot_absolute_path_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_root_command =
      "bash \"" + script + "\" --root \"" + std::string(root) +
      "/runtime/../escaped_runtime\" --name board_alpha"
      " --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_root_command.c_str()));
  EXPECT_FALSE(exists(std::string(root) + "/escaped_runtime"));

  const std::string bad_workspace_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_beta --workspace /opt/tunnel_lio/../catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_workspace_command.c_str()));
  EXPECT_FALSE(exists(std::string(root) + "/board_beta"));
}

TEST(RuntimeOpsScript, RejectsRootOnlyAbsolutePathArguments) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_runtime_ops_root_absolute_path_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_workspace_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_workspace_command.c_str()));
  EXPECT_FALSE(exists(std::string(root) + "/board_alpha"));
}

TEST(RuntimeOpsScript, RejectsShellMetacharRuntimeLaunchTokens) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_shell_token_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_session_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_fusion_timoo.launch"
      " --section-session-id '$(touch /tmp/tunnel_lio_bad_session_token)'"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_session_command.c_str()));

  const std::string bad_launch_package_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package 'mine_slam_bringup$(touch /tmp/tunnel_lio_bad_pkg_token)'"
      " --launch-file bringup_sensors.launch --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_launch_package_command.c_str()));

  const std::string bad_launch_file_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup"
      " --launch-file 'bringup_sensors.launch$(touch /tmp/tunnel_lio_bad_file_token)'"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_launch_file_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsShellMetacharRuntimeScriptLiterals) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_shell_literal_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_root_command =
      "bash \"" + script + "\" --root '" + std::string(root) +
      "/$(id)' --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_root_command.c_str()));

  const std::string bad_workspace_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace '/opt/tunnel_lio/$(id)'"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_workspace_command.c_str()));

  const std::string bad_watchdog_topic_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --watchdog-topic '/time/status$(id)'"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_watchdog_topic_command.c_str()));

  const std::string malformed_watchdog_topic_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --watchdog-topic '/time/status bad'"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(malformed_watchdog_topic_command.c_str()));

  const std::string bad_cpu_set_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --cpu-set '2-5$(id)' --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_cpu_set_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsDockerVolumeSeparatorRuntimePaths) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_runtime_ops_docker_volume_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_root_command =
      "bash \"" + script + "\" --root '" + std::string(root) +
      ":bad' --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_root_command.c_str()));

  const std::string bad_workspace_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace '/opt/tunnel_lio/catkin_ws:bad'"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_workspace_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsRelativeRuntimePaths) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_runtime_ops_relative_path_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_root_command =
      "cd \"" + std::string(root) + "\" && bash \"" + script +
      "\" --root relative_runtime_root --name board_alpha"
      " --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_root_command.c_str()));

  const std::string bad_workspace_command =
      "cd \"" + std::string(root) + "\" && bash \"" + script +
      "\" --root \"" + root + "\" --name board_alpha"
      " --workspace relative_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_workspace_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsMalformedRuntimeNumericArguments) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_numeric_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_min_free_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --min-free-gb '12;runtime_dir=evil'"
      " --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_min_free_command.c_str()));

  const std::string bad_log_retention_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --log-retention-days seven --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_log_retention_command.c_str()));

  const std::string bad_watchdog_timeout_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --watchdog-timeout 0 --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_watchdog_timeout_command.c_str()));
}

TEST(RuntimeOpsScript, RejectsMalformedCpuSetArgument) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_cpuset_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string bad_cpu_set_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --cpu-set two --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(bad_cpu_set_command.c_str()));

  const std::string reversed_cpu_set_range_command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_alpha --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_sensors.launch"
      " --cpu-set 5-2 --dry-run >/dev/null 2>&1";
  EXPECT_NE(0, std::system(reversed_cpu_set_range_command.c_str()));
}

TEST(RuntimeOpsScript, FusionRuntimeUsesRuntimeNameAsSectionSessionByDefault) {
  const char* script_env = std::getenv("RUNTIME_OPS_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RUNTIME_OPS_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_runtime_ops_fusion_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name board_fusion --workspace /opt/tunnel_lio/catkin_ws"
      " --launch-package mine_slam_bringup --launch-file bringup_fusion_timoo.launch"
      " --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string runtime_dir = std::string(root) + "/board_fusion";
  const std::string metadata = readFile(runtime_dir + "/runtime.env");
  EXPECT_NE(std::string::npos,
            metadata.find("launch=mine_slam_bringup bringup_fusion_timoo.launch"));
  EXPECT_NE(std::string::npos,
            metadata.find("section_session_id=board_fusion"));

  const std::string start_command =
      readFile(runtime_dir + "/commands/start_runtime.sh");
  EXPECT_NE(std::string::npos,
            start_command.find("roslaunch mine_slam_bringup bringup_fusion_timoo.launch section_session_id:=board_fusion"));

  const std::string service =
      readFile(runtime_dir + "/systemd/tunnel-lio.service");
  EXPECT_NE(std::string::npos,
            service.find("ExecStart=" + runtime_dir + "/commands/start_runtime.sh"));

  const std::string compose =
      readFile(runtime_dir + "/docker/docker-compose.yaml");
  EXPECT_NE(std::string::npos,
            compose.find("command: /runtime/commands/start_runtime.sh"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
