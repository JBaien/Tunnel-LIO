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

}  // namespace

TEST(ValidationLaunchContract, ReportNodeShutsDownRoslaunchWhenComplete) {
  const std::string launch = readFile(VALIDATION_REPORT_LAUNCH_PATH);

  ASSERT_NE(std::string::npos, launch.find("validation_report_node"));
  EXPECT_NE(std::string::npos, launch.find("required=\"true\""));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
