#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

std::string makeTempDir() {
  std::string path = "/tmp/timoo_gen_calibration_test_XXXXXX";
  std::vector<char> writable(path.begin(), path.end());
  writable.push_back('\0');
  char* created = mkdtemp(writable.data());
  return created == nullptr ? std::string() : std::string(created);
}

void writeFile(const std::string& path, const std::string& text) {
  std::ofstream stream(path.c_str());
  stream << text;
}

bool fileExists(const std::string& path) {
  struct stat status;
  return stat(path.c_str(), &status) == 0;
}

std::string readFile(const std::string& path) {
  std::ifstream stream(path.c_str());
  std::ostringstream content;
  content << stream.rdbuf();
  return content.str();
}

int runGenerator(const std::string& xml, const std::string& yaml, const std::string& log) {
  const std::string script = std::string(GEN_CALIBRATION_SCRIPT_DIR) + "/gen_calibration.py";
  const std::string command =
      "python3 \"" + script + "\" \"" + xml + "\" \"" + yaml + "\" > \"" + log + "\" 2>&1";
  return std::system(command.c_str());
}

std::string validOneLaserXml() {
  return "<root><DB>"
         "<enabled_><item>1</item></enabled_>"
         "<distLSB_>0.2</distLSB_>"
         "<points_><item><px>"
         "<id_>0</id_>"
         "<rotCorrection_>0.0</rotCorrection_>"
         "<vertCorrection_>1.0</vertCorrection_>"
         "<distCorrection_>0.0</distCorrection_>"
         "<distCorrectionX_>0.0</distCorrectionX_>"
         "<distCorrectionY_>0.0</distCorrectionY_>"
         "<vertOffsetCorrection_>0.0</vertOffsetCorrection_>"
         "<horizOffsetCorrection_>0.0</horizOffsetCorrection_>"
         "<focalDistance_>10.0</focalDistance_>"
         "<focalSlope_>1.0</focalSlope_>"
         "</px></item></points_>"
         "</DB></root>";
}

}  // namespace

TEST(GenCalibrationScript, ConvertsCompleteEnabledLaser) {
  const std::string dir = makeTempDir();
  ASSERT_FALSE(dir.empty());
  const std::string xml = dir + "/valid.xml";
  const std::string yaml = dir + "/valid.yaml";
  const std::string log = dir + "/valid.log";
  writeFile(xml, validOneLaserXml());

  EXPECT_EQ(0, runGenerator(xml, yaml, log));
  ASSERT_TRUE(fileExists(yaml)) << readFile(log);
  const std::string output = readFile(yaml);
  EXPECT_NE(std::string::npos, output.find("num_lasers: 1"));
  EXPECT_NE(std::string::npos, output.find("laser_id: 0"));
  EXPECT_NE(std::string::npos, output.find("vert_correction"));
}

TEST(GenCalibrationScript, RejectsEnabledLaserMissingRequiredField) {
  const std::string dir = makeTempDir();
  ASSERT_FALSE(dir.empty());
  const std::string xml = dir + "/missing.xml";
  const std::string yaml = dir + "/missing.yaml";
  const std::string log = dir + "/missing.log";
  std::string broken = validOneLaserXml();
  const std::string missing = "<vertCorrection_>1.0</vertCorrection_>";
  broken.erase(broken.find(missing), missing.size());
  writeFile(xml, broken);

  EXPECT_NE(0, runGenerator(xml, yaml, log));
  EXPECT_FALSE(fileExists(yaml)) << readFile(yaml);
  EXPECT_NE(std::string::npos, readFile(log).find("missing required calibration field"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
