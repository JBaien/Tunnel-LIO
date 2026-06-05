#include <gtest/gtest.h>

#include <dirent.h>
#include <sys/stat.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

bool isDirectory(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool hasSuffix(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string readFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

void collectContractFiles(const std::string& dir, std::vector<std::string>* files) {
  DIR* handle = opendir(dir.c_str());
  ASSERT_NE(nullptr, handle) << dir;
  while (dirent* entry = readdir(handle)) {
    const std::string name(entry->d_name);
    if (name == "." || name == "..") {
      continue;
    }
    const std::string path = dir + "/" + name;
    if (isDirectory(path)) {
      collectContractFiles(path, files);
      continue;
    }
    if (name == "CMakeLists.txt" || name == "package.xml" ||
        hasSuffix(name, ".launch")) {
      files->push_back(path);
    }
  }
  closedir(handle);
}

}  // namespace

TEST(RosLanguageContract, RuntimeRosNodesUseCppRoscpp) {
  const std::string src_dir(CATKIN_WORKSPACE_SRC_DIR);
  std::vector<std::string> files;
  collectContractFiles(src_dir, &files);
  ASSERT_FALSE(files.empty());

  for (const std::string& file : files) {
    const std::string content = readFile(file);
    EXPECT_EQ(std::string::npos, content.find("rospy")) << file;
    EXPECT_EQ(std::string::npos, content.find("catkin_install_python")) << file;
    EXPECT_EQ(std::string::npos, content.find("catkin_add_nosetests")) << file;
    EXPECT_EQ(std::string::npos, content.find("type=\"*.py\"")) << file;
    EXPECT_EQ(std::string::npos, content.find("type='*.py'")) << file;
    EXPECT_EQ(std::string::npos, content.find(".py\"")) << file;
    EXPECT_EQ(std::string::npos, content.find(".py'")) << file;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
