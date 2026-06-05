#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <limits>

#include "slam_backend_manager/stable_map_store.h"

namespace slam_backend_manager {
namespace {

TEST(StableMapStore, PersistsPromotedKeyframesAtomically) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("stable_map_store_test_%%%%%%");
  boost::filesystem::create_directories(root);
  const std::string path = (root / "stable_map.json").string();

  StableMapLedger ledger(path);
  ledger.promote(StableMapEntry{"kf_1", 12.5, "A", 100.0, true});

  StableMapLedger loaded(path);
  ASSERT_EQ(1u, loaded.entries().size());
  EXPECT_EQ("kf_1", loaded.entries()[0].keyframe_id);
  EXPECT_DOUBLE_EQ(12.5, loaded.entries()[0].chainage_m);
  EXPECT_TRUE(loaded.entries()[0].loop_verified);
  EXPECT_FALSE(boost::filesystem::exists(path + ".tmp"));
  boost::filesystem::remove_all(root);
}

TEST(StableMapStore, ReplacesExistingKeyframeEntry) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("stable_map_store_test_%%%%%%");
  boost::filesystem::create_directories(root);
  const std::string path = (root / "stable_map.json").string();

  StableMapLedger ledger(path);
  ledger.promote(StableMapEntry{"kf_1", 10.0, "B", 100.0, false});
  ledger.promote(StableMapEntry{"kf_1", 11.0, "A", 101.0, true});

  StableMapLedger loaded(path);
  ASSERT_EQ(1u, loaded.entries().size());
  EXPECT_EQ("A", loaded.entries()[0].section_quality);
  EXPECT_DOUBLE_EQ(11.0, loaded.entries()[0].chainage_m);
  boost::filesystem::remove_all(root);
}

TEST(StableMapStore, SkipsIncompleteLedgerEntries) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("stable_map_store_test_%%%%%%");
  boost::filesystem::create_directories(root);
  const std::string path = (root / "stable_map.json").string();
  {
    std::ofstream output(path.c_str());
    output << "{\n"
           << "  \"entries\": [\n"
           << "    {\n"
           << "      \"keyframe_id\": \"kf_good\",\n"
           << "      \"chainage_m\": 12.5,\n"
           << "      \"section_quality\": \"A\",\n"
           << "      \"promoted_at\": 100.0,\n"
           << "      \"loop_verified\": true\n"
           << "    },\n"
           << "    {\n"
           << "      \"keyframe_id\": \"kf_missing_chainage\",\n"
           << "      \"section_quality\": \"A\",\n"
           << "      \"promoted_at\": 101.0,\n"
           << "      \"loop_verified\": true\n"
           << "    },\n"
           << "    {\n"
           << "      \"keyframe_id\": \"kf_missing_quality\",\n"
           << "      \"chainage_m\": 13.0,\n"
           << "      \"promoted_at\": 102.0,\n"
           << "      \"loop_verified\": true\n"
           << "    }\n"
           << "  ]\n"
           << "}\n";
  }

  StableMapLedger loaded(path);

  ASSERT_EQ(1u, loaded.entries().size());
  EXPECT_EQ("kf_good", loaded.entries()[0].keyframe_id);
  EXPECT_DOUBLE_EQ(12.5, loaded.entries()[0].chainage_m);
  EXPECT_EQ("A", loaded.entries()[0].section_quality);
  boost::filesystem::remove_all(root);
}

TEST(StableMapStore, SkipsLedgerEntriesWithPollutedKeyframeIds) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("stable_map_store_test_%%%%%%");
  boost::filesystem::create_directories(root);
  const std::string path = (root / "stable_map.json").string();
  {
    std::ofstream output(path.c_str());
    output << "{\n"
           << "  \"entries\": [\n"
           << "    {\n"
           << "      \"keyframe_id\": \"kf_good-1\",\n"
           << "      \"chainage_m\": 12.5,\n"
           << "      \"section_quality\": \"A\",\n"
           << "      \"promoted_at\": 100.0,\n"
           << "      \"loop_verified\": true\n"
           << "    },\n"
           << "    {\n"
           << "      \"keyframe_id\": \"kf_bad;field=spoof\",\n"
           << "      \"chainage_m\": 13.0,\n"
           << "      \"section_quality\": \"A\",\n"
           << "      \"promoted_at\": 101.0,\n"
           << "      \"loop_verified\": true\n"
           << "    },\n"
           << "    {\n"
           << "      \"keyframe_id\": \"..\",\n"
           << "      \"chainage_m\": 14.0,\n"
           << "      \"section_quality\": \"A\",\n"
           << "      \"promoted_at\": 102.0,\n"
           << "      \"loop_verified\": true\n"
           << "    }\n"
           << "  ]\n"
           << "}\n";
  }

  StableMapLedger loaded(path);

  ASSERT_EQ(1u, loaded.entries().size());
  EXPECT_EQ("kf_good-1", loaded.entries()[0].keyframe_id);
  boost::filesystem::remove_all(root);
}

TEST(StableMapStore, IgnoresInvalidPromotions) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("stable_map_store_test_%%%%%%");
  boost::filesystem::create_directories(root);
  const std::string path = (root / "stable_map.json").string();

  StableMapLedger ledger(path);
  ledger.promote(StableMapEntry{"kf_good", 12.5, "A", 100.0, true});
  ledger.promote(StableMapEntry{"kf_nan", std::numeric_limits<double>::infinity(),
                                "A", 101.0, true});
  ledger.promote(StableMapEntry{"kf_bad_quality", 13.0, "missing", 102.0, true});
  ledger.promote(StableMapEntry{"kf_good", 14.0, "", 103.0, true});

  StableMapLedger loaded(path);
  ASSERT_EQ(1u, loaded.entries().size());
  EXPECT_EQ("kf_good", loaded.entries()[0].keyframe_id);
  EXPECT_DOUBLE_EQ(12.5, loaded.entries()[0].chainage_m);
  EXPECT_EQ("A", loaded.entries()[0].section_quality);
  boost::filesystem::remove_all(root);
}

TEST(StableMapStore, RejectsPollutedPromotionKeyframeIdsWithoutOverwritingExistingEntry) {
  const boost::filesystem::path root =
      boost::filesystem::temp_directory_path() /
      boost::filesystem::unique_path("stable_map_store_test_%%%%%%");
  boost::filesystem::create_directories(root);
  const std::string path = (root / "stable_map.json").string();

  StableMapLedger ledger(path);
  ledger.promote(StableMapEntry{"kf_good", 12.5, "A", 100.0, true});
  ledger.promote(StableMapEntry{"kf_good;quality=A", 20.0, "A", 101.0, true});
  ledger.promote(StableMapEntry{"..", 30.0, "A", 102.0, true});

  StableMapLedger loaded(path);
  ASSERT_EQ(1u, loaded.entries().size());
  EXPECT_EQ("kf_good", loaded.entries()[0].keyframe_id);
  EXPECT_DOUBLE_EQ(12.5, loaded.entries()[0].chainage_m);
  boost::filesystem::remove_all(root);
}

}  // namespace
}  // namespace slam_backend_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
