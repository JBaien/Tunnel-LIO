#pragma once

#include <string>
#include <vector>

namespace slam_backend_manager {

struct StableMapEntry {
  std::string keyframe_id;
  double chainage_m = 0.0;
  std::string section_quality;
  double promoted_at = 0.0;
  bool loop_verified = false;
};

class StableMapLedger {
 public:
  explicit StableMapLedger(const std::string& path);

  void promote(const StableMapEntry& entry);
  const std::vector<StableMapEntry>& entries() const { return entries_; }

 private:
  void load();
  void save() const;

  std::string path_;
  std::vector<StableMapEntry> entries_;
};

}  // namespace slam_backend_manager
