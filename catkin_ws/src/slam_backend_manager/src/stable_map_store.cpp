#include "slam_backend_manager/stable_map_store.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace slam_backend_manager {
namespace {

bool validQuality(const std::string& quality) {
  return quality == "A" || quality == "B" || quality == "C";
}

bool validKeyframeId(const std::string& value) {
  if (value.empty() || value == "." || value == "..") {
    return false;
  }
  for (const char character : value) {
    const bool uppercase = character >= 'A' && character <= 'Z';
    const bool lowercase = character >= 'a' && character <= 'z';
    const bool digit = character >= '0' && character <= '9';
    if (!uppercase && !lowercase && !digit && character != '_' &&
        character != '-' && character != '.') {
      return false;
    }
  }
  return true;
}

bool validEntry(const StableMapEntry& entry) {
  return validKeyframeId(entry.keyframe_id) && std::isfinite(entry.chainage_m) &&
         validQuality(entry.section_quality) && std::isfinite(entry.promoted_at);
}

bool readRequiredString(const boost::property_tree::ptree& data,
                        const std::string& key,
                        std::string* value) {
  try {
    *value = data.get<std::string>(key);
  } catch (const boost::property_tree::ptree_error&) {
    return false;
  }
  return !value->empty();
}

bool readRequiredFiniteDouble(const boost::property_tree::ptree& data,
                              const std::string& key,
                              double* value) {
  try {
    *value = data.get<double>(key);
  } catch (const boost::property_tree::ptree_error&) {
    return false;
  }
  return std::isfinite(*value);
}

bool readLedgerEntry(const boost::property_tree::ptree& data,
                     StableMapEntry* entry) {
  StableMapEntry parsed;
  if (!readRequiredString(data, "keyframe_id", &parsed.keyframe_id) ||
      !readRequiredFiniteDouble(data, "chainage_m", &parsed.chainage_m) ||
      !readRequiredString(data, "section_quality", &parsed.section_quality) ||
      !readRequiredFiniteDouble(data, "promoted_at", &parsed.promoted_at)) {
    return false;
  }
  try {
    parsed.loop_verified = data.get<bool>("loop_verified", false);
  } catch (const boost::property_tree::ptree_error&) {
    return false;
  }
  if (!validEntry(parsed)) {
    return false;
  }
  *entry = parsed;
  return true;
}

}  // namespace

StableMapLedger::StableMapLedger(const std::string& path) : path_(path) {
  load();
}

void StableMapLedger::promote(const StableMapEntry& entry) {
  if (!validEntry(entry)) {
    return;
  }
  entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                                [&entry](const StableMapEntry& existing) {
                                  return existing.keyframe_id == entry.keyframe_id;
                                }),
                 entries_.end());
  entries_.push_back(entry);
  std::sort(entries_.begin(), entries_.end(),
            [](const StableMapEntry& lhs, const StableMapEntry& rhs) {
              return lhs.chainage_m < rhs.chainage_m;
            });
  save();
}

void StableMapLedger::load() {
  entries_.clear();
  if (!boost::filesystem::is_regular_file(path_)) {
    return;
  }
  boost::property_tree::ptree root;
  try {
    boost::property_tree::read_json(path_, root);
  } catch (const boost::property_tree::ptree_error&) {
    return;
  }
  const boost::property_tree::ptree empty;
  for (const auto& item : root.get_child("entries", empty)) {
    const boost::property_tree::ptree& data = item.second;
    StableMapEntry entry;
    if (readLedgerEntry(data, &entry)) {
      entries_.push_back(entry);
    }
  }
}

void StableMapLedger::save() const {
  const boost::filesystem::path output_path(path_);
  if (output_path.has_parent_path()) {
    boost::filesystem::create_directories(output_path.parent_path());
  }

  boost::property_tree::ptree entries_tree;
  for (const StableMapEntry& entry : entries_) {
    if (!validEntry(entry)) {
      continue;
    }
    boost::property_tree::ptree item;
    item.put("keyframe_id", entry.keyframe_id);
    item.put("chainage_m", entry.chainage_m);
    item.put("section_quality", entry.section_quality);
    item.put("promoted_at", entry.promoted_at);
    item.put("loop_verified", entry.loop_verified);
    entries_tree.push_back(std::make_pair("", item));
  }

  boost::property_tree::ptree root;
  root.add_child("entries", entries_tree);
  const std::string tmp_path = path_ + ".tmp";
  boost::property_tree::write_json(tmp_path, root);
  boost::filesystem::rename(tmp_path, path_);
}

}  // namespace slam_backend_manager
