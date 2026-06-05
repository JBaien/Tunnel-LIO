#include "mine_slam_bringup/event_dump.h"

#include <sys/stat.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

namespace mine_slam_bringup {
namespace {

bool ensureDirectory(const std::string& path, std::string* error) {
  if (path.empty()) {
    *error = "directory path is empty";
    return false;
  }
  std::string current;
  if (path[0] == '/') {
    current = "/";
  }
  std::size_t start = path[0] == '/' ? 1u : 0u;
  while (start <= path.size()) {
    const std::size_t slash = path.find('/', start);
    const std::string part =
        slash == std::string::npos ? path.substr(start) : path.substr(start, slash - start);
    if (!part.empty()) {
      if (!current.empty() && current[current.size() - 1] != '/') {
        current += "/";
      }
      current += part;
      if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
        *error = std::string("failed to create directory ") + current + ": " + std::strerror(errno);
        return false;
      }
    }
    if (slash == std::string::npos) {
      break;
    }
    start = slash + 1;
  }
  return true;
}

std::string trim(const std::string& text) {
  std::size_t begin = 0;
  while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin]))) {
    ++begin;
  }
  std::size_t end = text.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1]))) {
    --end;
  }
  return text.substr(begin, end - begin);
}

std::map<std::string, std::string> readMetadata(const std::string& path) {
  std::map<std::string, std::string> values;
  std::ifstream input(path.c_str());
  std::string line;
  while (std::getline(input, line)) {
    const std::size_t equals = line.find('=');
    if (equals == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, equals));
    const std::string value = line.substr(equals + 1);
    if (!key.empty()) {
      const auto inserted = values.emplace(key, value);
      if (!inserted.second) {
        inserted.first->second = "__DUPLICATE_KEY__";
      }
    }
  }
  return values;
}

bool validMetadataValue(const std::string& value) {
  return !value.empty() && value == trim(value) && value != "missing" &&
         value != "__DUPLICATE_KEY__" &&
         value.find(';') == std::string::npos &&
         value.find('\r') == std::string::npos &&
         value.find('\n') == std::string::npos &&
         value.find('"') == std::string::npos &&
         value.find('$') == std::string::npos &&
         value.find('`') == std::string::npos &&
         value.find('\\') == std::string::npos;
}

bool validRequestValue(const std::string& value) {
  return validMetadataValue(value);
}

bool hasDotPathSegment(const std::string& value) {
  std::size_t start = 0;
  while (start <= value.size()) {
    const std::size_t slash = value.find('/', start);
    const std::string segment =
        slash == std::string::npos ? value.substr(start) : value.substr(start, slash - start);
    if (segment == "." || segment == "..") {
      return true;
    }
    if (slash == std::string::npos) {
      break;
    }
    start = slash + 1;
  }
  return false;
}

bool validAbsolutePathValue(const std::string& value) {
  return validRequestValue(value) && value[0] == '/' && value != "/" &&
         !hasDotPathSegment(value);
}

bool validEventId(const std::string& value) {
  if (!validRequestValue(value) || value == "." || value == "..") {
    return false;
  }
  for (std::size_t i = 0; i < value.size(); ++i) {
    const char c = value[i];
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.')) {
      return false;
    }
  }
  return true;
}

bool lookupMetadataValue(const std::map<std::string, std::string>& metadata,
                         const std::string& key,
                         std::string* value,
                         std::string* error) {
  const auto iter = metadata.find(key);
  if (iter == metadata.end() || !validMetadataValue(iter->second)) {
    *error = "metadata " + key + " is missing or invalid";
    return false;
  }
  *value = iter->second;
  return true;
}

bool lookupMetadataPath(const std::map<std::string, std::string>& metadata,
                        const std::string& key,
                        std::string* value,
                        std::string* error) {
  const auto iter = metadata.find(key);
  if (iter == metadata.end() || !validAbsolutePathValue(iter->second)) {
    *error = "metadata " + key + " is missing or invalid";
    return false;
  }
  *value = iter->second;
  return true;
}

bool lookupMetadataToken(const std::map<std::string, std::string>& metadata,
                         const std::string& key,
                         std::string* value,
                         std::string* error) {
  const auto iter = metadata.find(key);
  if (iter == metadata.end() || !validEventId(iter->second)) {
    *error = "metadata " + key + " is missing or invalid";
    return false;
  }
  *value = iter->second;
  return true;
}

std::string formatDouble(double value) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream << std::setprecision(3) << value;
  return stream.str();
}

std::string safeId(std::string id) {
  for (std::size_t i = 0; i < id.size(); ++i) {
    const char c = id[i];
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.')) {
      id[i] = '_';
    }
  }
  return id.empty() ? "event" : id;
}

bool writeText(const std::string& path, const std::string& text, std::string* error) {
  std::ofstream output(path.c_str());
  if (!output) {
    *error = "failed to open " + path;
    return false;
  }
  output << text;
  return true;
}

void chmodExecutable(const std::string& path) {
  chmod(path.c_str(), 0755);
}

}  // namespace

EventDumpResult createEventDump(const EventDumpRequest& request) {
  EventDumpResult result;
  result.event_id = request.event_id;
  if (!validAbsolutePathValue(request.session_dir)) {
    result.message = "session_dir is missing or invalid";
    return result;
  }
  if (!validAbsolutePathValue(request.output_root)) {
    result.message = "output_root is missing or invalid";
    return result;
  }
  if (!validEventId(request.event_id)) {
    result.message = "event_id is missing or invalid";
    return result;
  }
  if (!validRequestValue(request.reason)) {
    result.message = "reason is missing or invalid";
    return result;
  }
  if (!std::isfinite(request.event_time_s)) {
    result.message = "event_time_s must be finite";
    return result;
  }
  if (request.event_time_s < 0.0) {
    result.message = "event_time_s must be non-negative";
    return result;
  }
  if (!std::isfinite(request.window_before_s) || !std::isfinite(request.window_after_s)) {
    result.message = "event windows must be finite";
    return result;
  }
  if (request.window_before_s < 0.0 || request.window_after_s < 0.0) {
    result.message = "event windows must be non-negative";
    return result;
  }

  const std::map<std::string, std::string> metadata =
      readMetadata(request.session_dir + "/metadata.env");
  std::string error;
  std::string bag_path;
  std::string pcap_path;
  std::string session_name;
  std::string topics;
  if (!lookupMetadataPath(metadata, "bag_path", &bag_path, &error) ||
      !lookupMetadataPath(metadata, "pcap_path", &pcap_path, &error) ||
      !lookupMetadataToken(metadata, "session_name", &session_name, &error) ||
      !lookupMetadataValue(metadata, "topics", &topics, &error)) {
    result.message = error;
    return result;
  }

  const double start_time = std::max(0.0, request.event_time_s - request.window_before_s);
  const double end_time = request.event_time_s + request.window_after_s;
  if (!std::isfinite(start_time) || !std::isfinite(end_time)) {
    result.message = "event time window must be finite";
    return result;
  }
  const std::string event_id = safeId(request.event_id);
  const std::string event_dir = request.output_root + "/" + event_id;
  if (!ensureDirectory(event_dir + "/commands", &error) ||
      !ensureDirectory(event_dir + "/bags", &error) ||
      !ensureDirectory(event_dir + "/pcap", &error) ||
      !ensureDirectory(event_dir + "/snapshots", &error)) {
    result.message = error;
    return result;
  }

  std::ostringstream manifest;
  manifest << "event_id=" << event_id << "\n"
           << "source_event_id=" << request.event_id << "\n"
           << "reason=" << request.reason << "\n"
           << "session_name=" << session_name << "\n"
           << "session_dir=" << request.session_dir << "\n"
           << "event_time_s=" << formatDouble(request.event_time_s) << "\n"
           << "start_time_s=" << formatDouble(start_time) << "\n"
           << "end_time_s=" << formatDouble(end_time) << "\n"
           << "bag_path=" << bag_path << "\n"
           << "pcap_path=" << pcap_path << "\n"
           << "topics=" << topics << "\n";
  if (!writeText(event_dir + "/event_manifest.env", manifest.str(), &error)) {
    result.message = error;
    return result;
  }

  std::ostringstream bag_command;
  bag_command << "#!/usr/bin/env bash\n"
              << "set -euo pipefail\n"
              << "mkdir -p \"" << event_dir << "/bags\"\n"
              << "rosbag filter \"" << bag_path << "\" \"" << event_dir
              << "/bags/" << event_id << ".bag\" 't.to_sec() >= "
              << formatDouble(start_time) << " and t.to_sec() <= "
              << formatDouble(end_time) << "'\n";
  const std::string bag_script = event_dir + "/commands/filter_rosbag.sh";
  if (!writeText(bag_script, bag_command.str(), &error)) {
    result.message = error;
    return result;
  }
  chmodExecutable(bag_script);

  std::ostringstream pcap_command;
  pcap_command << "#!/usr/bin/env bash\n"
               << "set -euo pipefail\n"
               << "mkdir -p \"" << event_dir << "/pcap\"\n"
               << "editcap -A \"${PCAP_START_TIME:-" << formatDouble(start_time)
               << "}\" -B \"${PCAP_END_TIME:-" << formatDouble(end_time)
               << "}\" \"" << pcap_path << "\" \"" << event_dir << "/pcap/"
               << event_id << ".pcap\"\n";
  const std::string pcap_script = event_dir + "/commands/extract_pcap_window.sh";
  if (!writeText(pcap_script, pcap_command.str(), &error)) {
    result.message = error;
    return result;
  }
  chmodExecutable(pcap_script);

  result.success = true;
  result.message = "event dump plan created";
  result.event_id = event_id;
  result.event_dir = event_dir;
  return result;
}

}  // namespace mine_slam_bringup
