#include "section_manager/control_field_parser.h"

#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>

namespace section_manager {
namespace {

std::string trimKey(const std::string& value) {
  const std::string whitespace = " \t\r\n";
  const std::size_t first = value.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return "";
  }
  const std::size_t last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1);
}

bool parseStrictDoubleValue(const std::string& value, double* parsed_value) {
  if (value.empty()) {
    return false;
  }
  if (std::isspace(static_cast<unsigned char>(value[0])) != 0) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || errno == ERANGE ||
      !std::isfinite(parsed)) {
    return false;
  }
  *parsed_value = parsed;
  return true;
}

bool lookupUniqueFieldValue(const std::string& text,
                            const std::string& key,
                            std::string* value) {
  std::stringstream stream(text);
  std::string token;
  bool found = false;
  while (std::getline(stream, token, ';')) {
    const std::size_t split = token.find('=');
    if (split == std::string::npos) {
      continue;
    }
    if (trimKey(token.substr(0, split)) != key) {
      continue;
    }
    if (found) {
      return false;
    }
    found = true;
    *value = token.substr(split + 1);
  }
  return found;
}

bool validTextValue(const std::string& value) {
  return !value.empty() &&
         std::isspace(static_cast<unsigned char>(value[0])) == 0 &&
         value != "missing" && value != "__DUPLICATE_KEY__" &&
         value.find(';') == std::string::npos &&
         value.find('\n') == std::string::npos &&
         value.find('\r') == std::string::npos;
}

}  // namespace

double parseStrictDoubleFieldOr(const std::string& text,
                                const std::string& key,
                                const double fallback) {
  std::string value;
  double parsed = fallback;
  if (!lookupUniqueFieldValue(text, key, &value)) {
    return fallback;
  }
  if (!parseStrictDoubleValue(value, &parsed)) {
    return fallback;
  }
  return parsed;
}

bool parseStrictBoolFieldOr(const std::string& text,
                            const std::string& key,
                            const bool fallback) {
  std::string value;
  if (!lookupUniqueFieldValue(text, key, &value)) {
    return fallback;
  }
  if (value == "true" || value == "True") {
    return true;
  }
  if (value == "false" || value == "False") {
    return false;
  }
  return fallback;
}

std::string parseTextFieldOr(const std::string& text,
                             const std::string& key,
                             const std::string& fallback) {
  std::string value;
  if (!lookupUniqueFieldValue(text, key, &value) || !validTextValue(value)) {
    return fallback;
  }
  return value;
}

}  // namespace section_manager
