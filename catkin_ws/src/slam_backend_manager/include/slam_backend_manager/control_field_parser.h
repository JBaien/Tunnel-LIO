#ifndef SLAM_BACKEND_MANAGER_CONTROL_FIELD_PARSER_H
#define SLAM_BACKEND_MANAGER_CONTROL_FIELD_PARSER_H

#include <string>

namespace slam_backend_manager {

double parseStrictDoubleFieldOr(const std::string& text,
                                const std::string& key,
                                double fallback);
bool parseStrictBoolFieldOr(const std::string& text,
                            const std::string& key,
                            bool fallback);
std::string parseTextFieldOr(const std::string& text,
                             const std::string& key,
                             const std::string& fallback);

}  // namespace slam_backend_manager

#endif  // SLAM_BACKEND_MANAGER_CONTROL_FIELD_PARSER_H
