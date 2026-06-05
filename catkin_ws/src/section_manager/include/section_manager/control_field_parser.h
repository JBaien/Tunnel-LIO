#ifndef SECTION_MANAGER_CONTROL_FIELD_PARSER_H
#define SECTION_MANAGER_CONTROL_FIELD_PARSER_H

#include <string>

namespace section_manager {

double parseStrictDoubleFieldOr(const std::string& text,
                                const std::string& key,
                                double fallback);
bool parseStrictBoolFieldOr(const std::string& text,
                            const std::string& key,
                            bool fallback);
std::string parseTextFieldOr(const std::string& text,
                             const std::string& key,
                             const std::string& fallback);

}  // namespace section_manager

#endif  // SECTION_MANAGER_CONTROL_FIELD_PARSER_H
