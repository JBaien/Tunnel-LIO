#include <gtest/gtest.h>

#include "section_manager/control_field_parser.h"

namespace section_manager {

TEST(ControlFieldParser, ParsesStrictDoubleFieldOrUsesFallback) {
  EXPECT_DOUBLE_EQ(12.5,
                   parseStrictDoubleFieldOr(
                       "chainage_m=12.5;section_sample=true",
                       "chainage_m",
                       1.0));
  EXPECT_DOUBLE_EQ(1.0,
                   parseStrictDoubleFieldOr(
                       "section_sample=true",
                       "chainage_m",
                       1.0));
}

TEST(ControlFieldParser, MatchesExactFieldName) {
  EXPECT_DOUBLE_EQ(10.0,
                   parseStrictDoubleFieldOr(
                       "last_chainage_m=99.0;chainage_m=10.0",
                       "chainage_m",
                       1.0));
}

TEST(ControlFieldParser, RejectsMalformedDoubleField) {
  EXPECT_DOUBLE_EQ(1.0,
                   parseStrictDoubleFieldOr(
                       "chainage_m=12.5junk",
                       "chainage_m",
                       1.0));
  EXPECT_DOUBLE_EQ(1.0,
                   parseStrictDoubleFieldOr(
                       "chainage_m= 12.5",
                       "chainage_m",
                       1.0));
  EXPECT_DOUBLE_EQ(1.0,
                   parseStrictDoubleFieldOr(
                       "chainage_m=nan",
                       "chainage_m",
                       1.0));
  EXPECT_DOUBLE_EQ(1.0,
                   parseStrictDoubleFieldOr(
                       "chainage_m=inf",
                       "chainage_m",
                       1.0));
}

TEST(ControlFieldParser, ParsesBoolFieldWithExactKey) {
  EXPECT_TRUE(parseStrictBoolFieldOr(
      "section_sample=true;chainage_m=12.5", "section_sample", false));
  EXPECT_FALSE(parseStrictBoolFieldOr(
      "last_section_sample=true;section_sample=false",
      "section_sample",
      true));
  EXPECT_FALSE(parseStrictBoolFieldOr(
      "section_sample=true;section_sample=false", "section_sample", false));
  EXPECT_TRUE(parseStrictBoolFieldOr(
      "section_sample=yes", "section_sample", true));
}

TEST(ControlFieldParser, ParsesTextFieldWithExactKey) {
  EXPECT_EQ("CUTTING_STATIC",
            parseTextFieldOr(
                "machine_state=CUTTING_STATIC;section_sample=true",
                "machine_state",
                "unknown"));
  EXPECT_EQ("IDLE_STATIC",
            parseTextFieldOr(
                "last_machine_state=FWD_MOVE;machine_state=IDLE_STATIC",
                "machine_state",
                "unknown"));
  EXPECT_EQ("unknown",
            parseTextFieldOr(
                "machine_state=IDLE_STATIC;machine_state=FWD_MOVE",
                "machine_state",
                "unknown"));
  EXPECT_EQ("unknown",
            parseTextFieldOr(
                "machine_state= IDLE_STATIC",
                "machine_state",
                "unknown"));
}

}  // namespace section_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
