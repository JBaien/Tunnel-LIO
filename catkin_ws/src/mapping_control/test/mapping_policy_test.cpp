#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mapping_control/policy.h"

TEST(MappingPolicy, StaticAndConflictGating) {
  mapping_control::MappingPolicyConfig config;
  config.section_spacing_m = 1.0;
  config.control_anchor_spacing_m = 50.0;

  const mapping_control::ControlDecision cutting =
      mapping_control::decideMappingControl(mapping_control::CUTTING_STATIC, 0.2, 1.0, 10.0, 9.0, config);
  EXPECT_EQ(mapping_control::FREEZE, cutting.action);
  EXPECT_FALSE(cutting.stable_map_write);

  const mapping_control::ControlDecision conflict =
      mapping_control::decideMappingControl(mapping_control::CONFLICT, 0.2, 1.0, 10.0, 9.0, config);
  EXPECT_EQ(mapping_control::REJECT, conflict.action);

  const mapping_control::ControlDecision commanded_no_displacement =
      mapping_control::decideMappingControl(mapping_control::CMD_MOVE_NO_DISP, 0.2, 1.0, 10.0, 9.0, config);
  EXPECT_EQ(mapping_control::REJECT, commanded_no_displacement.action);
  EXPECT_FALSE(commanded_no_displacement.stable_map_write);
  EXPECT_FALSE(commanded_no_displacement.section_sample);
  EXPECT_EQ("commanded_motion_without_displacement", commanded_no_displacement.reason);
}

TEST(MappingPolicy, ForwardMotionAcceptsOrLimitsLength) {
  mapping_control::MappingPolicyConfig config;
  config.section_spacing_m = 1.0;
  config.control_anchor_spacing_m = 50.0;

  const mapping_control::ControlDecision nominal =
      mapping_control::decideMappingControl(mapping_control::FWD_MOVE, 0.2, 1.0, 10.0, 9.5, config);
  EXPECT_EQ(mapping_control::ACCEPT, nominal.action);
  EXPECT_TRUE(nominal.stable_map_write);

  const mapping_control::ControlDecision weak =
      mapping_control::decideMappingControl(mapping_control::FWD_MOVE, 0.2, 0.1, 10.0, 9.5, config);
  EXPECT_EQ(mapping_control::LIMIT, weak.action);
  EXPECT_LE(std::fabs(weak.accepted_delta_m), config.max_weak_delta_m);
}

TEST(MappingPolicy, SectionQualityAndAnchorPolicy) {
  mapping_control::MappingPolicyConfig config;
  config.section_spacing_m = 1.0;
  config.control_anchor_spacing_m = 50.0;

  EXPECT_EQ("A", mapping_control::gradeSection(0.95, 20.0));
  EXPECT_EQ("B", mapping_control::gradeSection(0.8, 35.0));
  EXPECT_EQ("C", mapping_control::gradeSection(0.5, 50.0));
  EXPECT_TRUE(mapping_control::needsControlAnchor(50.0, 0.0, config));
}

TEST(MappingPolicy, RejectsInvalidInputsAndConfigFailClosed) {
  mapping_control::MappingPolicyConfig config;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  const mapping_control::ControlDecision invalid_delta =
      mapping_control::decideMappingControl(mapping_control::FWD_MOVE, nan, 1.0, 10.0, 9.5, config);
  EXPECT_EQ(mapping_control::REJECT, invalid_delta.action);
  EXPECT_TRUE(std::isfinite(invalid_delta.accepted_delta_m));
  EXPECT_FALSE(invalid_delta.stable_map_write);
  EXPECT_FALSE(invalid_delta.section_sample);

  mapping_control::MappingPolicyConfig invalid_config;
  invalid_config.max_nominal_delta_m = nan;
  const mapping_control::ControlDecision invalid_policy =
      mapping_control::decideMappingControl(mapping_control::FWD_MOVE, 0.2, 1.0, 10.0, 9.5, invalid_config);
  EXPECT_EQ(mapping_control::REJECT, invalid_policy.action);
  EXPECT_FALSE(invalid_policy.stable_map_write);

  const mapping_control::ControlDecision unknown_state =
      mapping_control::decideMappingControl("UNKNOWN", 0.2, 1.0, 10.0, 9.5, config);
  EXPECT_EQ(mapping_control::REJECT, unknown_state.action);

  EXPECT_FALSE(mapping_control::needsControlAnchor(nan, 0.0, config));
  EXPECT_EQ("C", mapping_control::gradeSection(0.95, -1.0));
  EXPECT_EQ("C", mapping_control::gradeSection(nan, 20.0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
