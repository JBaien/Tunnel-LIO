#include <gtest/gtest.h>

#include <limits>

#include "machine_state_manager/state_logic.h"

TEST(MachineStateLogic, ClassifiesStaticStates) {
  machine_state_manager::MachineStateThresholds thresholds;

  EXPECT_EQ(machine_state_manager::IDLE_STATIC,
            machine_state_manager::classifyMachineState(machine_state_manager::MachineSignals(), thresholds));

  machine_state_manager::MachineSignals cutting;
  cutting.cutting_on = true;
  cutting.imu_vibration = 1.0;
  EXPECT_EQ(machine_state_manager::CUTTING_STATIC,
            machine_state_manager::classifyMachineState(cutting, thresholds));
}

TEST(MachineStateLogic, MotionStatesRequireLidarConfirmation) {
  machine_state_manager::MachineStateThresholds thresholds;
  machine_state_manager::MachineSignals forward;
  forward.left_track_speed = 0.4;
  forward.right_track_speed = 0.4;
  forward.lidar_speed = 0.2;
  EXPECT_EQ(machine_state_manager::FWD_MOVE, machine_state_manager::classifyMachineState(forward, thresholds));

  machine_state_manager::MachineSignals reverse;
  reverse.left_track_speed = -0.4;
  reverse.right_track_speed = -0.4;
  reverse.lidar_speed = 0.2;
  EXPECT_EQ(machine_state_manager::REV_MOVE, machine_state_manager::classifyMachineState(reverse, thresholds));

  forward.lidar_speed = 0.0;
  EXPECT_EQ(machine_state_manager::CMD_MOVE_NO_DISP,
            machine_state_manager::classifyMachineState(forward, thresholds));
}

TEST(MachineStateLogic, TurningAndMapWriteGating) {
  machine_state_manager::MachineStateThresholds thresholds;
  machine_state_manager::MachineSignals turning;
  turning.left_track_speed = 0.4;
  turning.right_track_speed = -0.2;
  turning.lidar_speed = 0.2;

  const std::string state = machine_state_manager::classifyMachineState(turning, thresholds);

  EXPECT_EQ(machine_state_manager::TURNING, state);
  EXPECT_FALSE(machine_state_manager::allowsStableMapWrite(state));
  EXPECT_TRUE(machine_state_manager::freezesPose(machine_state_manager::CUTTING_STATIC));

  turning.lidar_speed = 0.0;
  EXPECT_EQ(machine_state_manager::CMD_MOVE_NO_DISP,
            machine_state_manager::classifyMachineState(turning, thresholds));
}

TEST(MachineStateLogic, RelocalizingOverridesMotionAndFreezesMapping) {
  machine_state_manager::MachineStateThresholds thresholds;
  machine_state_manager::MachineSignals signals;
  signals.left_track_speed = 0.4;
  signals.right_track_speed = 0.4;
  signals.lidar_speed = 0.2;
  signals.relocalizing = true;

  const std::string state = machine_state_manager::classifyMachineState(signals, thresholds);

  EXPECT_EQ(machine_state_manager::RELOCALIZING, state);
  EXPECT_FALSE(machine_state_manager::allowsStableMapWrite(state));
  EXPECT_TRUE(machine_state_manager::freezesPose(state));
  EXPECT_FALSE(machine_state_manager::allowsStableMapWrite(machine_state_manager::CMD_MOVE_NO_DISP));
  EXPECT_TRUE(machine_state_manager::freezesPose(machine_state_manager::CMD_MOVE_NO_DISP));
}

TEST(MachineStateLogic, NonFiniteSignalsOrThresholdsFailClosedToConflict) {
  machine_state_manager::MachineStateThresholds thresholds;
  machine_state_manager::MachineSignals polluted;
  polluted.left_track_speed = std::numeric_limits<double>::quiet_NaN();
  polluted.right_track_speed = 0.0;
  polluted.lidar_speed = 0.0;

  EXPECT_EQ(machine_state_manager::CONFLICT,
            machine_state_manager::classifyMachineState(polluted, thresholds));

  machine_state_manager::MachineSignals nominal;
  thresholds.move_track_threshold = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(machine_state_manager::CONFLICT,
            machine_state_manager::classifyMachineState(nominal, thresholds));
  EXPECT_TRUE(machine_state_manager::freezesPose(machine_state_manager::CONFLICT));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
