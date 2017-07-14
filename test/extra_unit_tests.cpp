#include <gtest/gtest.h>
#include "rosflight.h"
#include "test_board.h"
#include "cmath"
#include <stdio.h>

using namespace rosflight_firmware;

// move time forward while feeding acceleration data to the board
void step_imu(ROSflight& rf, testBoard& board, float acc_data[3])
{
  float dummy_gyro[3] = {0, 0, 0};

  // Set the calibrating_acc_flag
  rf.sensors_.start_imu_calibration();

  // Feed in fake acceleration data, run enough times to trigger calibration
  for (int i = 0; i < 1001; i++)
  {
    board.set_imu(acc_data, dummy_gyro, (uint64_t)(i));
    rf.run();
  }
}

// Get the bias values out of ROSflight
void get_bias(ROSflight rf, testBoard board, float bias[3])
{
  bias[0] = rf.params_.get_param_float(PARAM_ACC_X_BIAS);
  bias[1] = rf.params_.get_param_float(PARAM_ACC_Y_BIAS);
  bias[2] = rf.params_.get_param_float(PARAM_ACC_Z_BIAS);
}

TEST(extra_unit_tests, imu_calibration)
{
  testBoard board;
  ROSflight rf(board);

  // Initialize firmware
  rf.init();

  float fake_accel[3] = {1, 0.2, -10};

  step_imu(rf, board, fake_accel);

  float bias[3];

  get_bias(rf, board, bias);

  for (int i = 0; i < 3; i++)
  {
    // calibration should not have occured, so the bias values should be zero
    EXPECT_NE(bias[i], 0);
  }
}
