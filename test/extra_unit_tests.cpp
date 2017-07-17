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
  uint64_t start = board.clock_micros();
  for (int i = start; i < start + 1001; i++)
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

TEST(extra_unit_tests, time_going_backwards)
{
  testBoard board;
  ROSflight rf(board);

  // data to pass into rosflight
  float accel[3] = {1, 1, -9.8};
  float gyro[3] = {0, 0, 0};
  float acc_cal[3] = {0, 0, -9.8};

  // Initialize firmware
  rf.init();

  // calibrate the imu
  step_imu(rf, board, acc_cal);

  // call testBoard::set_imu so that the new_imu_ flag will get set
  board.set_imu(accel, gyro, (uint64_t)(1000));
  rf.run();

  // call set_imu again with a time before the first call
  board.set_imu(accel, gyro, (uint64_t)(500));
  rf.run();

  // when the error occures, the estimator::run() function sets the time_going_backwards error in the state machine
  // the error condition is picked up by state_manager::run() which then calls process_errors()
  // process_errors() calls set_event(), and this is where the state transition happens in the FSM
  // make sure that the error was caught
  EXPECT_EQ(rf.state_manager_.state().error, true);

  // make time go forwards
  board.set_imu(accel, gyro, (uint64_t)(1500));
  rf.run();

  // make sure the error got cleared
  EXPECT_EQ(rf.state_manager_.state().error, false);
}

TEST(extra_unit_tests, imu_not_responding)
{
  testBoard board;
  ROSflight rf(board);

  rf.init();

  float acc[3] = {0, 0, -9.8};
  float gyro[3] = {0, 0, 0};

  board.set_time((uint64_t)(5));

  // calibrate the imu
  step_imu(rf, board, acc);

  // there should not be any errors at this point
  EXPECT_EQ(rf.state_manager_.state().error, false);

  // go more than 1000ms without imu update
  board.set_time(board.clock_micros() + 1.5e6);
  rf.run();

  // rf should be in the error state
  EXPECT_EQ(rf.state_manager_.state().error, true);

  // update the imu and make sure the error gets cleared
  board.set_imu(acc, gyro, board.clock_micros() + 100);
  rf.run();
  EXPECT_EQ(rf.state_manager_.state().error, false);
}

TEST(extra_unit_tests, anti_windup)
{
  /*  TODO: 
  *   find motor outputs in codebase
  *   find where the saturation gets set
  *   figure out how to send commands
  *   figure out how to make rf go somewhere autonomously
  */

  // send a command to go to something really far away

  // make sure that rf does not try to send the motor output above saturation

  // reverse direction

  // make sure that rf turns around in a reasonable amount of time
}