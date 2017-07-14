#include <gtest/gtest.h>
#include "rosflight.h"
#include "test_board.h"
#include "cmath"
#include <stdio.h>

#define EXPECT_OPPOSITE(x, y) EXPECT_LE(std::abs(x) - std::abs(y), 0.001)

using namespace rosflight_firmware;

TEST(imu_test, calibration_simulation)
{
    /*
     *  Things I don't understand:
     *  How is IMU calibration triggered?
     *  Does it happen automatically?
     *  accel is calibrated by Sensors::update_imu when Sensors::calibrating_acc_flag
     *  is asserted
     *
     *  How do I tell when calibration has happened?
     *
     *
     *
    */

    testBoard board;
    ROSflight rf(board);

    // Initialize firmware
    rf.init();

    // Set the calibrating_acc_flag
    rf.sensors_.start_imu_calibration();

    float fake_accel[3] = {1, 0.2, -10};
    float dummy_gyro[3] = {0, 0, 0};

    // Feed in fake acceleration data, set time to 0
    board.set_imu(fake_accel, dummy_gyro, (uint64_t)(0));

    uint64_t start_time_us = board.clock_micros();
    uint64_t run_time_us = 1e5;

    for (int i = 0; i < 1500; i++)
    {
      board.set_imu(fake_accel, dummy_gyro, (uint64_t)(0));
      rf.run();
    }

    while(board.clock_micros() < start_time_us + run_time_us)
    {
      board.set_time(board.clock_micros() + 1000);
      rf.run();
    }

    EXPECT_EQ(rf.state_manager_.state().armed, true);

    float bias[3];

    // Get the bias values
    bias[0] = rf.params_.get_param_float(PARAM_ACC_X_BIAS);
    bias[1] = rf.params_.get_param_float(PARAM_ACC_Y_BIAS);
    bias[2] = rf.params_.get_param_float(PARAM_ACC_Z_BIAS);

    for (int i = 0; i < 3; i++)
    {
      EXPECT_OPPOSITE(fake_accel[i], bias[i]);
      std::cout << "Axis " << i << " bias: " << bias[i] << std::endl;
    }    
}
