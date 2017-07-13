#include <gtest/gtest.h>
#include "rosflight.h"
#include "test_board.h"
#include "cmath"

#define EXPECT_OPPOSITE(x, y) EXPECT_LE(std::abs(x) - std::abs(y), 0.001)

using namespace rosflight_firmware;

TEST(imu_test, calibration_simulation)
{
    testBoard board;
    ROSflight rf(board);

    // Initialize firmware
    rf.init();

    float fake_accel[3] = {1, 1.2, 0.8};
    float dummy_gyro[3] = {0, 0, 0};

    for (int t = 0; t < 100; ++t) {
        // Feed in fake acceleration data
        board.set_imu(fake_accel, dummy_gyro, (uint64_t)(t*1e6));

        // Run the firmware
        rf.run();

    }
    // When does the calibration happen?

    float bias[3];

    // Get the bias values
    bias[0] = rf.params_.get_param_float(PARAM_ACC_X_BIAS);
    bias[1] = rf.params_.get_param_float(PARAM_ACC_Y_BIAS);
    bias[2] = rf.params_.get_param_float(PARAM_ACC_Z_BIAS);

    for (int i = 0; i < 3; i++)
    {
      EXPECT_OPPOSITE(fake_accel[i], bias[i]);
    }    
}
