#pragma once

#include <stdint.h>

struct ImuAngles {
    // Relative orientation accumulated from the gyroscope, in radians.
    // yaw is rotation about the vertical axis, pitch is nose up/down.
    float yawRad;
    float pitchRad;
};

// Starts I2C0 and initialises the MPU-9250 / MPU-6500 / MPU-9255. Returns false
// if the IMU does not answer (the rest of the firmware keeps running, but shake
// detection and presentation pointing are disabled).
bool imuBegin();

// Samples the IMU and returns true exactly once per detected shake event.
// Call it regularly from loop().
bool imuPoll();

// Latest integrated pointing angles. Only meaningful after imuBegin().
ImuAngles imuGetAngles();

bool imuIsReady();
