#include <Arduino.h>
#include <Wire.h>
#include <MPU6500_WE.h>

#include "config.h"
#include "imu.h"

// MPU9250_WE / MPU6500_WE reject any WHO_AM_I value other than their own, but
// the MPU-9250 (0x71), MPU-6500 (0x70) and MPU-9255 (0x73) share the same
// accelerometer/gyroscope register map. Read the ID ourselves and pass it to
// the base initialiser so all three modules configure correctly.
class Mpu925x : public MPU6500_WE
{
public:
    explicit Mpu925x(uint8_t address) : MPU6500_WE(address) {}

    bool initAny()
    {
        const uint8_t id = whoAmI();
        if (id != 0x70 && id != 0x71 && id != 0x73)
        {
            return false;
        }
        return MPU6500_WE::init(id);
    }
};

static Mpu925x s_imuPrimary(IMU_I2C_ADDRESS);
static Mpu925x s_imuAlternate(IMU_I2C_ADDRESS_ALT);
static Mpu925x *s_imu = nullptr;

static bool s_ready = false;
static uint32_t s_lastPollMs = 0;

// Orientation used by presentation pointer mode.
static ImuAngles s_angles = {0.0f, 0.0f};
static uint32_t s_lastAngleMs = 0;
static bool s_anglesStarted = false;

// Shake detector state (hysteresis + cooldown, one key per shake burst).
static bool s_armed = true;
static bool s_belowRelease = false;
static uint32_t s_belowSinceMs = 0;
static uint32_t s_lastShakeMs = 0;

bool imuBegin()
{
    Wire.setSDA(IMU_I2C_SDA);
    Wire.setSCL(IMU_I2C_SCL);
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);

    s_imu = &s_imuPrimary;
    if (!s_imu->initAny())
    {
        s_imu = &s_imuAlternate;
        if (!s_imu->initAny())
        {
            Serial.printf("MPU-9250/6500/9255 not detected on I2C "
                          "(SDA=GP%d, SCL=GP%d, tried 0x%02X/0x%02X, "
                          "WHO_AM_I=0x%02X)\n",
                          IMU_I2C_SDA, IMU_I2C_SCL,
                          IMU_I2C_ADDRESS, IMU_I2C_ADDRESS_ALT,
                          s_imuPrimary.whoAmI());
            s_imu = nullptr;
            s_ready = false;
            return false;
        }
    }

    // +/-4 g and +/-500 deg/s give enough headroom for vigorous shaking while
    // still resolving normal mouse movement. DLPF 3 is ~41 Hz bandwidth.
    s_imu->setAccRange(MPU6500_ACC_RANGE_4G);
    s_imu->setGyrRange(MPU6500_GYRO_RANGE_500);
    s_imu->enableAccDLPF(true);
    s_imu->setAccDLPF(MPU6500_DLPF_3);
    s_imu->enableGyrDLPF();
    s_imu->setGyrDLPF(MPU6500_DLPF_3);

    s_ready = true;
    // Start disarmed and let the detector re-arm after the sensor has been
    // quiet for a moment; this avoids false shakes while the board is handled
    // during boot.
    s_armed = false;
    s_belowRelease = false;
    s_lastShakeMs = millis();
    s_lastPollMs = 0;
    s_angles = ImuAngles{0.0f, 0.0f};
    s_lastAngleMs = millis();
    s_anglesStarted = false;

    Serial.printf("MPU-925x initialised at I2C 0x%02X (WHO_AM_I=0x%02X), "
                  "shake detector armed\n",
                  s_imu == &s_imuPrimary ? IMU_I2C_ADDRESS : IMU_I2C_ADDRESS_ALT,
                  s_imu->whoAmI());
    return true;
}

bool imuPoll()
{
    if (!s_ready)
    {
        return false;
    }

    const uint32_t now = millis();
    if ((uint32_t)(now - s_lastPollMs) < IMU_POLL_INTERVAL_MS)
    {
        return false;
    }
    s_lastPollMs = now;

    const xyzFloat acc = s_imu->getGValues();
    const xyzFloat gyr = s_imu->getGyrValues();

    // Integrate the relevant gyro axes into the relative pointing angles.
    // A per-axis deadband suppresses bias drift while the mouse is held still.
    const uint32_t elapsedMs = now - s_lastAngleMs;
    s_lastAngleMs = now;
    if (s_anglesStarted)
    {
        const uint32_t dtMs = (elapsedMs > 100) ? 100 : elapsedMs;
        const float dt = (float)dtMs * 0.001f;

        float yawRate = 0.0f;
        if (fabsf(gyr.z) >= IMU_GYRO_DEADBAND_DPS)
        {
            yawRate = gyr.z * DEG_TO_RAD * IMU_YAW_GYRO_SIGN;
        }

        float pitchGyro = gyr.y;
#if IMU_PITCH_USES_GYRO_X
        pitchGyro = gyr.x;
#endif
        float pitchRate = 0.0f;
        if (fabsf(pitchGyro) >= IMU_GYRO_DEADBAND_DPS)
        {
            pitchRate = pitchGyro * DEG_TO_RAD * IMU_PITCH_GYRO_SIGN;
        }

        s_angles.yawRad += yawRate * dt;
        s_angles.pitchRad += pitchRate * dt;
    }
    else
    {
        s_anglesStarted = true;
    }

    // Orientation-independent shake signal: at rest the acceleration magnitude
    // is ~1 g regardless of how the module is mounted.
    const float accMag = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    const float accDeviation = fabsf(accMag - 1.0f);
    const float gyrMag = sqrtf(gyr.x * gyr.x + gyr.y * gyr.y + gyr.z * gyr.z);

    const bool aboveTrigger =
        (accDeviation >= IMU_SHAKE_TRIGGER_G) ||
        (gyrMag >= IMU_SHAKE_TRIGGER_DPS);
    const bool belowRelease =
        (accDeviation <= IMU_SHAKE_RELEASE_G) &&
        (gyrMag <= IMU_SHAKE_RELEASE_DPS);

    if (belowRelease)
    {
        if (!s_belowRelease)
        {
            s_belowRelease = true;
            s_belowSinceMs = now;
        }
        if (!s_armed &&
            (uint32_t)(now - s_belowSinceMs) >= IMU_SHAKE_SETTLE_MS &&
            (uint32_t)(now - s_lastShakeMs) >= IMU_SHAKE_COOLDOWN_MS)
        {
            s_armed = true;
        }
    }
    else
    {
        s_belowRelease = false;
    }

    if (s_armed && aboveTrigger)
    {
        s_armed = false;
        s_belowRelease = false;
        s_lastShakeMs = now;

#if IMU_DEBUG_SHAKE
        Serial.printf("Shake: |a|=%.2f g (dev %.2f g), |gyro|=%.0f dps\n",
                      accMag, accDeviation, gyrMag);
#endif
        return true;
    }

    return false;
}

bool imuIsReady()
{
    return s_ready;
}

ImuAngles imuGetAngles()
{
    return s_angles;
}
