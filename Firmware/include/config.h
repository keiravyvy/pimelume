#pragma once

// ---------------------------------------------------------------------------
// USB host (Pico-PIO-USB)
// ---------------------------------------------------------------------------
// The Pico W bit-bangs a full-speed USB host port on two consecutive GPIOs.
// D- is always D+ + 1. These defaults match the wiring documented in README.md.
#ifndef PIN_USB_HOST_DP
#define PIN_USB_HOST_DP 16
#endif

#ifndef PIN_USB_HOST_DM
#define PIN_USB_HOST_DM (PIN_USB_HOST_DP + 1)
#endif

// Optional GPIO that drives a VBUS load switch / MOSFET for the downstream
// USB port. Set to -1 if VBUS is wired permanently to 5 V.
#ifndef PIN_USB_VBUS_EN
#define PIN_USB_VBUS_EN 18
#endif

#ifndef PIN_USB_VBUS_EN_ACTIVE_HIGH
#define PIN_USB_VBUS_EN_ACTIVE_HIGH 1
#endif

// ---------------------------------------------------------------------------
// Bluetooth LE HID
// ---------------------------------------------------------------------------
#define BLE_DEVICE_NAME "MayDarkness Mouse"

// ---------------------------------------------------------------------------
// MPU-9250 / MPU-6500 / MPU-9255 IMU (I2C0 on the Pico W)
// ---------------------------------------------------------------------------
#ifndef IMU_I2C_SDA
#define IMU_I2C_SDA 4
#endif

#ifndef IMU_I2C_SCL
#define IMU_I2C_SCL 5
#endif

#define IMU_I2C_CLOCK_HZ 400000

// AD0 low = 0x68 (most GY-9250 boards), AD0 high = 0x69. Both are tried.
#define IMU_I2C_ADDRESS 0x68
#define IMU_I2C_ADDRESS_ALT 0x69

// Shake detector tuning. A deliberate shake easily exceeds 0.5 g of dynamic
// acceleration and 250 deg/s of rotation; normal mouse movement stays below.
// Raise the trigger values if normal movement switches modes by accident.
#define IMU_POLL_INTERVAL_MS 10

#define IMU_SHAKE_TRIGGER_G 0.5f
#define IMU_SHAKE_RELEASE_G 0.3f
#define IMU_SHAKE_TRIGGER_DPS 250.0f
#define IMU_SHAKE_RELEASE_DPS 120.0f

// After a shake, wait until the sensor has been quiet for this long and the
// cooldown has elapsed before the detector can fire again.
#define IMU_SHAKE_SETTLE_MS 120
#define IMU_SHAKE_COOLDOWN_MS 350

// Print the measured acceleration/rotation for each detected shake.
#define IMU_DEBUG_SHAKE 1

// ---------------------------------------------------------------------------
// Presentation pointer mode
// ---------------------------------------------------------------------------
// Shake the mouse in normal mode to enter presentation pointer mode.
//   LEFT click  -> set / re-set the pointing center
//   RIGHT click -> back to normal mouse mode
//
// Orientation is integrated from the gyroscope. This assumes the IMU's +Z axis
// points up and +X points out of the front of the mouse when it is flat on the
// desk (so yaw is rotation about Z and pitch is rotation about Y). If your
// module is mounted rotated 90 degrees, enable IMU_PITCH_USES_GYRO_X.
#define IMU_PITCH_USES_GYRO_X 0

// Flip these if pointing left/right or up/down is inverted. The defaults assume
// +Z is up, +X points forward and +Y points left when the mouse is flat, so:
// yaw positive = turning right, pitch positive = nose up, and HID dy is
// inverted because positive mouse Y points down.
#define IMU_YAW_GYRO_SIGN -1.0f
#define IMU_PITCH_GYRO_SIGN -1.0f
#define IMU_POINTER_X_SIGN 1.0f
#define IMU_POINTER_Y_SIGN -1.0f

// Ignore gyro readings below this rate so the pointer does not slowly drift
// while the mouse is held still.
#define IMU_GYRO_DEADBAND_DPS 1.5f

// Cursor travel per radian of rotation away from the center. 900 px/rad is
// roughly 16 px per degree; adjust to taste.
#define IMU_POINTER_PIXELS_PER_RADIAN 900.0f

// Angular dead zone around the center (radians) to suppress one-pixel jitter.
#define IMU_POINTER_DEADZONE_RAD 0.0035f

// Clamp the virtual cursor offset so a large/garbage rotation cannot run away.
#define IMU_POINTER_MAX_OFFSET_PX 4000.0f
