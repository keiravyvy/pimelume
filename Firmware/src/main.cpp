// MayDarkness 2026 - Pico W wireless mouse firmware
//
// Behaviour
//   * Advertises a Bluetooth LE mouse.
//   * Runs a USB host port on GPIO16/17 that sniffs the original mouse PCB and
//     forwards its movement, wheel, pan and every button over BLE.
//   * Shaking the mouse enters presentation pointer mode:
//       LEFT click  -> set the pointing center
//       RIGHT click -> back to normal mouse mode
//     In that mode the IMU orientation drives the pointer like a laser pointer;
//     physical mouse movement is ignored while scrolling still works.
//   * The MPU-9250/6500/9255 on I2C0 provides both the shake gesture and the
//     pointing orientation.

#include <Arduino.h>
#include <MouseBLE.h>

#include "config.h"
#include "imu.h"
#include "mouse_controller.h"
#include "usb_host.h"

// Core 1 is dedicated to Pico-PIO-USB so its bit-banging does not jitter the
// CYW43/BLE timing on core 0. These hooks are called by the arduino-pico core.
void setup1()
{
    usbHostBegin();
}

void loop1()
{
    usbHostTask();
}

static void startBleHid()
{
    MouseBLE.begin(BLE_DEVICE_NAME, BLE_DEVICE_NAME);
    Serial.printf("BLE HID advertising as \"%s\"\n", BLE_DEVICE_NAME);
}

void setup()
{
    Serial.begin(115200);

    // Do not block forever when no USB serial host is attached.
    const uint32_t serialWaitStart = millis();
    while (!Serial && (uint32_t)(millis() - serialWaitStart) < 1500)
    {
        delay(10);
    }

    Serial.println();
    Serial.println("MayDarkness 2026 wireless mouse");
    Serial.printf("USB host: D+ = GPIO%d, D- = GPIO%d, VBUS enable = GPIO%d\n",
                  PIN_USB_HOST_DP, PIN_USB_HOST_DM, PIN_USB_VBUS_EN);

    startBleHid();
    imuBegin();
    mouseControllerBegin();
}

void loop()
{
    mouseControllerUpdate();
}
