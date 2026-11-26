#include <Arduino.h>
#include <HID_Mouse.h>
#include <math.h>

#include "ble_hid.h"
#include "config.h"
#include "imu.h"
#include "mouse_controller.h"
#include "usb_host.h"

namespace
{

enum class Mode
{
    Mouse,
    Presentation
};

constexpr uint8_t kLeftButton = MOUSE_LEFT;   // 0x01
constexpr uint8_t kRightButton = MOUSE_RIGHT; // 0x02
constexpr uint8_t kModeButtons = kLeftButton | kRightButton;

Mode s_mode = Mode::Mouse;

bool s_centerSet = false;
float s_centerYaw = 0.0f;
float s_centerPitch = 0.0f;
float s_virtualX = 0.0f;
float s_virtualY = 0.0f;

// Physical state from the USB mouse vs. what the BLE host currently sees.
uint8_t s_hostButtons = 0;
uint8_t s_lastPhysicalButtons = 0;
uint8_t s_suppressButtons = 0;

bool s_lastUsbConnected = false;

int8_t clamp8(int32_t value)
{
    if (value > INT8_MAX)
    {
        return INT8_MAX;
    }
    if (value < INT8_MIN)
    {
        return INT8_MIN;
    }
    return (int8_t)value;
}

void sendButtonSync()
{
    if (bleHidConnected())
    {
        bleHidSendMouse(s_hostButtons, 0, 0, 0, 0);
    }
}

void centerPointer()
{
    const ImuAngles angles = imuGetAngles();
    s_centerYaw = angles.yawRad;
    s_centerPitch = angles.pitchRad;
    s_virtualX = 0.0f;
    s_virtualY = 0.0f;
    s_centerSet = true;
    Serial.println("Presentation center set");
}

void enterPresentation()
{
    if (s_mode == Mode::Presentation)
    {
        return;
    }

    s_mode = Mode::Presentation;
    s_centerSet = false;

    // Release the mode buttons on the host but keep any other held buttons
    // (middle/side) consistent. Suppress them until they are released.
    s_suppressButtons |= (uint8_t)(s_lastPhysicalButtons & kModeButtons);
    s_hostButtons = s_lastPhysicalButtons & (uint8_t)~kModeButtons;
    sendButtonSync();

    Serial.println("Presentation pointer mode: LEFT click = set center, "
                   "RIGHT click = mouse mode");
}

void exitPresentation()
{
    if (s_mode == Mode::Mouse)
    {
        return;
    }

    s_mode = Mode::Mouse;
    s_centerSet = false;

    // The report handler decides what to send next; it will keep the mode
    // buttons suppressed until they are released.
    Serial.println("Mouse mode");
}

// Splits motion larger than a HID relative report (±127) across several BLE
// reports so fast mouse movements are not lost.
void sendMotion(int32_t dx, int32_t dy, int8_t wheel, int8_t pan)
{
    if (!bleHidConnected())
    {
        return;
    }

    bool first = true;
    while (first || dx != 0 || dy != 0)
    {
        first = false;

        const int8_t stepX = clamp8(dx);
        const int8_t stepY = clamp8(dy);
        const int8_t stepWheel = wheel;
        const int8_t stepPan = pan;
        wheel = 0;
        pan = 0;

        if (!bleHidSendMouse(s_hostButtons, stepX, stepY, stepWheel, stepPan))
        {
            break;
        }

        dx -= stepX;
        dy -= stepY;
    }
}

void handleReport(const MouseReport &report)
{
    const uint8_t rawButtons = report.buttons;
    const uint8_t changed = rawButtons ^ s_lastPhysicalButtons;
    const bool wasPresentation = (s_mode == Mode::Presentation);

    if (wasPresentation)
    {
        if ((changed & kLeftButton) && (rawButtons & kLeftButton))
        {
            centerPointer();
        }
        if ((changed & kRightButton) && (rawButtons & kRightButton))
        {
            exitPresentation();
        }
    }

    uint8_t hostButtons;
    if (wasPresentation)
    {
        // Left/right are consumed by presentation mode, everything else passes
        // through.
        hostButtons = rawButtons & (uint8_t)~kModeButtons;
    }
    else
    {
        hostButtons = rawButtons & (uint8_t)~s_suppressButtons;
    }

    // If right-click just left presentation mode, keep it hidden from the host
    // until it is released, so exiting does not open a context menu.
    if (wasPresentation && s_mode == Mode::Mouse)
    {
        s_suppressButtons |= (uint8_t)(rawButtons & kModeButtons);
    }

    const bool buttonsChanged = (hostButtons != s_hostButtons);
    s_hostButtons = hostButtons;

    const bool motion = (report.dx != 0) || (report.dy != 0) ||
                        (report.wheel != 0) || (report.pan != 0);

    if (!wasPresentation)
    {
        if (buttonsChanged || motion)
        {
            sendMotion(report.dx, report.dy, report.wheel, report.pan);
        }
    }
    else if (buttonsChanged || report.wheel != 0 || report.pan != 0)
    {
        // Physical movement is ignored while pointing, but scrolling and extra
        // buttons stay useful (for example wheel to change slides).
        sendMotion(0, 0, report.wheel, report.pan);
    }

    s_suppressButtons &= rawButtons;
    s_lastPhysicalButtons = rawButtons;
}

void applyPointer()
{
    if (s_mode != Mode::Presentation || !s_centerSet || !imuIsReady())
    {
        return;
    }

    const ImuAngles angles = imuGetAngles();
    float yawDelta = angles.yawRad - s_centerYaw;
    float pitchDelta = angles.pitchRad - s_centerPitch;

    if (fabsf(yawDelta) < IMU_POINTER_DEADZONE_RAD)
    {
        yawDelta = 0.0f;
    }
    if (fabsf(pitchDelta) < IMU_POINTER_DEADZONE_RAD)
    {
        pitchDelta = 0.0f;
    }

    float targetX =
        yawDelta * IMU_POINTER_PIXELS_PER_RADIAN * IMU_POINTER_X_SIGN;
    float targetY =
        pitchDelta * IMU_POINTER_PIXELS_PER_RADIAN * IMU_POINTER_Y_SIGN;

    targetX = constrain(targetX, -IMU_POINTER_MAX_OFFSET_PX,
                        IMU_POINTER_MAX_OFFSET_PX);
    targetY = constrain(targetY, -IMU_POINTER_MAX_OFFSET_PX,
                        IMU_POINTER_MAX_OFFSET_PX);

    const int32_t dx = (int32_t)lroundf(targetX - s_virtualX);
    const int32_t dy = (int32_t)lroundf(targetY - s_virtualY);
    if (dx == 0 && dy == 0)
    {
        return;
    }

    if (!bleHidConnected())
    {
        // Do not accumulate a huge jump for when the host reconnects.
        s_virtualX = targetX;
        s_virtualY = targetY;
        return;
    }

    const int8_t stepX = clamp8(dx);
    const int8_t stepY = clamp8(dy);
    if (bleHidSendMouse(s_hostButtons, stepX, stepY, 0, 0))
    {
        s_virtualX += stepX;
        s_virtualY += stepY;
    }
}

void logMouseInfo()
{
    UsbMouseInfo info;
    if (!usbHostGetMouseInfo(&info))
    {
        return;
    }

    Serial.printf("USB mouse %04X:%04X, report ID %u, X%u-bit Y%u-bit, "
                  "wheel=%s pan=%s buttons=%u\n",
                  info.vid, info.pid, (unsigned)info.reportId,
                  (unsigned)info.xBits, (unsigned)info.yBits,
                  info.hasWheel ? "yes" : "no", info.hasPan ? "yes" : "no",
                  (unsigned)info.buttonCount);
    if (!info.hasX)
    {
        Serial.println("  (descriptor parse failed; using boot mouse layout)");
    }
}

} // namespace

void mouseControllerBegin()
{
    s_lastUsbConnected = usbHostMouseConnected();
    if (s_lastUsbConnected)
    {
        logMouseInfo();
    }

    Serial.println("Mouse mode (shake the mouse to enter presentation mode)");
    if (!imuIsReady())
    {
        Serial.println("IMU unavailable: presentation mode is disabled");
    }
}

void mouseControllerUpdate()
{
    const bool usbConnected = usbHostMouseConnected();

    // Always poll the IMU: it updates the pointing angles used in presentation
    // mode, and returns true once per shake gesture.
    const bool shake = imuPoll();

    // The shake detector switches normal mouse mode -> presentation mode.
    // Require the USB mouse so there is always a way to click/exit afterwards.
    if (shake && usbConnected && s_mode == Mode::Mouse)
    {
        enterPresentation();
    }

    if (usbConnected != s_lastUsbConnected)
    {
        s_lastUsbConnected = usbConnected;
        if (usbConnected)
        {
            logMouseInfo();
        }
        else
        {
            // Without the mouse PCB there is no way to click, so fall back.
            if (s_mode == Mode::Presentation)
            {
                exitPresentation();
            }
            s_lastPhysicalButtons = 0;
            s_suppressButtons = 0;
            s_hostButtons = 0;
            s_centerSet = false;
            // Release anything the BLE host still thinks is held.
            sendButtonSync();
        }
    }

    MouseReport report;
    while (usbHostTakeMouseReport(&report))
    {
        handleReport(report);
    }

    applyPointer();
}
