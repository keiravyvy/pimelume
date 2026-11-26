#include <Arduino.h>
#include <HID_Bluetooth.h>
#include <PicoBluetoothBLEHID.h>
#include <class/hid/hid.h>
#include <string.h>

#include "ble_hid.h"

bool bleHidConnected()
{
    return PicoBluetoothBLEHID.connected();
}

bool bleHidSendMouse(uint8_t buttons, int8_t dx, int8_t dy, int8_t wheel,
                     int8_t pan)
{
    // Same report layout as TinyUSB's standard relative mouse descriptor:
    // [report ID][buttons][x][y][wheel][pan].
    uint8_t report[1 + sizeof(hid_mouse_report_t)];
    hid_mouse_report_t data;
    // MouseBLE uses TinyUSB's standard 5-button descriptor; mask off any extra
    // buttons the original mouse may report so the padding bits stay zero.
    data.buttons = buttons & 0x1F;
    data.x = dx;
    data.y = dy;
    data.wheel = wheel;
    data.pan = pan;

    report[0] = __BLEGetMouseReportID();
    memcpy(&report[1], &data, sizeof(data));

    return PicoBluetoothBLEHID.send(report, sizeof(report));
}
