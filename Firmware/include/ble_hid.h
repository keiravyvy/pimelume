#pragma once

#include <stdint.h>

// True while a BLE host has an HID connection open.
bool bleHidConnected();

// Sends a complete relative mouse report over BLE. `buttons` is the HID button
// bitmask; dx/dy are clamped by the caller to -127..127 (HID relative mouse
// reports only carry signed bytes). Returns false when no host is connected.
bool bleHidSendMouse(uint8_t buttons, int8_t dx, int8_t dy,
                     int8_t wheel = 0, int8_t pan = 0);
