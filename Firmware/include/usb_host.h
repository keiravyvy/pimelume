#pragma once

#include <stdint.h>

// A single mouse report received from the USB host. Motion is accumulated
// (within one button state) and may exceed a single HID report's range; the
// controller splits it into several BLE reports when forwarding.
struct MouseReport {
    int16_t dx;
    int16_t dy;
    int8_t wheel;
    int8_t pan;
    uint8_t buttons;
};

// Debug/diagnostic information about the mounted USB mouse.
struct UsbMouseInfo {
    bool connected;
    uint16_t vid;
    uint16_t pid;
    uint8_t reportId;
    bool hasX;
    bool hasY;
    bool hasWheel;
    bool hasPan;
    bool hasButtons;
    uint8_t buttonCount;
    uint8_t xBits;
    uint8_t yBits;
    uint16_t dropped;
};

// Called from setup1() - configures Pico-PIO-USB and starts the TinyUSB host
// stack on core 1 so that bit-banging does not disturb the BLE timing on core 0.
void usbHostBegin();

// Called from loop1() - services the TinyUSB host stack.
void usbHostTask();

// Called from loop() on core 0. Pops the oldest pending mouse report. Returns
// false when the queue is empty.
bool usbHostTakeMouseReport(MouseReport *out);

// True while a USB HID mouse is mounted.
bool usbHostMouseConnected();

// Copies information about the mounted mouse (VID/PID, parsed report layout).
bool usbHostGetMouseInfo(UsbMouseInfo *out);
