#pragma once

#include <stdint.h>

#include "usb_host.h"

// Describes where the useful fields live inside one mouse input report.
// Offsets are in bits and are relative to the report payload (i.e. excluding
// the leading report-ID byte when the descriptor uses report IDs).
struct HidMouseLayout {
    bool valid;
    bool isMouseCollection;
    uint8_t reportId;

    bool hasX;
    bool hasY;
    bool hasWheel;
    bool hasPan;
    bool hasButtons;

    uint16_t xBit;
    uint16_t yBit;
    uint16_t wheelBit;
    uint16_t panBit;
    uint16_t buttonsBit;

    uint8_t xBits;
    uint8_t yBits;
    uint8_t wheelBits;
    uint8_t panBits;
    uint8_t buttonsBits;
    uint8_t buttonCount;

    bool xSigned;
    bool ySigned;
    bool wheelSigned;
    bool panSigned;
};

// Parses a HID report descriptor and selects the top-level collection that
// looks like a mouse (has an X and a Y field). Supports report IDs, 8/16-bit
// axes, buttons, wheel and AC Pan. Returns false if no pointer report is found.
bool hidParseMouseLayout(const uint8_t *desc, uint16_t len, HidMouseLayout *out);

// Parses one input report according to `layout`. When `bootProtocol` is true
// (or the layout is not valid) the standard 3/4-byte boot mouse layout is used.
bool hidParseMouseReport(const HidMouseLayout &layout, const uint8_t *report,
                         uint16_t len, MouseReport *out,
                         bool bootProtocol = false);
