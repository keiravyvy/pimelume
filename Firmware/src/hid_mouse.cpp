#include <string.h>

#include "hid_mouse.h"

namespace
{

// HID item types.
constexpr uint8_t kTypeMain = 0;
constexpr uint8_t kTypeGlobal = 1;
constexpr uint8_t kTypeLocal = 2;

// Main-item tags.
constexpr uint8_t kMainInput = 0x8;
constexpr uint8_t kMainCollection = 0xA;
constexpr uint8_t kMainEndCollection = 0xC;

// Global-item tags.
constexpr uint8_t kGlobalUsagePage = 0x0;
constexpr uint8_t kGlobalLogicalMin = 0x1;
constexpr uint8_t kGlobalReportSize = 0x7;
constexpr uint8_t kGlobalReportId = 0x8;
constexpr uint8_t kGlobalReportCount = 0x9;

// Local-item tags.
constexpr uint8_t kLocalUsage = 0x0;
constexpr uint8_t kLocalUsageMin = 0x1;
constexpr uint8_t kLocalUsageMax = 0x2;

constexpr uint16_t kUsagePageGenericDesktop = 0x01;
constexpr uint16_t kUsagePageButton = 0x09;
constexpr uint16_t kUsagePageConsumer = 0x0C;

constexpr uint16_t kUsageX = 0x30;
constexpr uint16_t kUsageY = 0x31;
constexpr uint16_t kUsageWheel = 0x38;
constexpr uint16_t kUsageAcPan = 0x0238;

constexpr uint16_t kUsageMouse = 0x02;
constexpr uint16_t kUsagePointer = 0x01;

constexpr uint8_t kMaxUsages = 32;
constexpr uint8_t kMaxCandidates = 8;

struct Candidate {
    HidMouseLayout layout;
    uint32_t bitOffset;
    bool appIsMouse;
};

struct Parser {
    uint16_t usagePage;
    uint8_t reportId;
    uint32_t reportSize;
    uint32_t reportCount;
    int32_t logicalMin;
    uint16_t usages[kMaxUsages];
    uint8_t usageCount;
    uint16_t usageMin;
    uint16_t usageMax;
    bool hasUsageMin;
    bool hasUsageMax;
    uint8_t depth;
    uint16_t appUsagePage;
    uint16_t appUsage;

    void clearLocal()
    {
        usageCount = 0;
        hasUsageMin = false;
        hasUsageMax = false;
    }
};

int32_t signExtend(uint32_t value, uint8_t bytes)
{
    switch (bytes)
    {
    case 1:
        return (int8_t)value;
    case 2:
        return (int16_t)value;
    case 4:
        return (int32_t)value;
    default:
        return (int32_t)value;
    }
}

bool resolveUsage(const Parser &p, uint32_t field, uint16_t *usage)
{
    if (p.usageCount > 0)
    {
        if (field < p.usageCount)
        {
            *usage = p.usages[field];
            return true;
        }
        return false;
    }

    if (p.hasUsageMin && p.hasUsageMax)
    {
        const uint32_t value = p.usageMin + field;
        if (value <= p.usageMax)
        {
            *usage = (uint16_t)value;
            return true;
        }
    }

    return false;
}

Candidate *findCandidate(Candidate *candidates, uint8_t *count, uint8_t reportId,
                         bool appIsMouse)
{
    for (uint8_t i = 0; i < *count; i++)
    {
        if (candidates[i].layout.reportId == reportId)
        {
            candidates[i].appIsMouse = candidates[i].appIsMouse || appIsMouse;
            return &candidates[i];
        }
    }

    if (*count >= kMaxCandidates)
    {
        return nullptr;
    }

    Candidate *candidate = &candidates[*count];
    memset(candidate, 0, sizeof(*candidate));
    candidate->layout.reportId = reportId;
    candidate->appIsMouse = appIsMouse;
    (*count)++;
    return candidate;
}

void setAxis(HidMouseLayout &layout, bool signedValue, uint16_t bit,
             uint8_t bits, bool isX, bool isY, bool isWheel, bool isPan)
{
    if (isX && !layout.hasX)
    {
        layout.hasX = true;
        layout.xBit = bit;
        layout.xBits = bits;
        layout.xSigned = signedValue;
    }
    else if (isY && !layout.hasY)
    {
        layout.hasY = true;
        layout.yBit = bit;
        layout.yBits = bits;
        layout.ySigned = signedValue;
    }
    else if (isWheel && !layout.hasWheel)
    {
        layout.hasWheel = true;
        layout.wheelBit = bit;
        layout.wheelBits = bits;
        layout.wheelSigned = signedValue;
    }
    else if (isPan && !layout.hasPan)
    {
        layout.hasPan = true;
        layout.panBit = bit;
        layout.panBits = bits;
        layout.panSigned = signedValue;
    }
}

bool extractBits(const uint8_t *data, uint16_t bitLength, uint16_t bitOffset,
                 uint8_t bitSize, bool signedValue, int32_t *out)
{
    if (!data || !out || bitSize == 0 || bitSize > 32)
    {
        return false;
    }

    if ((uint32_t)bitOffset + bitSize > bitLength)
    {
        return false;
    }

    const uint16_t byteOffset = bitOffset / 8;
    const uint8_t shift = bitOffset % 8;
    const uint8_t byteCount = (uint8_t)((shift + bitSize + 7) / 8);
    uint32_t raw = 0;

    for (uint8_t i = 0; i < byteCount; i++)
    {
        raw |= (uint32_t)data[byteOffset + i] << (8 * i);
    }

    raw >>= shift;
    if (bitSize < 32)
    {
        raw &= (1UL << bitSize) - 1UL;
        if (signedValue && (raw & (1UL << (bitSize - 1))))
        {
            raw |= ~((1UL << bitSize) - 1UL);
        }
    }

    *out = (int32_t)raw;
    return true;
}

int16_t clamp16(int32_t value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (value < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)value;
}

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

bool parseBootReport(const uint8_t *report, uint16_t len, MouseReport *out)
{
    if (len < 3)
    {
        return false;
    }

    out->buttons = report[0];
    out->dx = (int8_t)report[1];
    out->dy = (int8_t)report[2];
    out->wheel = (len >= 4) ? (int8_t)report[3] : 0;
    out->pan = (len >= 5) ? (int8_t)report[4] : 0;
    return true;
}

} // namespace

bool hidParseMouseLayout(const uint8_t *desc, uint16_t len, HidMouseLayout *out)
{
    if (!desc || !out || len < 4)
    {
        return false;
    }

    Parser p;
    memset(&p, 0, sizeof(p));

    Candidate candidates[kMaxCandidates];
    uint8_t candidateCount = 0;

    uint16_t pos = 0;
    while (pos < len)
    {
        const uint8_t prefix = desc[pos++];
        uint8_t size = prefix & 0x03;
        if (size == 3)
        {
            size = 4;
        }
        const uint8_t type = (prefix >> 2) & 0x03;
        const uint8_t tag = (prefix >> 4) & 0x0F;

        if ((uint32_t)pos + size > len)
        {
            break;
        }

        uint32_t data = 0;
        for (uint8_t i = 0; i < size; i++)
        {
            data |= (uint32_t)desc[pos + i] << (8 * i);
        }
        pos += size;

        if (type == kTypeGlobal)
        {
            switch (tag)
            {
            case kGlobalUsagePage:
                p.usagePage = (uint16_t)data;
                break;
            case kGlobalLogicalMin:
                p.logicalMin = signExtend(data, size);
                break;
            case kGlobalReportSize:
                p.reportSize = data;
                break;
            case kGlobalReportId:
                p.reportId = (uint8_t)data;
                break;
            case kGlobalReportCount:
                p.reportCount = data;
                break;
            default:
                break;
            }
            continue;
        }

        if (type == kTypeLocal)
        {
            switch (tag)
            {
            case kLocalUsage:
                if (p.usageCount < kMaxUsages)
                {
                    p.usages[p.usageCount++] = (uint16_t)data;
                }
                break;
            case kLocalUsageMin:
                p.usageMin = (uint16_t)data;
                p.hasUsageMin = true;
                break;
            case kLocalUsageMax:
                p.usageMax = (uint16_t)data;
                p.hasUsageMax = true;
                break;
            default:
                break;
            }
            continue;
        }

        if (type != kTypeMain)
        {
            continue;
        }

        if (tag == kMainCollection)
        {
            if (p.depth == 0)
            {
                p.appUsagePage = p.usagePage;
                if (p.usageCount > 0)
                {
                    p.appUsage = p.usages[0];
                }
                else if (p.hasUsageMin)
                {
                    p.appUsage = p.usageMin;
                }
                else
                {
                    p.appUsage = 0;
                }
            }
            p.depth++;
            p.clearLocal();
            continue;
        }

        if (tag == kMainEndCollection)
        {
            if (p.depth > 0)
            {
                p.depth--;
            }
            if (p.depth == 0)
            {
                p.appUsagePage = 0;
                p.appUsage = 0;
            }
            p.clearLocal();
            continue;
        }

        if (tag == kMainInput)
        {
            const bool constant = (data & 0x01) != 0;
            const bool variable = (data & 0x02) != 0;
            const bool relative = (data & 0x04) != 0;
            const bool appIsMouse =
                (p.appUsagePage == kUsagePageGenericDesktop) &&
                (p.appUsage == kUsageMouse || p.appUsage == kUsagePointer);

            Candidate *candidate =
                findCandidate(candidates, &candidateCount, p.reportId, appIsMouse);

            if (candidate && variable && !constant && p.reportSize > 0)
            {
                HidMouseLayout &layout = candidate->layout;

                if (p.usagePage == kUsagePageButton && p.reportSize == 1 &&
                    !layout.hasButtons)
                {
                    layout.hasButtons = true;
                    layout.buttonsBit = (uint16_t)candidate->bitOffset;
                    layout.buttonsBits = 1;
                    layout.buttonCount =
                        (p.reportCount > 8) ? 8 : (uint8_t)p.reportCount;
                }

                for (uint32_t field = 0; field < p.reportCount; field++)
                {
                    uint16_t usage = 0;
                    if (!resolveUsage(p, field, &usage))
                    {
                        continue;
                    }

                    const uint32_t bit =
                        candidate->bitOffset + field * p.reportSize;
                    const uint8_t bits = (p.reportSize > 32) ? 32 : (uint8_t)p.reportSize;
                    const bool signedValue = relative || (p.logicalMin < 0);

                    if (p.usagePage == kUsagePageGenericDesktop)
                    {
                        setAxis(layout, signedValue, (uint16_t)bit, bits,
                                usage == kUsageX, usage == kUsageY,
                                usage == kUsageWheel, false);
                    }
                    else if (p.usagePage == kUsagePageConsumer &&
                             usage == kUsageAcPan)
                    {
                        setAxis(layout, signedValue, (uint16_t)bit, bits, false,
                                false, false, true);
                    }
                }
            }

            if (candidate)
            {
                candidate->bitOffset += p.reportSize * p.reportCount;
            }
            p.clearLocal();
            continue;
        }

        // Output, Feature, etc. terminate the local item state as well.
        p.clearLocal();
    }

    const Candidate *best = nullptr;
    int bestScore = -1;
    for (uint8_t i = 0; i < candidateCount; i++)
    {
        const Candidate &candidate = candidates[i];
        if (!candidate.layout.hasX || !candidate.layout.hasY)
        {
            continue;
        }

        int score = candidate.appIsMouse ? 100 : 0;
        score += candidate.layout.hasButtons ? 4 : 0;
        score += candidate.layout.hasWheel ? 2 : 0;
        score += candidate.layout.hasPan ? 1 : 0;

        if (score > bestScore)
        {
            best = &candidate;
            bestScore = score;
        }
    }

    if (!best)
    {
        return false;
    }

    *out = best->layout;
    out->valid = true;
    out->isMouseCollection = best->appIsMouse;
    return true;
}

bool hidParseMouseReport(const HidMouseLayout &layout, const uint8_t *report,
                         uint16_t len, MouseReport *out, bool bootProtocol)
{
    if (!report || !out || len < 3)
    {
        return false;
    }

    memset(out, 0, sizeof(*out));

    if (bootProtocol || !layout.valid)
    {
        return parseBootReport(report, len, out);
    }

    const uint8_t *payload = report;
    uint16_t payloadLen = len;

    if (layout.reportId != 0)
    {
        if (report[0] != layout.reportId)
        {
            // Another report ID on the same interface (e.g. consumer control)
            // or a report we do not understand: do not forward it as a mouse.
            return false;
        }
        payload = report + 1;
        payloadLen = (uint16_t)(len - 1);
    }

    const uint16_t payloadBits = (uint16_t)(payloadLen * 8);
    int32_t x = 0;
    int32_t y = 0;

    if (!extractBits(payload, payloadBits, layout.xBit, layout.xBits,
                     layout.xSigned, &x) ||
        !extractBits(payload, payloadBits, layout.yBit, layout.yBits,
                     layout.ySigned, &y))
    {
        return false;
    }

    out->dx = clamp16(x);
    out->dy = clamp16(y);

    if (layout.hasButtons && layout.buttonCount > 0)
    {
        const uint8_t buttonBits =
            (uint8_t)(layout.buttonsBits * layout.buttonCount);
        int32_t rawButtons = 0;
        if (extractBits(payload, payloadBits, layout.buttonsBit, buttonBits,
                        false, &rawButtons))
        {
            out->buttons = (uint8_t)rawButtons;
        }
    }

    if (layout.hasWheel)
    {
        int32_t wheel = 0;
        if (extractBits(payload, payloadBits, layout.wheelBit, layout.wheelBits,
                        layout.wheelSigned, &wheel))
        {
            out->wheel = clamp8(wheel);
        }
    }

    if (layout.hasPan)
    {
        int32_t pan = 0;
        if (extractBits(payload, payloadBits, layout.panBit, layout.panBits,
                        layout.panSigned, &pan))
        {
            out->pan = clamp8(pan);
        }
    }

    return true;
}
