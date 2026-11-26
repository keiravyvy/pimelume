#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <class/hid/hid.h>
#include <pico/sync.h>
#include <pio_usb.h>
#include <string.h>

#include "config.h"
#include "hid_mouse.h"
#include "usb_host.h"

// TinyUSB host controller instance. On the RP2040 the native USB controller is
// device-only, so rhport 1 is the PIO-based host port.
static Adafruit_USBH_Host USBHost;

// Core 1 (USB callbacks) -> core 0 hand-off. Reports are kept in a short ring
// buffer so button press/release edges are never lost. Consecutive reports with
// the same button state are merged to absorb 1000 Hz mouse traffic.
static constexpr uint8_t kQueueSize = 32;

static mutex_t s_lock;
static MouseReport s_queue[kQueueSize];
static uint8_t s_head = 0;
static uint8_t s_tail = 0;
static uint16_t s_dropped = 0;

static volatile bool s_mouseMounted = false;
static volatile uint8_t s_mouseDevAddr = 0xFF;
static volatile uint8_t s_mouseInstance = 0xFF;

static HidMouseLayout s_layout;

static UsbMouseInfo s_info;

static int16_t clamp16(int32_t value)
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

static int8_t clamp8(int32_t value)
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

static void enqueueReport(const MouseReport &report)
{
    mutex_enter_blocking(&s_lock);

    // Merge with the newest queued report while the button state is unchanged.
    // This keeps the queue from overflowing on high-rate mice without losing
    // click edges (a button change always starts a new entry).
    if (s_head != s_tail)
    {
        const uint8_t last = (uint8_t)((s_head + kQueueSize - 1) % kQueueSize);
        MouseReport &pending = s_queue[last];
        const int32_t dx = (int32_t)pending.dx + report.dx;
        const int32_t dy = (int32_t)pending.dy + report.dy;

        if (pending.buttons == report.buttons && dx >= INT16_MIN &&
            dx <= INT16_MAX && dy >= INT16_MIN && dy <= INT16_MAX)
        {
            pending.dx = (int16_t)dx;
            pending.dy = (int16_t)dy;
            pending.wheel =
                clamp8((int32_t)pending.wheel + report.wheel);
            pending.pan = clamp8((int32_t)pending.pan + report.pan);
            mutex_exit(&s_lock);
            return;
        }
    }

    const uint8_t next = (uint8_t)((s_head + 1) % kQueueSize);
    if (next == s_tail)
    {
        // Full: drop the oldest entry (keeps the most recent button edges).
        s_tail = (uint8_t)((s_tail + 1) % kQueueSize);
        s_dropped++;
    }

    s_queue[s_head] = report;
    s_head = next;
    mutex_exit(&s_lock);
}

// Minimal descriptor sniff used as a fallback when the full parser cannot find
// X/Y (for example a truncated descriptor).
static bool descriptorLooksLikeMouse(const uint8_t *desc, uint16_t len)
{
    if (!desc || len < 2)
    {
        return false;
    }

    bool genericDesktop = false;
    for (uint16_t i = 0; i + 1 < len; i++)
    {
        if (desc[i] == 0x05 && desc[i + 1] == 0x01)
        {
            genericDesktop = true;
        }
        if (genericDesktop && desc[i] == 0x09 && desc[i + 1] == 0x02)
        {
            return true;
        }
    }
    return false;
}

extern "C" {

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                      uint8_t const *desc_report, uint16_t desc_len)
{
    const uint8_t protocol = tuh_hid_interface_protocol(dev_addr, instance);

    HidMouseLayout layout;
    memset(&layout, 0, sizeof(layout));
    const bool parsed =
        desc_report && hidParseMouseLayout(desc_report, desc_len, &layout);

    const bool isMouse =
        (protocol == HID_ITF_PROTOCOL_MOUSE) ||
        (parsed && layout.isMouseCollection) ||
        ((protocol == HID_ITF_PROTOCOL_NONE) &&
         descriptorLooksLikeMouse(desc_report, desc_len));

    if (!isMouse)
    {
        return;
    }

    // Only forward one pointer interface. Composite mice often expose several
    // HID interfaces (mouse, consumer control, vendor); the first mouse-like
    // one is the actual pointer.
    if (s_mouseMounted)
    {
        return;
    }

    uint16_t vid = 0;
    uint16_t pid = 0;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    s_mouseDevAddr = dev_addr;
    s_mouseInstance = instance;

    // Prefer report protocol so wheel, extra buttons and high-resolution axes
    // are available. Only do this when we successfully parsed the descriptor;
    // otherwise stay in boot protocol and use the fallback parser.
    if (protocol == HID_ITF_PROTOCOL_MOUSE && parsed)
    {
        tuh_hid_set_protocol(dev_addr, instance, HID_PROTOCOL_REPORT);
    }

    mutex_enter_blocking(&s_lock);
    s_layout = parsed ? layout : HidMouseLayout{};
    s_head = 0;
    s_tail = 0;
    s_dropped = 0;

    memset(&s_info, 0, sizeof(s_info));
    s_info.connected = true;
    s_info.vid = vid;
    s_info.pid = pid;
    s_info.reportId = s_layout.reportId;
    s_info.hasX = s_layout.hasX;
    s_info.hasY = s_layout.hasY;
    s_info.hasWheel = s_layout.hasWheel;
    s_info.hasPan = s_layout.hasPan;
    s_info.hasButtons = s_layout.hasButtons;
    s_info.buttonCount = s_layout.buttonCount;
    s_info.xBits = s_layout.xBits;
    s_info.yBits = s_layout.yBits;
    s_mouseMounted = true;
    mutex_exit(&s_lock);

    tuh_hid_receive_report(dev_addr, instance);
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    if (dev_addr != s_mouseDevAddr || instance != s_mouseInstance)
    {
        return;
    }

    s_mouseMounted = false;
    s_mouseDevAddr = 0xFF;
    s_mouseInstance = 0xFF;

    mutex_enter_blocking(&s_lock);
    s_head = 0;
    s_tail = 0;
    s_info.connected = false;
    mutex_exit(&s_lock);
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                uint8_t const *report, uint16_t len)
{
    if (dev_addr == s_mouseDevAddr && instance == s_mouseInstance)
    {
        MouseReport parsed;
        // Mount and report callbacks both run from USBHost.task() on core 1,
        // so s_layout needs no locking here.
        const bool bootProtocol =
            tuh_hid_get_protocol(dev_addr, instance) == HID_PROTOCOL_BOOT;
        if (hidParseMouseReport(s_layout, report, len, &parsed, bootProtocol))
        {
            enqueueReport(parsed);
        }

        // Keep the mouse interrupt endpoint armed.
        tuh_hid_receive_report(dev_addr, instance);
    }
}

} // extern "C"

void usbHostBegin()
{
    mutex_init(&s_lock);

#if PIN_USB_VBUS_EN >= 0
    pinMode(PIN_USB_VBUS_EN, OUTPUT);
    digitalWrite(PIN_USB_VBUS_EN, PIN_USB_VBUS_EN_ACTIVE_HIGH ? HIGH : LOW);
#endif

    pio_usb_configuration_t config = PIO_USB_DEFAULT_CONFIG;
    config.pin_dp = PIN_USB_HOST_DP;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    // On the Pico W the CYW43 driver also uses PIO, so move the PIO-USB state
    // machines to resources that do not collide with it.
    // See https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/46
    config.sm_tx = 3;
    config.sm_rx = 2;
    config.sm_eop = 3;
    config.pio_rx_num = 0;
    config.pio_tx_num = 1;
    config.tx_ch = 9;
#endif

    USBHost.configure_pio_usb(1, &config);
    USBHost.begin(1);
}

void usbHostTask()
{
    USBHost.task();
}

bool usbHostTakeMouseReport(MouseReport *out)
{
    bool haveReport = false;

    mutex_enter_blocking(&s_lock);
    if (s_head != s_tail)
    {
        *out = s_queue[s_tail];
        s_tail = (uint8_t)((s_tail + 1) % kQueueSize);
        haveReport = true;
    }
    mutex_exit(&s_lock);

    return haveReport;
}

bool usbHostMouseConnected()
{
    return s_mouseMounted;
}

bool usbHostGetMouseInfo(UsbMouseInfo *out)
{
    if (!out)
    {
        return false;
    }

    mutex_enter_blocking(&s_lock);
    *out = s_info;
    out->dropped = s_dropped;
    mutex_exit(&s_lock);

    return out->connected;
}
