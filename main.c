#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "bsp/board_api.h"

#include "tusb.h"

// Keyboard on UART0 (bi-directional)
#define KEYB_UART       uart0
#define KEYB_UART_TX    0   // => "KEY RxD" (pin 2, Keyboard Mini-DIN 7-pin)
#define KEYB_UART_RX    1   // => "KEY TxD" (pin 4, Keyboard Mini-DIN 7-pin)

// Mouse on UART1 (uni-directional)
#define MOUSE_UART      uart1
#define MOUSE_UART_TX   4   // => "MOUSE DATA" (pin 4, Keyboard Mini-DIN 7-pin)
                            // or "MSDATA"     (pin 3, Mouse Mini-DIN 5-pin)

// Signaling lines
#define MSCTRL_GPIO     3   // => "MSCTRL"    (pin 2, Mouse Mini-DIN 5-pin)
#define READY_GPIO      5   // => "READY"     (pin 5, Keyboard Mini-DIN 7-pin)

static void gpioISR(uint gpio, uint32_t events);
static void uartISR();

static void processKeybAndMouse(uint32_t deltaTime);
static void flashActivityLED(uint32_t flashRate);

int main()
{
    stdio_init_all();
    board_init();
    tusb_init();

    // Keyboard UART (2400 8n1)
    uart_init(KEYB_UART, 2400);
    uart_set_format(KEYB_UART, 8, 1, UART_PARITY_NONE);
    gpio_set_function(KEYB_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(KEYB_UART_RX, GPIO_FUNC_UART);

    // Mouse UART (4800 8n2)
    uart_init(MOUSE_UART, 4800);
    uart_set_format(MOUSE_UART, 8, 2, UART_PARITY_NONE);
    gpio_set_function(MOUSE_UART_TX, GPIO_FUNC_UART);

    // MSCTRL and READY
    gpio_set_function(MSCTRL_GPIO, GPIO_FUNC_SIO);
    gpio_pull_up(MSCTRL_GPIO);
    gpio_set_dir(MSCTRL_GPIO, GPIO_IN);

    gpio_set_irq_enabled_with_callback(MSCTRL_GPIO, GPIO_IRQ_EDGE_FALL, true, &gpioISR);

    gpio_set_function(READY_GPIO, GPIO_FUNC_SIO);
    gpio_pull_up(READY_GPIO);
    gpio_set_dir(READY_GPIO, GPIO_IN);

    gpio_set_irq_enabled_with_callback(READY_GPIO, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpioISR);

    while (uart_is_readable(KEYB_UART))
        uart_getc(KEYB_UART);

    // UART IRQ is not used
    if (0)
    {
        irq_set_exclusive_handler(UART0_IRQ, &uartISR);
        irq_set_enabled(UART0_IRQ, true);
        uart_set_irqs_enabled(KEYB_UART, true, false);
    }

    uint32_t lastTimer = board_millis();
    while (true)
    {
        uint32_t currentTimer = board_millis();
        uint32_t deltaTime = currentTimer - lastTimer;
        lastTimer = currentTimer;

        flashActivityLED(500);

        tuh_task();

        processKeybAndMouse(deltaTime);
    }
}

static void flashActivityLED(uint32_t flashRate)
{
    static uint32_t lastUpdated = 0;
    static bool activityLED = false;\

    uint32_t currentTimer = board_millis();

    if ((currentTimer - lastUpdated) < flashRate)
        return;

    board_led_write(activityLED);
    activityLED = !activityLED;
    lastUpdated += flashRate;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// TinyUSB callbacks
//

static void processKeybReport(hid_keyboard_report_t const *report);
static void processMouseReport(hid_mouse_report_t const * report);

void tuh_hid_mount_cb(uint8_t devAddr, uint8_t instance, uint8_t const* descReport, uint16_t descLen)
{
    tuh_hid_receive_report(devAddr, instance);
}

void tuh_hid_report_received_cb(uint8_t devAddr, uint8_t instance, const uint8_t *report, uint16_t len)
{
    uint8_t const proto = tuh_hid_interface_protocol(devAddr, instance);

    if (proto == HID_ITF_PROTOCOL_KEYBOARD)
        processKeybReport( (hid_keyboard_report_t const*) report );
    else if (proto == HID_ITF_PROTOCOL_MOUSE)
        processMouseReport( (hid_mouse_report_t const*) report );

    tuh_hid_receive_report(devAddr, instance);
}

int tuh_printf(const char *format, ...)
{
    return 0;
}

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
// x68-HID code
//

// x68000 state
static bool txInhibit               = false;
static bool keyInhibit              = false;
static bool msctrlAsserted          = false;

static uint8_t currentLedLevel      = 0;
static uint8_t currentLedState      = 0;

static uint16_t keyRepeatDelay      = 500; // ms
static uint16_t keyRepeatInterval   = 110; // ms

// USB HID state
static int16_t dx = 0, dy = 0;
static bool lmbPressed              = false;
static bool rmbPressed              = false;
static hid_keyboard_report_t prevReport = { 0, 0, {0} };
static uint8_t keyRepeatKeyCode     = 0x00;
static int32_t keyRepeatCountdown   = 0;

static void sendMouse();

static void gpioISR(uint gpio, uint32_t events)
{
    if (gpio == MSCTRL_GPIO && (events & GPIO_IRQ_EDGE_FALL))
    {
        sendMouse();
    }
    else if (gpio = READY_GPIO)
    {
        txInhibit = (events & GPIO_IRQ_EDGE_FALL);
    }
}

// These are taken from the 'X68000 Technical Guide.pdf', Chapter 5.

#define KEYB_MSCTRL                 0b01000000
#define KEYB_MSCTRL_MASK            0b11111000
#define KEYB_LED_BRIGHTNESS         0b01010100
#define KEYB_LED_BRIGHTNESS_MASK    0b11111100
#define KEYB_KEY_INHIBIT            0b01011000
#define KEYB_KEY_INHIBIT_MASK       0b11111000
#define KEYB_REPEAT_DELAY           0b01100000
#define KEYB_REPEAT_INTERVAL        0b01110000
#define KEYB_REPEAT_MASK            0b11110000
#define KEYB_LED_CTRL_MASK          0b10000000

static void uartISR()
{
    while (uart_is_readable(KEYB_UART))
    {
        uint8_t ch = uart_getc(KEYB_UART);

        if ((ch & KEYB_MSCTRL_MASK) == KEYB_MSCTRL)
        {
            msctrlAsserted = (ch & 0x01) == 0;
        }
        else if ((ch & KEYB_LED_BRIGHTNESS_MASK) == KEYB_LED_BRIGHTNESS)
        {
            uint8_t led_value = ch & 0x03;
            currentLedLevel = led_value;
        }
        else if ((ch & KEYB_KEY_INHIBIT_MASK) == KEYB_KEY_INHIBIT)
        {
            keyInhibit = ch & 0x01 == 0;
        }
        else if ((ch & KEYB_REPEAT_MASK) == KEYB_REPEAT_DELAY)
        {
            uint16_t delayMS = 200 + (ch & 0x0f) * 100;
            keyRepeatDelay = delayMS;
        }
        else if ((ch & KEYB_REPEAT_MASK) == KEYB_REPEAT_INTERVAL)
        {
            uint16_t v = (ch & 0x0f);
            uint16_t intervalMS = 30 + v * v * 5;
            keyRepeatInterval = intervalMS;
        }
        else if ((ch & KEYB_LED_CTRL_MASK) == KEYB_LED_CTRL_MASK)
        {
            uint8_t led = ch & 0x7f;
            currentLedState = led;
        }
    }
}

typedef struct 
{
    union
    {
        uint8_t data[3];
        struct
        {
            union
            {
                struct
                {
                    uint8_t Lbtn:1;
                    uint8_t Rbtn:1;
                    uint8_t unused:2;
                    uint8_t Xover:1;
                    uint8_t Xundr:1;
                    uint8_t Yover:1;
                    uint8_t Yundr:1;
                };
                uint8_t state;
            };
            int8_t  dx;
            int8_t  dy;
        };
    };
    
} MouseData;

static void sendMouse()
{
    if (txInhibit)
        return;

    MouseData mdata =
    {
        .Lbtn  = lmbPressed,
        .Rbtn  = rmbPressed,
        .Xover = dx > 127,
        .Xundr = dx <-128,
        .Yover = dy > 127,
        .Yundr = dy <-128,
        .dx = dx & 0xff,
        .dy = dy & 0xff,
    };

    uart_write_blocking(MOUSE_UART, mdata.data, sizeof(mdata));

    // Reset mouse readings
    dx = 0;
    dy = 0;

    // Early-out if packet was empty
    if (mdata.data[0] == mdata.data[1] == mdata.data[2] == 0x00)
        return;

    flashActivityLED(100);
}

static uint8_t modifierScans[] =
{
    0x71, // "CTRL"  = KEYBOARD_MODIFIER_LEFTCTRL
    0x70, // "SHIFT" = KEYBOARD_MODIFIER_LEFTSHIFT
    0x56, // "XF2"   = KEYBOARD_MODIFIER_LEFTALT
    0x55, // "XF1"   = KEYBOARD_MODIFIER_LEFTGUI
    0x59, // "XF5"   = KEYBOARD_MODIFIER_RIGHTCTRL
    0x70, // "SHIFT" = KEYBOARD_MODIFIER_RIGHTSHIFT
    0x57, // "XF3"   = KEYBOARD_MODIFIER_RIGHTALT
    0x58, // "XF4"   = KEYBOARD_MODIFIER_RIGHTGUI
};

static uint8_t keycodeScans[] =
{
    0x1e,    // "A"         = HID_KEY_A
    0x2e,    // "B"         = HID_KEY_B
    0x2c,    // "C"         = HID_KEY_C
    0x20,    // "D"         = HID_KEY_D
    0x13,    // "E"         = HID_KEY_E
    0x21,    // "F"         = HID_KEY_F
    0x22,    // "G"         = HID_KEY_G
    0x23,    // "H"         = HID_KEY_H
    0x18,    // "I"         = HID_KEY_I
    0x24,    // "J"         = HID_KEY_J
    0x25,    // "K"         = HID_KEY_K
    0x26,    // "L"         = HID_KEY_L
    0x30,    // "M"         = HID_KEY_M
    0x2f,    // "N"         = HID_KEY_N
    0x19,    // "O"         = HID_KEY_O
    0x1a,    // "P"         = HID_KEY_P
    0x11,    // "Q"         = HID_KEY_Q
    0x14,    // "R"         = HID_KEY_R
    0x1f,    // "S"         = HID_KEY_S
    0x15,    // "T"         = HID_KEY_T
    0x17,    // "U"         = HID_KEY_U
    0x2d,    // "V"         = HID_KEY_V
    0x12,    // "W"         = HID_KEY_W
    0x2b,    // "X"         = HID_KEY_X
    0x16,    // "Y"         = HID_KEY_Y
    0x2a,    // "Z"         = HID_KEY_Z
    0x02,    // "1"         = HID_KEY_1
    0x03,    // "2"         = HID_KEY_2
    0x04,    // "3"         = HID_KEY_3
    0x05,    // "4"         = HID_KEY_4
    0x06,    // "5"         = HID_KEY_5
    0x07,    // "6"         = HID_KEY_6
    0x08,    // "7"         = HID_KEY_7
    0x09,    // "8"         = HID_KEY_8
    0x0a,    // "9"         = HID_KEY_9
    0x0b,    // "0"         = HID_KEY_0
    0x1d,    // "RETURN"    = HID_KEY_ENTER
    0x01,    // "ESC"       = HID_KEY_ESCAPE
    0x0f,    // "BS"        = HID_KEY_BACKSPACE
    0x10,    // "TAB"       = HID_KEY_TAB
    0x35,    // "SPACE"     = HID_KEY_SPACE
    0x0c,    // "-"         = HID_KEY_MINUS
    0x0d,    // "^"         = HID_KEY_EQUAL
    0x1b,    // "@"         = HID_KEY_BRACKET_LEFT
    0x1c,    // "["         = HID_KEY_BRACKET_RIGHT
    0x0e,    // "YEN"       = HID_KEY_BACKSLASH
    0x29,    // "]"         = HID_KEY_EUROPE_1
    0x27,    // ";"         = HID_KEY_SEMICOLON
    0x28,    // ":"         = HID_KEY_APOSTROPHE
    0x60,    //"ZENKAKU"    = HID_KEY_GRAVE
    0x31,    // < ,         = HID_KEY_COMMA
    0x32,    // > .         = HID_KEY_PERIOD
    0x33,    // ? /         = HID_KEY_SLASH
    0x5d,    // "CAPS"      = HID_KEY_CAPS_LOCK
    0x63,    // "F1"        = HID_KEY_F1
    0x64,    // "F2"        = HID_KEY_F2
    0x65,    // "F3"        = HID_KEY_F3
    0x66,    // "F4"        = HID_KEY_F4
    0x67,    // "F5"        = HID_KEY_F5
    0x68,    // "F6"        = HID_KEY_F6
    0x69,    // "F7"        = HID_KEY_F7
    0x6a,    // "F8"        = HID_KEY_F8
    0x6b,    // "F9"        = HID_KEY_F9
    0x6c,    // "F10"       = HID_KEY_F10
    0x5a,    // "KANA"      = HID_KEY_F11               (0x72 ?= "OPT.1")
    0x5b,    // "LATIN"     = HID_KEY_F12               (0x73 ?= "OPT.2")
    0x62,    // "COPY"      = HID_KEY_PRINT_SCREEN
    0x54,    // "HELP"      = HID_KEY_SCROLL_LOCK
    0x61,    // "BREAK"     = HID_KEY_PAUSE
    0x5e,    // "INS"       = HID_KEY_INSERT
    0x36,    // "HOME"      = HID_KEY_HOME
    0x38,    // "ROLL UP"   = HID_KEY_PAGE_UP
    0x37,    // "DEL"       = HID_KEY_DELETE
    0x3a,    // "UNDO"      = HID_KEY_END
    0x39,    // "ROLL DOWN" = HID_KEY_PAGE_DOWN
    0x3d,    // "RIGHT"     = HID_KEY_ARROW_RIGHT
    0x3b,    // "LEFT"      = HID_KEY_ARROW_LEFT
    0x3e,    // "DOWN"      = HID_KEY_ARROW_DOWN
    0x3c,    // "UP"        = HID_KEY_ARROW_UP
    0x3f,    // "CLR"       = HID_KEY_NUM_LOCK
    0x40,    // "/"         = HID_KEY_KEYPAD_DIVIDE      (0x52 ?= "SYMBOL INPUT")
    0x41,    // "*"         = HID_KEY_KEYPAD_MULTIPLY    (0x53 ?= "TOROKU")
    0x42,    // "-"         = HID_KEY_KEYPAD_SUBTRACT    (0x5c ?= "CODE INPUT")
    0x46,    // "+"         = HID_KEY_KEYPAD_ADD
    0x4e,    // "ENTER"     = HID_KEY_KEYPAD_ENTER
    0x4b,    // "1"         = HID_KEY_KEYPAD_1
    0x4c,    // "2"         = HID_KEY_KEYPAD_2
    0x4d,    // "3"         = HID_KEY_KEYPAD_3
    0x47,    // "4"         = HID_KEY_KEYPAD_4
    0x48,    // "5"         = HID_KEY_KEYPAD_5
    0x49,    // "6"         = HID_KEY_KEYPAD_6
    0x43,    // "7"         = HID_KEY_KEYPAD_7
    0x44,    // "8"         = HID_KEY_KEYPAD_8
    0x45,    // "9"         = HID_KEY_KEYPAD_9
    0x4f,    // "0"         = HID_KEY_KEYPAD_0
    0x51,    // "."         = HID_KEY_KEYPAD_DECIMAL
    0x0e,    // "YEN"       = HID_KEY_EUROPE_2
};

static void sendKeycode(uint8_t keyCode, bool make)
{
    if (keyCode < HID_KEY_A || (keyCode - HID_KEY_A) >= sizeof(keycodeScans))
        return;

    uint8_t scan = keycodeScans[keyCode - HID_KEY_A];
    scan |= (make ? 0x00 : 0x80);
    uart_write_blocking(KEYB_UART, &scan, sizeof(scan));

    flashActivityLED(100);
}

static bool isPresentInReport(hid_keyboard_report_t const *report, uint8_t keyCode)
{
    for (int i = 0; i < sizeof(report->keycode); ++i)
    {
        if (report->keycode[i] == keyCode)
            return true;
    }
    return false;
}

static void compareKeybReports(hid_keyboard_report_t const* reportA, hid_keyboard_report_t const* reportB, bool make)
{
    for (int i = 0; i < sizeof(reportA->keycode); ++i)
    {
        uint8_t keyCode = reportA->keycode[i];

        // Skip ErrorRollOver, POSTFail & ErrorUndefined
        if (keyCode == HID_KEY_NONE || keyCode <= 0x03)
            continue;

        if (!isPresentInReport(reportB, keyCode))
        {
            sendKeycode(keyCode, make);

            // Record the most recent KEYDOWN, or reset it
            if (make)
            {
                keyRepeatKeyCode = keyCode;
                keyRepeatCountdown = keyRepeatDelay;
            }
            else
            {
                if (keyRepeatKeyCode == keyCode)
                {
                    keyRepeatKeyCode = 0;
                    keyRepeatCountdown = 0;
                }
            }
        }
    }
}

static void processKeybAndMouse(uint32_t deltaTime)
{
    // This should really be done via ISR, but the UART IRQ is limited to >=4 chars, or 32 bitclocks
    if (1)
    {
        bool wasAserted = msctrlAsserted;
        uartISR();
        bool isAsserted = msctrlAsserted;
        if (!wasAserted && isAsserted)
            sendMouse();
    }

    // Handle key repeats
    if (keyRepeatCountdown)
    {
        keyRepeatCountdown -= deltaTime;
        if (keyRepeatCountdown <= 0)
        {
            // send repeat
            sendKeycode(keyRepeatKeyCode, true);
            keyRepeatCountdown = keyRepeatInterval;
        }
    }
}

static void processKeybReport(hid_keyboard_report_t const *report)
{
    // Evaluate modifiers (SHIFT/CTRL/ALT/GUI)
    uint8_t modified = prevReport.modifier ^ report->modifier;
    if (modified)
    {
        for (int i = 0; i < 8; ++i)
        {
            hid_keyboard_modifier_bm_t mod = 1 << i;
            if (modified & mod)
            {
                bool make = report->modifier & mod;
                uint8_t scan = modifierScans[i] | (make ? 0x00 : 0x80);
                uart_write_blocking(KEYB_UART, &scan, sizeof(scan));

                flashActivityLED(100);
            }
        }
    }

    // evaluate BREAK codes (key-up)
    compareKeybReports(&prevReport, report, false);

    // evaluate MAKE codes (key-down)
    compareKeybReports(report, &prevReport, true);

    prevReport = *report;
}

static void processMouseReport(hid_mouse_report_t const * report)
{
    lmbPressed = report->buttons & MOUSE_BUTTON_LEFT;
    rmbPressed = report->buttons & MOUSE_BUTTON_RIGHT;
    dx += report->x;
    dy += report->y;
}
