/*
 * ClearCore_WS2811_MQTT.ino
 *
 * Teknic ClearCore -> WS2811 COB LED strip, controlled over Ethernet / MQTT.
 * Supports plain RGB WS2811 strips (one IC per segment) and the BTF-LIGHTING
 * "Dual IC" RGBCCT FCOB strip (two WS2811 ICs per segment: RGB + CW/WW).
 *
 * ---------------------------------------------------------------------------
 * MQTT
 *   Subscribes   <MQTT_PREFIX>/cmd      text commands, one per message
 *   Publishes    <MQTT_PREFIX>/state    JSON, retained: current effect, colours,
 *                                       parameters, fps, estimated watts, limiter, ip
 *                <MQTT_PREFIX>/status   "online" / "offline" (retained, last-will)
 *                <MQTT_PREFIX>/reply    "ok ..." / "error ..." for each command
 *
 * COMMANDS  (colours: #RRGGBB, #RRGGBBCCWW, r,g,b, r,g,b,cw,ww, or a name:
 *            red green blue yellow cyan magenta orange purple pink amber
 *            white warm cool rgbwhite black)
 *
 *   off                                 everything dark (state is remembered)
 *   on                                  restore what ran before "off"
 *   solid <colour>                      one colour everywhere          (alias: color)
 *   white [kelvin] [level]              tunable white, 2700-6500 K, level 0-255
 *   chase [colour] [bg] [speed]         comet with a fading tail        size= tail length
 *   scanner [colour] [bg] [speed]       Larson scanner, bouncing bar    size= bar width
 *   marquee [c1] [c2] [speed]           scrolling candy-cane blocks     size= block length
 *   alternate [c1] [c2] [rate]          blocks that swap colours        size= block length
 *   flash [colour] [bg] [rate]          strobe, rate in Hz
 *   breathe [colour] [rate]             sine fade in/out, rate in Hz
 *   rainbow [speed]                     scrolling hue wheel             size= pixels per cycle
 *   wipe [colour] [bg] [speed]          fills from the start, then empties
 *   sparkle [colour] [bg] [rate]        random sparks, rate = sparks per second
 *   fire [rate]                         flames, rate = intensity (0.3 .. 3)
 *   police [c1] [c2] [rate]             halves flash alternately
 *   cycle [seconds]                     rotates through a playlist of the above
 *   brightness <0-255>                  global dimmer (keeps the effect)
 *   speed <px/s> | rate <Hz> | size <px> | dir fwd|rev     tweak the running effect
 *   status                              publish state now
 *
 *   Any command also accepts key=value pairs: speed= rate= size= dir= bri= kelvin=
 *   Examples:  "chase #ff4000 speed=80 size=12"   "alternate red green rate=0.5 size=6"
 *              "white 3000 180"   "sparkle white #000020 rate=40"   "cycle 20"
 *
 * ---------------------------------------------------------------------------
 * SIGNAL (unchanged from the non-network version)
 *   WS2811 waveform is synthesised on the COM port's SPI MOSI line (5 V) and
 *   pushed out by DMA; the CPU renders the next frame while one is on the wire.
 *   SPI clock from GCLK7 = 10 MHz -> 2.5 MHz (3 SPI bits per data bit:
 *   '0' = 400/800 ns, '1' = 800/400 ns) or 5 MHz (TIMING_PROFILE 1).
 *   Wiring: COM port MOSI -> strip DIN, GND -> strip GND; RJ-45 pins from the
 *   ClearCore Hardware Manual, "COM-0, 1 configured for SPI devices". Keep the
 *   data cable short and scope the strip's DIN once (clean ~5 V, 400/800 ns).
 *
 * POWER
 *   Strip + ClearCore on one 24 V / 250 W supply. The limiter estimates every
 *   frame's draw and scales it to LED_BUDGET_WATTS. Inject 24 V at both ends.
 *
 * NETWORK
 *   DHCP or static IP (ClearCore library has no DNS resolver for the client,
 *   so the broker is given as an IP). The MQTT client here is a small
 *   self-contained MQTT 3.1.1 implementation (QoS 0, keep-alive, last-will,
 *   auto-reconnect) on the ClearCore EthernetTcpClient; no extra libraries.
 *   While the link/IP/broker is missing, the last segment blinks
 *   (red: no link, orange: no IP, yellow: no broker) unless NET_STATUS_PIXEL 0.
 */

#include "ClearCore.h"
#include "EthernetTcpClient.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>

// ============================================================================
// 1. STRIP  -- set these from the strip's label / datasheet
// ============================================================================
enum PixelType  { PIXEL_RGB = 1, PIXEL_RGBCCT_2IC = 2 };   // value = WS2811 ICs per addressable segment
enum ColorOrder { ORDER_RGB, ORDER_RBG, ORDER_GRB, ORDER_GBR, ORDER_BRG, ORDER_BGR };
enum CctOrder   { CCT_CW_WW, CCT_WW_CW };

#define PIXEL_TYPE          PIXEL_RGBCCT_2IC // BTF-LIGHTING FCOB RGBCCT "Dual IC": 2 x WS2811 per segment.
                                            // Use PIXEL_RGB for an ordinary RGB WS2811 strip.
#define STRIP_LENGTH_M      7.5f
#define PIXELS_PER_M        14      // addressable SEGMENTS per metre (BTF RGBCCT: 28 ICs/m / 2 = 14)
#define STRIP_W_PER_M       23.0f   // watts per metre with every channel full (BTF: 18-23 W/m; larger used)
#define STRIP_IDLE_W_PER_M  0.0f    // standby draw per metre if the spec lists one, else 0
#define STRIP_KHZ           800     // 800 for normal WS2811 strips; 400 only for low-speed-mode parts
#define COLOR_ORDER         ORDER_RGB   // byte order on the RGB IC
#define CCT_ORDER           CCT_CW_WW   // byte order on the white IC; check with "white 2700" vs "white 6500"

// ============================================================================
// 2. POWER  -- 250 W supply shared with the ClearCore
// ============================================================================
#define PSU_WATTS           250.0f
#define PSU_HEADROOM        0.80f   // plan for at most 80 % of the supply's nameplate
#define CLEARCORE_WATTS     7.2f    // ClearCore: 300 mA @ 24 V (Hardware Manual, Appendix D)
#define LED_BUDGET_WATTS    (PSU_WATTS * PSU_HEADROOM - CLEARCORE_WATTS)   // 192.8 W

// ============================================================================
// 3. OUTPUT
// ============================================================================
#define USE_TWO_PORTS       0   // 1: COM-0 drives the first half, COM-1 the second half (cut the data trace)
#define TIMING_PROFILE      0   // 0 = NeoPixel-style 400/800 ns @ 2.5 MHz, 1 = WS2811-datasheet 200/600 ns @ 5 MHz
#define GAMMA_CORRECT       1
#define LATCH_US            300
#define REPORT_STATS        1   // USB serial log (115200): network events, fps, power

// ============================================================================
// 4. NETWORK / MQTT
// ============================================================================
#define USE_DHCP            1
#define STATIC_IP           "192.168.1.50"      // used when USE_DHCP is 0
#define STATIC_NETMASK      "255.255.255.0"
#define STATIC_GATEWAY      "192.168.1.1"
#define MQTT_BROKER_IP      "192.168.1.10"      // IP address of the broker (no hostname lookup)
#define MQTT_PORT           1883
#define MQTT_CLIENT_ID      "clearcore-leds"
#define MQTT_USER           ""                  // "" = anonymous
#define MQTT_PASS           ""
#define MQTT_PREFIX         "clearcore/leds"
#define MQTT_KEEPALIVE_S    30
#define MQTT_RECONNECT_MS   5000
#define STATE_PERIOD_MS     10000               // periodic state publish
#define NET_STATUS_PIXEL    1                   // blink the last segment while the network is down
#define BOOT_COMMAND        "rainbow"           // what runs after power-up

// ============================================================================
// Derived constants
// ============================================================================
#if STRIP_KHZ == 400
#undef  TIMING_PROFILE
#define TIMING_PROFILE 1
#endif
#if TIMING_PROFILE == 0
static const uint32_t SPI_HZ                = 2500000UL;
static const int      SPI_BITS_PER_DATA_BIT = 3;
static const uint32_t PATTERN_ZERO          = 0x4;       // 100
static const uint32_t PATTERN_ONE           = 0x6;       // 110
#else
static const uint32_t SPI_HZ                = (STRIP_KHZ == 800) ? 5000000UL : 2500000UL;
static const int      SPI_BITS_PER_DATA_BIT = 6;
static const uint32_t PATTERN_ZERO          = 0x20;      // 100000
static const uint32_t PATTERN_ONE           = 0x38;      // 111000
#endif

static const int      ICS_PER_PIXEL        = (int)PIXEL_TYPE;
static const int      WIRE_BYTES_PER_PIXEL = 3 * ICS_PER_PIXEL;
static const int      ACTIVE_CHANNELS      = (ICS_PER_PIXEL == 2) ? 5 : 3;
static const int      BYTES_PER_COLOR      = SPI_BITS_PER_DATA_BIT;
static const int      BYTES_PER_PIXEL      = WIRE_BYTES_PER_PIXEL * BYTES_PER_COLOR;
static const int      NUM_PIXELS           = (int)(STRIP_LENGTH_M * PIXELS_PER_M + 0.5f);
static const int      NUM_PORTS            = USE_TWO_PORTS ? 2 : 1;
static const int      PORT_PIXELS          = (NUM_PIXELS + NUM_PORTS - 1) / NUM_PORTS;
static const uint32_t LATCH_BYTES          = ((uint32_t)LATCH_US * (SPI_HZ / 1000UL) + 7999UL) / 8000UL;
static const uint32_t PORT_BYTES           = (uint32_t)PORT_PIXELS * BYTES_PER_PIXEL + LATCH_BYTES;
static const float    WATTS_PER_COUNT      = STRIP_W_PER_M / PIXELS_PER_M / ACTIVE_CHANNELS / 255.0f;
static const float    STRIP_IDLE_W         = STRIP_IDLE_W_PER_M * STRIP_LENGTH_M;
static const float    FRAME_WIRE_US        = (float)PORT_BYTES * 8.0f * 1.0e6f / (float)SPI_HZ;

static_assert(NUM_PIXELS > 1, "need at least two segments");
static_assert(PORT_BYTES <= 65535UL, "one DMA block is limited to 65535 bytes; use USE_TWO_PORTS");
static_assert(LED_BUDGET_WATTS > 0.0f, "no power left for the LEDs");

// ============================================================================
// Types (all declared before the first function so the Arduino preprocessor
// can generate prototypes that use them)
// ============================================================================
struct Pixel { uint8_t r, g, b, cw, ww; };

struct LedPort {
    SerialDriver *com;
    DmaChannels   rxChannel;
    int           firstPixel;
    int           numPixels;
    uint32_t      bytes;
    uint8_t      *buf[2];
};

enum Effect {
    FX_OFF, FX_SOLID, FX_WHITE, FX_CHASE, FX_SCANNER, FX_MARQUEE, FX_ALTERNATE,
    FX_FLASH, FX_BREATHE, FX_RAINBOW, FX_WIPE, FX_SPARKLE, FX_FIRE, FX_POLICE,
    FX_CYCLE, FX_COUNT
};

struct Scene {
    Effect fx;
    Pixel  c1, c2;
    float  speed;    // pixels per second
    float  rate;     // Hz (or sparks/s, or fire intensity)
    int    size;     // pixels
    int    dir;      // +1 / -1
    int    kelvin;   // white
    int    level;    // white intensity 0-255
    int    period;   // cycle seconds
};

struct NamedColor { const char *name; Pixel c; };

// ============================================================================
// LED output engine
// ============================================================================
static Pixel    frame[NUM_PIXELS];
static uint8_t  spiBuf[NUM_PORTS][2][PORT_BYTES] __attribute__((aligned(4)));
static LedPort  ports[NUM_PORTS];
static uint8_t  lut[256][BYTES_PER_COLOR];
static uint8_t  gammaLut[256];
static uint8_t  wireFrame[NUM_PIXELS * WIRE_BYTES_PER_PIXEL];

static uint8_t  g_brightness = 255;
static float    g_estWatts   = 0.0f;
static uint8_t  g_powerScale = 255;
static uint32_t g_frames     = 0;
static uint32_t g_fps        = 0;

static inline uint8_t scale8(uint8_t v, uint8_t s) {
    return (uint8_t)(((uint16_t)v * (uint16_t)(s + 1)) >> 8);
}

static void buildLuts() {
    for (int v = 0; v < 256; v++) {
        uint64_t bits = 0;
        for (int i = 7; i >= 0; i--) {
            bits = (bits << SPI_BITS_PER_DATA_BIT) | (((v >> i) & 1) ? PATTERN_ONE : PATTERN_ZERO);
        }
        for (int b = 0; b < BYTES_PER_COLOR; b++) {
            lut[v][b] = (uint8_t)(bits >> (8 * (BYTES_PER_COLOR - 1 - b)));
        }
        gammaLut[v] = GAMMA_CORRECT ? (uint8_t)(powf((float)v / 255.0f, 2.2f) * 255.0f + 0.5f) : (uint8_t)v;
    }
}

static void portInit(LedPort &p, SerialDriver &com, DmaChannels rxChannel,
                     int firstPixel, int numPixels, uint8_t *buf0, uint8_t *buf1) {
    p.com = &com; p.rxChannel = rxChannel; p.firstPixel = firstPixel; p.numPixels = numPixels;
    p.bytes = (uint32_t)numPixels * BYTES_PER_PIXEL + LATCH_BYTES;
    p.buf[0] = buf0; p.buf[1] = buf1;
    memset(buf0, 0, PORT_BYTES);
    memset(buf1, 0, PORT_BYTES);
    com.Mode(Connector::SPI);
    com.Speed(SPI_HZ);
    com.DataOrder(SerialDriver::COM_MSB_FIRST);
    com.SpiClock(SerialDriver::SCK_LOW, SerialDriver::LEAD_CHANGE);
    com.PortOpen();
    com.SpiSsMode(SerialDriver::LINE_OFF);
    com.SpiTransferData(buf0, nullptr, LATCH_BYTES);     // start from a clean reset
}

static inline bool portBusy(const LedPort &p) {
    return DmaManager::Channel(p.rxChannel)->CHCTRLA.bit.ENABLE != 0;
}

static inline void portStart(LedPort &p, int which) {
    if (!p.com->SpiTransferDataAsync(p.buf[which], nullptr, (int32_t)p.bytes)) {
        p.com->SpiTransferData(p.buf[which], nullptr, (int32_t)p.bytes);
    }
}

static uint32_t prepareFrame() {
    static const uint8_t idx[6][3] = { {0,1,2}, {0,2,1}, {1,0,2}, {1,2,0}, {2,0,1}, {2,1,0} };
    const uint8_t *o = idx[COLOR_ORDER];
    uint32_t sum = 0;
    uint8_t *w = wireFrame;
    for (int i = 0; i < NUM_PIXELS; i++) {
        const Pixel &px = frame[i];
        uint8_t c[3] = { px.r, px.g, px.b };
        uint8_t v0 = scale8(gammaLut[c[o[0]]], g_brightness);
        uint8_t v1 = scale8(gammaLut[c[o[1]]], g_brightness);
        uint8_t v2 = scale8(gammaLut[c[o[2]]], g_brightness);
        sum += (uint32_t)v0 + v1 + v2;
        *w++ = v0; *w++ = v1; *w++ = v2;
        if (ICS_PER_PIXEL == 2) {
            uint8_t cw = scale8(gammaLut[px.cw], g_brightness);
            uint8_t ww = scale8(gammaLut[px.ww], g_brightness);
            sum += (uint32_t)cw + ww;
            if (CCT_ORDER == CCT_CW_WW) { *w++ = cw; *w++ = ww; } else { *w++ = ww; *w++ = cw; }
            *w++ = 0;
        }
    }
    return sum;
}

static uint8_t powerScaleFor(uint32_t sum) {
    float est = (float)sum * WATTS_PER_COUNT + STRIP_IDLE_W;
    float budget = LED_BUDGET_WATTS;
    if (est <= budget || est <= STRIP_IDLE_W) { g_estWatts = est; return 255; }
    float s = (budget - STRIP_IDLE_W) / (est - STRIP_IDLE_W);
    if (s < 0.0f) s = 0.0f;
    uint8_t s8 = (uint8_t)(s * 255.0f);
    g_estWatts = (float)sum * ((float)(s8 + 1) / 256.0f) * WATTS_PER_COUNT + STRIP_IDLE_W;
    return s8;
}

static void encodeFrame(int which, uint8_t powerScale) {
    for (int pi = 0; pi < NUM_PORTS; pi++) {
        LedPort &p = ports[pi];
        uint8_t *dst = p.buf[which];
        const uint8_t *src = &wireFrame[p.firstPixel * WIRE_BYTES_PER_PIXEL];
        const int n = p.numPixels * WIRE_BYTES_PER_PIXEL;
        for (int k = 0; k < n; k++) {
            uint8_t v = (powerScale == 255) ? src[k] : scale8(src[k], powerScale);
            memcpy(dst, lut[v], BYTES_PER_COLOR);
            dst += BYTES_PER_COLOR;
        }
    }
}

// Push frame[] out. Returns once the DMA for this frame has STARTED.
static void show() {
    static int which = 0;
    uint32_t sum = prepareFrame();
    g_powerScale = powerScaleFor(sum);
    encodeFrame(which, g_powerScale);
    for (int pi = 0; pi < NUM_PORTS; pi++) ports[pi].com->SpiAsyncWaitComplete();
    for (int pi = 0; pi < NUM_PORTS; pi++) portStart(ports[pi], which);
    which ^= 1;
    g_frames++;
}

// ============================================================================
// Colour helpers
// ============================================================================
static inline Pixel rgb(uint8_t r, uint8_t g, uint8_t b) { Pixel p = { r, g, b, 0, 0 }; return p; }
static inline Pixel black() { Pixel p = { 0, 0, 0, 0, 0 }; return p; }
static inline Pixel stripWhite() {                      // "white": real whites on CCT strips
    if (ICS_PER_PIXEL == 2) { Pixel p = { 0, 0, 0, 255, 255 }; return p; }
    return rgb(255, 255, 255);
}
static inline uint8_t lerp8(uint8_t a, uint8_t b, uint8_t k) {
    return (uint8_t)(a + (((int)b - (int)a) * (int)k + 127) / 255);
}
static inline Pixel mix(const Pixel &a, const Pixel &b, uint8_t k) {   // k = 0 -> a, 255 -> b
    Pixel p = { lerp8(a.r, b.r, k), lerp8(a.g, b.g, k), lerp8(a.b, b.b, k), lerp8(a.cw, b.cw, k), lerp8(a.ww, b.ww, k) };
    return p;
}
static inline Pixel dim(const Pixel &a, uint8_t k) {
    Pixel p = { scale8(a.r, k), scale8(a.g, k), scale8(a.b, k), scale8(a.cw, k), scale8(a.ww, k) };
    return p;
}

static Pixel hueWheel(uint8_t h) {
    if (h < 85)  return rgb(255 - h * 3, h * 3, 0);
    if (h < 170) { h -= 85;  return rgb(0, 255 - h * 3, h * 3); }
    h -= 170;    return rgb(h * 3, 0, 255 - h * 3);
}

// Tunable white. CCT strips mix the two white ICs; RGB strips approximate.
static Pixel whiteKelvin(int kelvin, uint8_t level) {
    if (kelvin < 2700) kelvin = 2700;
    if (kelvin > 6500) kelvin = 6500;
    if (ICS_PER_PIXEL == 2) {
        // strip whites are ~3000 K (ww) and ~6000 K (cw)
        float f = (float)(kelvin - 3000) / 3000.0f;
        if (f < 0.0f) f = 0.0f;
        if (f > 1.0f) f = 1.0f;
        Pixel p = { 0, 0, 0, (uint8_t)(level * f + 0.5f), (uint8_t)(level * (1.0f - f) + 0.5f) };
        return p;
    }
    float f = (float)(kelvin - 2700) / 3800.0f;           // 0 = warm, 1 = cool
    Pixel p = { level, (uint8_t)(level * (0.72f + 0.28f * f)), (uint8_t)(level * (0.40f + 0.60f * f)), 0, 0 };
    return p;
}

static const NamedColor NAMED_COLORS[] = {
    { "red",     { 255,   0,   0,   0,   0 } },
    { "green",   {   0, 255,   0,   0,   0 } },
    { "blue",    {   0,   0, 255,   0,   0 } },
    { "yellow",  { 255, 200,   0,   0,   0 } },
    { "cyan",    {   0, 255, 255,   0,   0 } },
    { "magenta", { 255,   0, 255,   0,   0 } },
    { "orange",  { 255,  80,   0,   0,   0 } },
    { "purple",  { 128,   0, 255,   0,   0 } },
    { "pink",    { 255,  40, 120,   0,   0 } },
    { "amber",   { 255, 140,   0,   0,   0 } },
    { "warm",    {   0,   0,   0,   0, 255 } },
    { "cool",    {   0,   0,   0, 255,   0 } },
    { "rgbwhite",{ 255, 255, 255,   0,   0 } },
    { "black",   {   0,   0,   0,   0,   0 } },
    { "off",     {   0,   0,   0,   0,   0 } },
};

static int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

// #RRGGBB, #RRGGBBCCWW, "r,g,b", "r,g,b,cw,ww", or a name. Returns false if not a colour.
static bool parseColor(const char *tok, Pixel *out) {
    if (tok[0] == '#') {
        size_t n = strlen(tok + 1);
        if (n != 6 && n != 10) return false;
        uint8_t v[5] = { 0, 0, 0, 0, 0 };
        for (size_t i = 0; i < n / 2; i++) {
            int hi = hexNibble(tok[1 + 2 * i]), lo = hexNibble(tok[2 + 2 * i]);
            if (hi < 0 || lo < 0) return false;
            v[i] = (uint8_t)(hi * 16 + lo);
        }
        Pixel p = { v[0], v[1], v[2], v[3], v[4] };
        *out = p;
        return true;
    }
    if (strchr(tok, ',') != nullptr) {
        int v[5] = { 0, 0, 0, 0, 0 };
        int count = 0;
        const char *s = tok;
        while (*s && count < 5) {
            char *end;
            long val = strtol(s, &end, 10);
            if (end == s) return false;
            if (val < 0) val = 0;
            if (val > 255) val = 255;
            v[count++] = (int)val;
            s = end;
            if (*s == ',') s++;
            else if (*s) return false;
        }
        if (count != 3 && count != 5) return false;
        Pixel p = { (uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2], (uint8_t)v[3], (uint8_t)v[4] };
        *out = p;
        return true;
    }
    if (strcmp(tok, "white") == 0) { *out = stripWhite(); return true; }
    for (size_t i = 0; i < sizeof(NAMED_COLORS) / sizeof(NAMED_COLORS[0]); i++) {
        if (strcmp(tok, NAMED_COLORS[i].name) == 0) { *out = NAMED_COLORS[i].c; return true; }
    }
    return false;
}

static void colorToHex(const Pixel &c, char *out) {       // out: at least 12 chars
    if (ICS_PER_PIXEL == 2) snprintf(out, 12, "#%02x%02x%02x%02x%02x", c.r, c.g, c.b, c.cw, c.ww);
    else                    snprintf(out, 12, "#%02x%02x%02x", c.r, c.g, c.b);
}

// ============================================================================
// Effects
// ============================================================================
static const char *const FX_NAMES[FX_COUNT] = {
    "off", "solid", "white", "chase", "scanner", "marquee", "alternate",
    "flash", "breathe", "rainbow", "wipe", "sparkle", "fire", "police", "cycle"
};

static Scene    g_scene;            // what is rendering now
static Scene    g_saved;            // for "on" after "off"
static bool     g_cycleActive = false;
static int      g_cycleIndex  = 0;
static uint32_t g_cycleStart  = 0;
static int      g_cyclePeriod = 15;
static uint8_t  heat[NUM_PIXELS];   // fire
static uint8_t  spark[NUM_PIXELS];  // sparkle
static uint32_t g_fxTick      = 0;  // last fixed-step update (fire/sparkle)
static uint32_t g_rng         = 0x9E3779B9u;

static const char *const CYCLE_PLAYLIST[] = {
    "rainbow speed=25",
    "chase #ff4000 speed=60 size=12",
    "scanner red size=8 speed=50",
    "marquee red green speed=12 size=5",
    "sparkle white #000018 rate=40",
    "breathe #0040ff rate=0.2",
    "fire",
    "alternate cyan magenta rate=0.7 size=7",
    "wipe blue speed=45",
    "police",
};

static inline uint32_t rnd() { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return g_rng; }

// Fraction 0..1 of a cycle at `hz`, and position 0..period at `pxPerS`, both
// free of float drift: integer math on the millisecond clock.
static float phase(uint32_t nowMs, float hz) {
    uint32_t milliHz = (uint32_t)(hz * 1000.0f + 0.5f);
    if (milliHz == 0) return 0.0f;
    uint64_t x = (uint64_t)nowMs * milliHz;      // 1e-6 cycles
    return (float)(x % 1000000ULL) / 1000000.0f;
}
static float travel(uint32_t nowMs, float pxPerS, int periodPx) {
    uint32_t milli = (uint32_t)(fabsf(pxPerS) * 1000.0f + 0.5f);
    if (milli == 0 || periodPx <= 0) return 0.0f;
    uint64_t x = (uint64_t)nowMs * milli;        // 1e-6 pixels
    return (float)(x % ((uint64_t)periodPx * 1000000ULL)) / 1000000.0f;
}

static void fill(const Pixel &c) { for (int i = 0; i < NUM_PIXELS; i++) frame[i] = c; }

static void defaultsFor(Effect fx, Scene &s) {
    s.fx = fx; s.c1 = stripWhite(); s.c2 = black();
    s.speed = 30.0f; s.rate = 1.0f; s.size = 5; s.dir = 1; s.kelvin = 4000; s.level = 255; s.period = 15;
    switch (fx) {
        case FX_SOLID:     s.c1 = stripWhite(); break;
        case FX_WHITE:     break;
        case FX_CHASE:     s.c1 = rgb(255, 64, 0);  s.speed = 40; s.size = 8; break;
        case FX_SCANNER:   s.c1 = rgb(255, 0, 0);   s.speed = 40; s.size = 6; break;
        case FX_MARQUEE:   s.c1 = rgb(255, 0, 0);   s.c2 = rgb(255, 255, 255); s.speed = 10; s.size = 4; break;
        case FX_ALTERNATE: s.c1 = rgb(255, 0, 0);   s.c2 = rgb(0, 255, 0); s.rate = 1.0f; s.size = 4; break;
        case FX_FLASH:     s.rate = 2.0f; break;
        case FX_BREATHE:   s.c1 = rgb(0, 0, 255);   s.rate = 0.25f; break;
        case FX_RAINBOW:   s.speed = 20; s.size = NUM_PIXELS / 2 > 0 ? NUM_PIXELS / 2 : 1; break;
        case FX_WIPE:      s.c1 = rgb(0, 0, 255);   s.speed = 30; break;
        case FX_SPARKLE:   s.rate = 20.0f; break;
        case FX_FIRE:      s.rate = 1.0f; break;
        case FX_POLICE:    s.c1 = rgb(255, 0, 0);   s.c2 = rgb(0, 0, 255); s.rate = 2.5f; break;
        default: break;
    }
}

static void renderFire(uint32_t nowMs) {
    // fixed 60 Hz simulation so the look does not depend on frame rate
    float intensity = g_scene.rate;
    if (intensity < 0.3f) intensity = 0.3f;
    if (intensity > 3.0f) intensity = 3.0f;
    while ((int32_t)(nowMs - g_fxTick) >= 16) {
        g_fxTick += 16;
        for (int i = 0; i < NUM_PIXELS; i++) {                           // cool
            int cool = (int)(rnd() % (uint32_t)(10 + 180 / NUM_PIXELS));
            heat[i] = (uint8_t)(cool >= heat[i] ? 0 : heat[i] - cool);
        }
        for (int i = NUM_PIXELS - 1; i >= 2; i--) {                      // drift along the strip
            heat[i] = (uint8_t)((heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3);
        }
        int sparks = 1 + (int)(2.0f * intensity);
        for (int k = 0; k < sparks; k++) {                               // ignite near the start
            if ((rnd() & 255) < (uint32_t)(60 * intensity)) {
                int i = (int)(rnd() % 4);
                int h = heat[i] + 160 + (int)(rnd() % 96);
                heat[i] = (uint8_t)(h > 255 ? 255 : h);
            }
        }
    }
    for (int i = 0; i < NUM_PIXELS; i++) {                               // black -> red -> yellow -> white
        uint8_t h = heat[i];
        Pixel p = black();
        if (h < 85)       { p.r = (uint8_t)(h * 3); }
        else if (h < 170) { p.r = 255; p.g = (uint8_t)((h - 85) * 3); }
        else              { p.r = 255; p.g = 255; p.b = (uint8_t)((h - 170) * 3); }
        frame[i] = p;
    }
}

static void renderSparkle(uint32_t nowMs) {
    while ((int32_t)(nowMs - g_fxTick) >= 16) {
        g_fxTick += 16;
        for (int i = 0; i < NUM_PIXELS; i++) spark[i] = (uint8_t)(spark[i] > 12 ? spark[i] - 12 : 0);
        // expected sparks per tick = rate * 0.016; spawn with that probability (x1000 resolution)
        uint32_t perMille = (uint32_t)(g_scene.rate * 16.0f);
        while (perMille >= 1000) { spark[rnd() % NUM_PIXELS] = 255; perMille -= 1000; }
        if ((rnd() % 1000) < perMille) spark[rnd() % NUM_PIXELS] = 255;
    }
    for (int i = 0; i < NUM_PIXELS; i++) frame[i] = mix(g_scene.c2, g_scene.c1, spark[i]);
}

static void renderScene(uint32_t nowMs) {
    Scene &s = g_scene;
    switch (s.fx) {
        case FX_OFF:   fill(black()); break;
        case FX_SOLID: fill(s.c1); break;
        case FX_WHITE: fill(whiteKelvin(s.kelvin, (uint8_t)s.level)); break;

        case FX_CHASE: {                                     // comet with a fading tail
            int size = s.size < 1 ? 1 : s.size;
            float head = travel(nowMs, s.speed, NUM_PIXELS + size);
            for (int i = 0; i < NUM_PIXELS; i++) {
                float pos = (s.dir >= 0) ? (float)i : (float)(NUM_PIXELS - 1 - i);
                float d = head - pos;                        // distance behind the head
                uint8_t k = 0;
                if (d >= 0.0f && d < (float)size) {
                    float f = 1.0f - d / (float)size;
                    k = (uint8_t)(f * f * 255.0f);
                }
                frame[i] = mix(s.c2, s.c1, k);
            }
            break;
        }
        case FX_SCANNER: {                                   // bouncing bar
            int size = s.size < 1 ? 1 : s.size;
            int span = NUM_PIXELS - size;
            if (span < 1) span = 1;
            float x = travel(nowMs, s.speed, 2 * span);
            float pos = (x <= (float)span) ? x : (float)(2 * span) - x;
            for (int i = 0; i < NUM_PIXELS; i++) {
                float d = fabsf((float)i - (pos + (float)size * 0.5f)) / ((float)size * 0.5f);
                uint8_t k = d >= 1.0f ? 0 : (uint8_t)((1.0f - d) * 255.0f);
                frame[i] = mix(s.c2, s.c1, k);
            }
            break;
        }
        case FX_MARQUEE: {                                   // scrolling blocks
            int size = s.size < 1 ? 1 : s.size;
            int period = 2 * size;
            int off = (int)travel(nowMs, s.speed, period);
            for (int i = 0; i < NUM_PIXELS; i++) {
                int j = (s.dir >= 0) ? (i - off) : (i + off);
                j = ((j % period) + period) % period;
                frame[i] = (j < size) ? s.c1 : s.c2;
            }
            break;
        }
        case FX_ALTERNATE: {                                 // blocks swap colours
            int size = s.size < 1 ? 1 : s.size;
            int swap = (phase(nowMs, s.rate) < 0.5f) ? 0 : 1;
            for (int i = 0; i < NUM_PIXELS; i++) frame[i] = (((i / size) + swap) & 1) ? s.c2 : s.c1;
            break;
        }
        case FX_FLASH:
            fill(phase(nowMs, s.rate) < 0.5f ? s.c1 : s.c2);
            break;
        case FX_BREATHE: {
            float ph = phase(nowMs, s.rate);
            float level = 0.5f - 0.5f * cosf(ph * 6.2831853f);
            fill(mix(s.c2, s.c1, (uint8_t)(level * 255.0f)));
            break;
        }
        case FX_RAINBOW: {
            int size = s.size < 1 ? 1 : s.size;
            float off = travel(nowMs, s.speed, size);
            for (int i = 0; i < NUM_PIXELS; i++) {
                float pos = (s.dir >= 0) ? ((float)i - off) : ((float)i + off);
                int h = (int)(pos * 256.0f / (float)size);
                frame[i] = hueWheel((uint8_t)(h & 255));
            }
            break;
        }
        case FX_WIPE: {                                      // fill, then empty
            float x = travel(nowMs, s.speed, 2 * NUM_PIXELS);
            for (int i = 0; i < NUM_PIXELS; i++) {
                int idx = (s.dir >= 0) ? i : (NUM_PIXELS - 1 - i);
                bool lit = (x < (float)NUM_PIXELS) ? ((float)idx < x) : ((float)idx >= x - (float)NUM_PIXELS);
                frame[i] = lit ? s.c1 : s.c2;
            }
            break;
        }
        case FX_SPARKLE: renderSparkle(nowMs); break;
        case FX_FIRE:    renderFire(nowMs);    break;
        case FX_POLICE: {                                    // halves flash alternately, double pulse
            float ph = phase(nowMs, s.rate);
            bool left = ph < 0.5f;
            float sub = (ph < 0.5f ? ph : ph - 0.5f) * 2.0f;  // 0..1 inside the half period
            bool on = (sub < 0.2f) || (sub > 0.3f && sub < 0.5f);
            int half = NUM_PIXELS / 2;
            for (int i = 0; i < NUM_PIXELS; i++) {
                bool inLeft = i < half;
                frame[i] = (on && inLeft == left) ? (inLeft ? s.c1 : s.c2) : black();
            }
            break;
        }
        default: fill(black()); break;
    }
}

// ============================================================================
// Command parser
// ============================================================================
static Effect effectByName(const char *name) {
    if (strcmp(name, "color") == 0 || strcmp(name, "colour") == 0) return FX_SOLID;
    for (int i = 0; i < FX_COUNT; i++) if (strcmp(name, FX_NAMES[i]) == 0) return (Effect)i;
    return FX_COUNT;
}

static bool isNumber(const char *tok) {
    char *end;
    strtof(tok, &end);
    return end != tok && *end == '\0';
}

static bool applyKeyValue(Scene &s, const char *key, const char *val) {
    float f = strtof(val, nullptr);
    if (strcmp(key, "speed") == 0)       { s.speed = f; if (s.speed < 0) { s.speed = -s.speed; s.dir = -1; } }
    else if (strcmp(key, "rate") == 0)   { s.rate = f < 0.0f ? 0.0f : f; }
    else if (strcmp(key, "size") == 0)   { s.size = (int)f < 1 ? 1 : (int)f; }
    else if (strcmp(key, "kelvin") == 0 || strcmp(key, "k") == 0) { s.kelvin = (int)f; }
    else if (strcmp(key, "level") == 0)  { s.level = (int)f < 0 ? 0 : ((int)f > 255 ? 255 : (int)f); }
    else if (strcmp(key, "period") == 0) { s.period = (int)f < 1 ? 1 : (int)f; }
    else if (strcmp(key, "dir") == 0)    { s.dir = (val[0] == 'r' || val[0] == '-' || val[0] == 'b') ? -1 : 1; }
    else if (strcmp(key, "bri") == 0 || strcmp(key, "brightness") == 0) {
        int v = (int)f; g_brightness = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
    }
    else if (strcmp(key, "c1") == 0 || strcmp(key, "color") == 0) { return parseColor(val, &s.c1); }
    else if (strcmp(key, "c2") == 0 || strcmp(key, "bg") == 0)    { return parseColor(val, &s.c2); }
    else return false;
    return true;
}

// Applies one command line. Writes a reply ("ok ..." / "error ...") into `reply`.
// `fromCycle` = true when the playlist is loading a step (keeps cycle mode).
static bool applyCommand(const char *line, char *reply, size_t replyLen, bool fromCycle) {
    char buf[160];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    for (char *p = buf; *p; p++) *p = (char)tolower((unsigned char)*p);

    char *tok[12];
    int ntok = 0;
    for (char *p = strtok(buf, " \t\r\n"); p && ntok < 12; p = strtok(nullptr, " \t\r\n")) tok[ntok++] = p;
    if (ntok == 0) { snprintf(reply, replyLen, "error empty command"); return false; }

    const char *cmd = tok[0];

    // ---- global / tweak commands
    if (strcmp(cmd, "off") == 0) {
        if (g_scene.fx != FX_OFF) g_saved = g_scene;
        g_cycleActive = false;
        g_scene.fx = FX_OFF;
        snprintf(reply, replyLen, "ok off");
        return true;
    }
    if (strcmp(cmd, "on") == 0) {
        if (g_scene.fx == FX_OFF) g_scene = g_saved;
        snprintf(reply, replyLen, "ok on %s", FX_NAMES[g_scene.fx]);
        return true;
    }
    if (strcmp(cmd, "status") == 0) { snprintf(reply, replyLen, "ok status"); return true; }
    if (strcmp(cmd, "brightness") == 0 || strcmp(cmd, "bri") == 0) {
        if (ntok < 2 || !isNumber(tok[1])) { snprintf(reply, replyLen, "error brightness needs 0-255"); return false; }
        applyKeyValue(g_scene, "bri", tok[1]);
        snprintf(reply, replyLen, "ok brightness %u", g_brightness);
        return true;
    }
    if (strcmp(cmd, "speed") == 0 || strcmp(cmd, "rate") == 0 || strcmp(cmd, "size") == 0 ||
        strcmp(cmd, "dir") == 0 || strcmp(cmd, "kelvin") == 0 || strcmp(cmd, "level") == 0) {
        if (ntok < 2 || !applyKeyValue(g_scene, cmd, tok[1])) { snprintf(reply, replyLen, "error %s needs a value", cmd); return false; }
        snprintf(reply, replyLen, "ok %s %s", cmd, tok[1]);
        return true;
    }

    // ---- effect commands
    Effect fx = effectByName(cmd);
    if (fx == FX_COUNT) { snprintf(reply, replyLen, "error unknown command '%s'", cmd); return false; }

    if (fx == FX_CYCLE) {
        g_cyclePeriod = 15;
        if (ntok >= 2 && isNumber(tok[1])) { g_cyclePeriod = atoi(tok[1]); if (g_cyclePeriod < 2) g_cyclePeriod = 2; }
        g_cycleActive = true;
        g_cycleIndex = -1;                                 // first tick loads step 0
        g_cycleStart = 0;
        snprintf(reply, replyLen, "ok cycle every %d s", g_cyclePeriod);
        return true;
    }

    Scene s;
    defaultsFor(fx, s);
    int colors = 0, numbers = 0;
    for (int i = 1; i < ntok; i++) {
        char *eq = strchr(tok[i], '=');
        if (eq) {
            *eq = '\0';
            if (!applyKeyValue(s, tok[i], eq + 1)) { snprintf(reply, replyLen, "error bad option '%s'", tok[i]); return false; }
            continue;
        }
        Pixel c;
        if (parseColor(tok[i], &c)) {
            if (colors == 0) s.c1 = c; else s.c2 = c;
            colors++;
            continue;
        }
        if (isNumber(tok[i])) {
            float v = strtof(tok[i], nullptr);
            numbers++;
            switch (fx) {
                case FX_WHITE:
                    if (numbers == 1) s.kelvin = (int)v; else s.level = (int)v < 0 ? 0 : ((int)v > 255 ? 255 : (int)v);
                    break;
                case FX_FLASH: case FX_BREATHE: case FX_ALTERNATE: case FX_SPARKLE: case FX_FIRE: case FX_POLICE:
                    s.rate = v < 0.0f ? 0.0f : v;
                    break;
                default:
                    s.speed = fabsf(v);
                    if (v < 0.0f) s.dir = -1;
                    break;
            }
            continue;
        }
        snprintf(reply, replyLen, "error don't understand '%s'", tok[i]);
        return false;
    }
    if (!fromCycle) g_cycleActive = false;
    if (fx == FX_FIRE)    { memset(heat, 0, sizeof(heat)); g_fxTick = millis(); }
    if (fx == FX_SPARKLE) { memset(spark, 0, sizeof(spark)); g_fxTick = millis(); }
    g_scene = s;
    snprintf(reply, replyLen, "ok %s", FX_NAMES[fx]);
    return true;
}

static void cycleTick(uint32_t nowMs) {
    if (!g_cycleActive) return;
    if (g_cycleIndex >= 0 && (uint32_t)(nowMs - g_cycleStart) < (uint32_t)g_cyclePeriod * 1000UL) return;
    const int n = sizeof(CYCLE_PLAYLIST) / sizeof(CYCLE_PLAYLIST[0]);
    g_cycleIndex = (g_cycleIndex + 1) % n;
    g_cycleStart = nowMs;
    char reply[64];
    applyCommand(CYCLE_PLAYLIST[g_cycleIndex], reply, sizeof(reply), true);
}

// ============================================================================
// Minimal MQTT 3.1.1 client on the ClearCore EthernetTcpClient (QoS 0)
// ============================================================================
#define MQTT_RX_BUF 512
#define MQTT_TX_BUF 512

typedef void (*MqttMessageHandler)(const char *topic, const char *payload);

class MqttLink {
public:
    enum State { DOWN, WAIT_CONNACK, CONNECTED };

    void begin(IpAddress broker, uint16_t port, MqttMessageHandler handler,
               const char *willTopic, const char *willMsg) {
        m_ip = broker; m_port = port; m_handler = handler;
        m_willTopic = willTopic; m_willMsg = willMsg;
        m_tcp.ConnectionTimeout(500);              // Connect() blocks at most this long
        m_state = DOWN; m_lastAttempt = 0; m_haveAttempted = false;
    }
    bool connected() const { return m_state == CONNECTED; }
    uint32_t reconnects() const { return m_reconnects; }
    const char *lastError() const { return m_err; }

    // Call every loop. Pumps the stack, parses incoming packets, keeps alive.
    void poll(uint32_t now, bool netReady) {
        if (m_state != DOWN && !m_tcp.Connected()) drop("tcp closed");
        if (m_state == DOWN) {
            if (!netReady) return;
            if (m_haveAttempted && (uint32_t)(now - m_lastAttempt) < MQTT_RECONNECT_MS) return;
            m_lastAttempt = now; m_haveAttempted = true;
            tryConnect(now);
            return;
        }
        readPackets(now);
        if (m_state == WAIT_CONNACK && (uint32_t)(now - m_connectSent) > 5000) { drop("no CONNACK"); return; }
        if (m_state == CONNECTED) {
            if (m_pingOut && (uint32_t)(now - m_pingSent) > (uint32_t)MQTT_KEEPALIVE_S * 1000UL) { drop("ping timeout"); return; }
            if (!m_pingOut && (uint32_t)(now - m_lastTx) >= (uint32_t)MQTT_KEEPALIVE_S * 500UL) {
                static const uint8_t ping[2] = { 0xC0, 0x00 };
                if (sendRaw(ping, 2)) { m_pingOut = true; m_pingSent = now; }
            }
        }
    }

    bool publish(const char *topic, const char *payload, bool retain) {
        if (m_state != CONNECTED) return false;
        size_t tl = strlen(topic), pl = strlen(payload);
        if (2 + tl + pl > MQTT_TX_BUF - 5) return false;
        uint8_t buf[MQTT_TX_BUF];
        size_t n = 0;
        buf[n++] = (uint8_t)(0x30 | (retain ? 0x01 : 0x00));
        n += putLength(buf + n, 2 + tl + pl);
        n += putString(buf + n, topic, tl);
        memcpy(buf + n, payload, pl); n += pl;
        return sendRaw(buf, n);
    }

    bool subscribe(const char *topic) {
        if (m_state != CONNECTED) return false;
        size_t tl = strlen(topic);
        uint8_t buf[MQTT_TX_BUF];
        size_t n = 0;
        buf[n++] = 0x82;
        n += putLength(buf + n, 2 + 2 + tl + 1);
        if (++m_packetId == 0) m_packetId = 1;
        buf[n++] = (uint8_t)(m_packetId >> 8); buf[n++] = (uint8_t)m_packetId;
        n += putString(buf + n, topic, tl);
        buf[n++] = 0x00;                                     // QoS 0
        return sendRaw(buf, n);
    }

    // Called once right after CONNACK: subscriptions and "online" announcements go here.
    void (*onConnected)() = nullptr;

private:
    EthernetTcpClient m_tcp;
    IpAddress  m_ip;
    uint16_t   m_port = 1883;
    MqttMessageHandler m_handler = nullptr;
    const char *m_willTopic = nullptr, *m_willMsg = nullptr;
    State      m_state = DOWN;
    uint8_t    m_rx[MQTT_RX_BUF];
    size_t     m_rxLen = 0;
    size_t     m_skip = 0;                   // bytes of an oversized packet still to discard
    uint16_t   m_packetId = 0;
    uint32_t   m_lastTx = 0, m_connectSent = 0, m_pingSent = 0, m_lastAttempt = 0;
    bool       m_pingOut = false, m_haveAttempted = false;
    uint32_t   m_reconnects = 0;
    const char *m_err = "";

    static size_t putLength(uint8_t *dst, size_t len) {    // MQTT variable-length integer
        size_t n = 0;
        do { uint8_t d = (uint8_t)(len % 128); len /= 128; if (len) d |= 0x80; dst[n++] = d; } while (len);
        return n;
    }
    static size_t putString(uint8_t *dst, const char *s, size_t len) {
        dst[0] = (uint8_t)(len >> 8); dst[1] = (uint8_t)len; memcpy(dst + 2, s, len);
        return 2 + len;
    }
    bool sendRaw(const uint8_t *buf, size_t n) {
        size_t sent = 0;
        uint32_t t0 = millis();
        while (sent < n) {
            uint32_t w = m_tcp.Send(buf + sent, (uint32_t)(n - sent));
            if (w == 0) {
                if (!m_tcp.Connected() || (uint32_t)(millis() - t0) > 100) { drop("send failed"); return false; }
                EthernetMgr.Refresh();
                continue;
            }
            sent += w;
        }
        m_lastTx = millis();
        return true;
    }
    void drop(const char *why) {
        m_err = why;
        m_tcp.Close();
        if (m_state != DOWN) m_reconnects++;
        m_state = DOWN;
        m_rxLen = 0; m_skip = 0; m_pingOut = false;
    }
    void tryConnect(uint32_t now) {
        if (!m_tcp.Connect(m_ip, m_port)) { m_err = "tcp connect failed"; return; }
        m_rxLen = 0; m_skip = 0; m_pingOut = false;
        // CONNECT
        const char *id = MQTT_CLIENT_ID; size_t idl = strlen(id);
        const char *user = MQTT_USER;    size_t ul = strlen(user);
        const char *pass = MQTT_PASS;    size_t pl = strlen(pass);
        size_t wtl = m_willTopic ? strlen(m_willTopic) : 0, wml = m_willMsg ? strlen(m_willMsg) : 0;
        uint8_t flags = 0x02;                                // clean session
        if (wtl) flags |= 0x04 | 0x20;                       // will, will retain, QoS 0
        if (ul)  flags |= 0x80;
        if (pl)  flags |= 0x40;
        size_t rem = 10 + 2 + idl + (wtl ? 4 + wtl + wml : 0) + (ul ? 2 + ul : 0) + (pl ? 2 + pl : 0);
        uint8_t buf[MQTT_TX_BUF];
        size_t n = 0;
        buf[n++] = 0x10;
        n += putLength(buf + n, rem);
        n += putString(buf + n, "MQTT", 4);
        buf[n++] = 0x04;                                     // protocol level 3.1.1
        buf[n++] = flags;
        buf[n++] = (uint8_t)(MQTT_KEEPALIVE_S >> 8); buf[n++] = (uint8_t)MQTT_KEEPALIVE_S;
        n += putString(buf + n, id, idl);
        if (wtl) { n += putString(buf + n, m_willTopic, wtl); n += putString(buf + n, m_willMsg, wml); }
        if (ul)  n += putString(buf + n, user, ul);
        if (pl)  n += putString(buf + n, pass, pl);
        m_state = WAIT_CONNACK;                              // sendRaw needs a non-DOWN state check? no: only drop() cares
        if (!sendRaw(buf, n)) return;
        m_connectSent = now;
    }
    void readPackets(uint32_t now) {
        int16_t avail;
        while ((avail = m_tcp.BytesAvailable()) > 0) {        // BytesAvailable() also pumps the stack
            if (m_skip) {
                uint8_t tmp[64];
                int16_t n = m_tcp.Read(tmp, m_skip < sizeof(tmp) ? (uint32_t)m_skip : (uint32_t)sizeof(tmp));
                if (n <= 0) break;
                m_skip -= (size_t)n;
                continue;
            }
            int16_t n = m_tcp.Read(m_rx + m_rxLen, (uint32_t)(MQTT_RX_BUF - m_rxLen));
            if (n <= 0) break;
            m_rxLen += (size_t)n;
            processBuffer(now);
            if (m_state == DOWN) return;
        }
    }
    void processBuffer(uint32_t now) {
        for (;;) {
            if (m_rxLen < 2) return;
            size_t rem = 0, mult = 1, i = 1;
            bool complete = false;
            while (i < m_rxLen && i < 5) {
                rem += (size_t)(m_rx[i] & 0x7F) * mult;
                mult *= 128;
                if (!(m_rx[i] & 0x80)) { complete = true; i++; break; }
                i++;
            }
            if (!complete) { if (i >= 5) { drop("bad length"); } return; }
            size_t total = i + rem;
            if (total > MQTT_RX_BUF) {                        // too big for us: discard it entirely
                m_skip = total - m_rxLen;
                m_rxLen = 0;
                return;
            }
            if (m_rxLen < total) return;                      // wait for the rest
            handlePacket(m_rx[0], m_rx + i, rem, now);
            size_t rest = (m_rxLen > total) ? (m_rxLen - total) : 0;
            if (rest) memmove(m_rx, m_rx + total, rest);
            m_rxLen = rest;
            if (m_state == DOWN) return;
        }
    }
    void handlePacket(uint8_t hdr, const uint8_t *p, size_t len, uint32_t now) {
        (void)now;
        switch (hdr & 0xF0) {
            case 0x20:                                        // CONNACK
                if (len >= 2 && p[1] == 0) {
                    m_state = CONNECTED;
                    m_err = "";
                    if (onConnected) onConnected();
                } else {
                    drop("CONNACK refused");
                }
                break;
            case 0x30: {                                      // PUBLISH
                if (len < 2) break;
                size_t tl = ((size_t)p[0] << 8) | p[1];
                size_t pos = 2 + tl;
                if (pos > len) break;
                uint8_t qos = (hdr >> 1) & 0x03;
                if (qos > 0) {
                    if (pos + 2 > len) break;
                    uint8_t ack[4] = { 0x40, 0x02, p[pos], p[pos + 1] };   // PUBACK (QoS 1)
                    pos += 2;
                    if (qos == 1) sendRaw(ack, 4);
                }
                char topic[96], payload[160];
                if (tl >= sizeof(topic)) break;
                memcpy(topic, p + 2, tl); topic[tl] = '\0';
                size_t pl = len - pos;
                if (pl >= sizeof(payload)) pl = sizeof(payload) - 1;
                memcpy(payload, p + pos, pl); payload[pl] = '\0';
                if (m_handler) m_handler(topic, payload);
                break;
            }
            case 0xD0:                                        // PINGRESP
                m_pingOut = false;
                break;
            case 0x90:                                        // SUBACK
                if (len >= 3 && p[2] == 0x80) m_err = "subscribe refused";
                break;
            default:
                break;
        }
    }
};

// ============================================================================
// Network glue
// ============================================================================
static MqttLink  g_mqtt;
static bool      g_haveIp      = false;
static uint32_t  g_lastDhcpTry = 0;
static char      g_topicCmd[64], g_topicState[64], g_topicStatus[64], g_topicReply[64];
static uint32_t  g_lastStatePub = 0;
static bool      g_stateDirty   = true;

static void logLine(const char *s) {
#if REPORT_STATS
    Serial.println(s);
#else
    (void)s;
#endif
}

static void publishState() {
    char c1[12], c2[12], json[256];
    colorToHex(g_scene.c1, c1);
    colorToHex(g_scene.c2, c2);
    int watts10 = (int)(g_estWatts * 10.0f + 0.5f);
    int speed10 = (int)(g_scene.speed * 10.0f + 0.5f);
    int rate100 = (int)(g_scene.rate * 100.0f + 0.5f);
    snprintf(json, sizeof(json),
             "{\"effect\":\"%s\",\"cycle\":%s,\"c1\":\"%s\",\"c2\":\"%s\",\"speed\":%d.%d,\"rate\":%d.%02d,"
             "\"size\":%d,\"dir\":%d,\"kelvin\":%d,\"brightness\":%u,\"fps\":%lu,\"watts\":%d.%d,"
             "\"limiter\":%d,\"segments\":%d,\"ip\":\"%s\"}",
             FX_NAMES[g_scene.fx], g_cycleActive ? "true" : "false", c1, c2,
             speed10 / 10, speed10 % 10, rate100 / 100, rate100 % 100,
             g_scene.size, g_scene.dir, g_scene.kelvin, g_brightness, (unsigned long)g_fps,
             watts10 / 10, watts10 % 10, g_powerScale == 255 ? 100 : (g_powerScale * 100) / 255,
             NUM_PIXELS, EthernetMgr.LocalIp().StringValue());
    g_mqtt.publish(g_topicState, json, true);
    g_stateDirty = false;
    g_lastStatePub = millis();
}

static void onMqttConnected() {
    g_mqtt.subscribe(g_topicCmd);
    g_mqtt.publish(g_topicStatus, "online", true);
    logLine("mqtt: connected");
    g_stateDirty = true;
}

static void onMqttMessage(const char *topic, const char *payload) {
    if (strcmp(topic, g_topicCmd) != 0) return;
    char reply[96];
    applyCommand(payload, reply, sizeof(reply), false);
    g_mqtt.publish(g_topicReply, reply, false);
    g_stateDirty = true;
#if REPORT_STATS
    Serial.print("cmd: "); Serial.print(payload); Serial.print(" -> "); Serial.println(reply);
#endif
}

static void networkSetup() {
    EthernetMgr.Setup();
#if USE_DHCP
    if (EthernetMgr.PhyLinkActive()) {
        g_haveIp = EthernetMgr.DhcpBegin();                  // blocks up to ~7.5 s
        g_lastDhcpTry = millis();
    }
#else
    EthernetMgr.LocalIp(IpAddress(STATIC_IP));
    EthernetMgr.NetmaskIp(IpAddress(STATIC_NETMASK));
    EthernetMgr.GatewayIp(IpAddress(STATIC_GATEWAY));
    g_haveIp = true;
#endif
    snprintf(g_topicCmd,    sizeof(g_topicCmd),    "%s/cmd",    MQTT_PREFIX);
    snprintf(g_topicState,  sizeof(g_topicState),  "%s/state",  MQTT_PREFIX);
    snprintf(g_topicStatus, sizeof(g_topicStatus), "%s/status", MQTT_PREFIX);
    snprintf(g_topicReply,  sizeof(g_topicReply),  "%s/reply",  MQTT_PREFIX);
    g_mqtt.onConnected = onMqttConnected;
    g_mqtt.begin(IpAddress(MQTT_BROKER_IP), MQTT_PORT, onMqttMessage, g_topicStatus, "offline");
}

static void networkPoll(uint32_t now) {
    bool link = EthernetMgr.PhyLinkActive();
#if USE_DHCP
    if (!g_haveIp && link && (uint32_t)(now - g_lastDhcpTry) > 60000UL) {
        g_haveIp = EthernetMgr.DhcpBegin();
        g_lastDhcpTry = now;
    }
#endif
    if (!g_haveIp) { EthernetMgr.Refresh(); return; }
    if (!g_mqtt.connected()) EthernetMgr.Refresh();          // keep DHCP/ARP alive between attempts
    g_mqtt.poll(now, link);
    if (g_mqtt.connected() && (g_stateDirty || (uint32_t)(now - g_lastStatePub) >= STATE_PERIOD_MS)) {
        publishState();
    }
}

// Blink the last segment while something is missing: red = no link,
// orange = no IP, yellow = no broker.
static void netStatusOverlay(uint32_t now) {
#if NET_STATUS_PIXEL
    if (g_mqtt.connected()) return;
    bool link = EthernetMgr.PhyLinkActive();
    Pixel c = !link ? rgb(255, 0, 0) : (!g_haveIp ? rgb(255, 60, 0) : rgb(255, 200, 0));
    frame[NUM_PIXELS - 1] = ((now / 500) & 1) ? c : black();
#else
    (void)now;
#endif
}

// ============================================================================
void setup() {
#if REPORT_STATS
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 1500) { }
#endif
    buildLuts();
    memset(frame, 0, sizeof(frame));
    portInit(ports[0], ConnectorCOM0, DMA_SERCOM7_SPI_RX, 0, PORT_PIXELS, spiBuf[0][0], spiBuf[0][1]);
#if USE_TWO_PORTS
    portInit(ports[1], ConnectorCOM1, DMA_SERCOM0_SPI_RX, PORT_PIXELS, NUM_PIXELS - PORT_PIXELS, spiBuf[1][0], spiBuf[1][1]);
#endif
    show(); show();                                           // black on both DMA buffers

    char reply[64];
    defaultsFor(FX_OFF, g_saved);
    applyCommand(BOOT_COMMAND, reply, sizeof(reply), false);
    networkSetup();

#if REPORT_STATS
    Serial.print("segments ");  Serial.print(NUM_PIXELS);
    Serial.print("  wire ms "); Serial.print(FRAME_WIRE_US / 1000.0f, 2);
    Serial.print("  max fps "); Serial.println(1.0e6f / FRAME_WIRE_US, 1);
    Serial.print("ip ");        Serial.println(g_haveIp ? EthernetMgr.LocalIp().StringValue() : "(none)");
    Serial.print("broker ");    Serial.print(MQTT_BROKER_IP); Serial.print(":"); Serial.println(MQTT_PORT);
    Serial.print("topics ");    Serial.print(g_topicCmd); Serial.print(" "); Serial.println(g_topicState);
#endif
}

void loop() {
    static uint32_t lastSecond = 0;
    uint32_t now = millis();

    networkPoll(now);            // MQTT in, commands applied, state out
    cycleTick(now);
    renderScene(now);
    netStatusOverlay(now);
    show();                      // waits only for the previous frame to leave the wire

    if ((uint32_t)(now - lastSecond) >= 1000) {
        g_fps = g_frames; g_frames = 0; lastSecond = now;
#if REPORT_STATS
        Serial.print("fps "); Serial.print(g_fps);
        Serial.print("  est W "); Serial.print(g_estWatts, 1);
        Serial.print("  limiter "); Serial.print(g_powerScale == 255 ? 100 : (g_powerScale * 100) / 255);
        Serial.print("%  mqtt "); Serial.print(g_mqtt.connected() ? "up" : g_mqtt.lastError());
        Serial.print("  fx "); Serial.println(FX_NAMES[g_scene.fx]);
#endif
    }
}
