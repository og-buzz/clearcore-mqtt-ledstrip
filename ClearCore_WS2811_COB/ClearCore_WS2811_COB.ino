/*
 * ClearCore_WS2811_COB.ino
 *
 * Teknic ClearCore -> WS2811 COB LED strip (7.5 m), 250 W / 24 V supply shared
 * with the ClearCore. Supports plain RGB WS2811 strips (one IC per segment)
 * and the BTF-LIGHTING "Dual IC" RGBCCT FCOB strip (two WS2811 ICs per
 * segment: one drives R,G,B, the other drives cool white, warm white, unused).
 *
 * HOW THE SIGNAL IS MADE
 *   The ClearCore has no general-purpose logic-level GPIO fast enough for the
 *   WS2811 waveform (the I/O-x connectors are 24 V protected FET outputs), but
 *   each COM port can run as a 5 V SPI master and has DMA channels for SPI.
 *   This sketch synthesises the WS2811 waveform on the SPI MOSI line:
 *   every WS2811 data bit becomes 3 (or 6) SPI bits, and the whole frame is
 *   pushed out of the SERCOM by DMA (SerialBase::SpiTransferDataAsync).
 *   The CPU renders the NEXT frame while the current one is on the wire.
 *
 *   SPI clock comes from GCLK7 = 10 MHz in the ClearCore library, so
 *   Speed(2500000) gives exactly 2.5 MHz and Speed(5000000) exactly 5 MHz.
 *
 *   TIMING_PROFILE 0  (default)  3 SPI bits per data bit @ 2.5 MHz
 *       '0' = 400 ns high, 800 ns low      '1' = 800 ns high, 400 ns low
 *       (the same timing Adafruit NeoPixel / NeoPixel_ZeroDMA use for 800 kHz strips)
 *   TIMING_PROFILE 1             6 SPI bits per data bit @ 5 MHz (800 kHz strips)
 *       '0' = 200 ns high, 1000 ns low     '1' = 600 ns high, 600 ns low
 *       (WS2811 datasheet nominal: T0H 0.25 us, T1H 0.6 us, T0L 1.0 us, T1L 0.65 us, +-150 ns)
 *                                6 SPI bits per data bit @ 2.5 MHz (400 kHz strips)
 *       '0' = 400 ns high, 2000 ns low     '1' = 1200 ns high, 1200 ns low
 *
 * WIRING (COM-0, and COM-1 if USE_TWO_PORTS)
 *   COM port in SPI mode: MOSI ("Data In" on the device side) -> strip DIN,
 *   GND -> strip GND (run it next to the data wire). Take the RJ-45 pin
 *   numbers from the ClearCore Hardware Manual, page "COM-0, 1 configured
 *   for SPI devices". The port's 5 V pin is NOT needed (24 V strip).
 *   Keep the data cable short (well under 2 m); the pulses are 200-400 ns wide.
 *   SCK and SS also toggle on the connector; leave them unconnected.
 *
 *   FIRST POWER-UP CHECK: Teknic rates the COM ports at 115.2 kBaud, "up to
 *   2 MBaud achievable depending on cable length, slave transceiver circuit
 *   and grounding" (Hardware Manual, Appendix D). This sketch runs the MOSI
 *   line at 2.5 Mbit/s (5 Mbit/s with TIMING_PROFILE 1). Put a scope on the
 *   strip's DIN once: you should see clean 400 ns / 800 ns high pulses
 *   reaching ~5 V. If the 400 ns pulses are rounded off, shorten the cable;
 *   if they are still bad, the port cannot do this and the strip needs a
 *   separate pixel driver.
 *
 * POWER
 *   Strip and ClearCore both on the 24 V / 250 W supply. The limiter below
 *   estimates each frame's draw from the pixel data and scales the frame down
 *   so the strip never asks for more than LED_BUDGET_WATTS. Feed 24 V into the
 *   strip at BOTH ends (and the middle for 7.5 m); the strip's copper cannot
 *   carry ~7 A from one end.
 *
 * Verified against: ClearCore library (SerialBase.cpp: __SERCOM_SPI_CLOCK
 * 10 MHz, SpiTransferDataAsync uses DMAC; COM-0 = SERCOM7, COM-1 = SERCOM0,
 * both with SPI DMA channels) and ClearCore Hardware Manual rev 1.14
 * (COM ports: 5 V SPI/UART, 130 ohm series on outputs; ClearCore draws
 * 300 mA @ 24 V; input 20-28 VDC).
 */

#include "ClearCore.h"
#include <string.h>
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
#define PIXELS_PER_M        14      // addressable SEGMENTS per metre. BTF RGBCCT: 840 LEDs/m / 60 LEDs per
                                    // segment = 14 segments/m (71 mm cut, 28 ICs/m). Not the LED count.
#define STRIP_W_PER_M       23.0f   // watts per metre with every channel full. BTF quotes "Max 18 W/m"
                                    // (btf-lighting.com) and 23 W/m (fcob.com); the larger is used.
#define STRIP_IDLE_W_PER_M  0.0f    // standby draw per metre if the spec lists one, else 0
#define STRIP_KHZ           800     // 800 for normal WS2811 strips; 400 only for low-speed-mode parts
#define COLOR_ORDER         ORDER_RGB   // byte order on the RGB IC (WS2811 datasheet order is RGB)
#define CCT_ORDER           CCT_CW_WW   // byte order on the white IC: [cw][ww][unused] or [ww][cw][unused].
                                        // Not documented by BTF: run DEMO_MODE 1 once to find out.

// ============================================================================
// 2. POWER  -- 250 W supply shared with the ClearCore
// ============================================================================
#define PSU_WATTS           250.0f
#define PSU_HEADROOM        0.80f   // plan for at most 80 % of the supply's nameplate
#define CLEARCORE_WATTS     7.2f    // ClearCore: 300 mA @ 24 V (Hardware Manual, Appendix D)
#define LED_BUDGET_WATTS    (PSU_WATTS * PSU_HEADROOM - CLEARCORE_WATTS)   // 192.8 W with the values above

// ============================================================================
// 3. OUTPUT
// ============================================================================
#define USE_TWO_PORTS       0   // 0: COM-0 drives the whole strip.
                                // 1: COM-0 drives segments [0, N/2), COM-1 drives [N/2, N): ~2x frame rate.
                                //    Cut the strip's DATA trace between the halves (power stays common)
                                //    and feed COM-1 MOSI into the second half's DIN.
#define TIMING_PROFILE      0   // see header: 0 = NeoPixel-style 400/800 ns, 1 = WS2811-datasheet 200/600 ns
#define BRIGHTNESS          255 // global 0-255 applied before the power limiter
#define GAMMA_CORRECT       1   // 1 = apply gamma 2.2 to the frame buffer
#define LATCH_US            300 // WS2811 latches after >= 50 us low; 300 us also covers newer variants
#define REPORT_STATS        1   // print fps / power estimate once a second on the USB serial port
#define DEMO_MODE           0   // 0: rainbow on RGB, whites off.
                                // 1: CCT order check: even segments cool white, odd segments warm white.
                                //    If the even segments look WARM, flip CCT_ORDER.

// ============================================================================
// Derived constants (no need to edit)
// ============================================================================
#if STRIP_KHZ == 400
#undef  TIMING_PROFILE
#define TIMING_PROFILE 1        // only the 6-bit pattern gives in-spec 400 kHz pulses
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

static const int      ICS_PER_PIXEL   = (int)PIXEL_TYPE;                    // WS2811 chips per segment
static const int      WIRE_BYTES_PER_PIXEL = 3 * ICS_PER_PIXEL;             // data bytes per segment on the wire
static const int      ACTIVE_CHANNELS = (ICS_PER_PIXEL == 2) ? 5 : 3;       // channels that draw power
static const int      BYTES_PER_COLOR = SPI_BITS_PER_DATA_BIT;              // SPI bytes per data byte
static const int      BYTES_PER_PIXEL = WIRE_BYTES_PER_PIXEL * BYTES_PER_COLOR;
static const int      NUM_PIXELS      = (int)(STRIP_LENGTH_M * PIXELS_PER_M + 0.5f);
static const int      NUM_PORTS       = USE_TWO_PORTS ? 2 : 1;
static const int      PORT_PIXELS     = (NUM_PIXELS + NUM_PORTS - 1) / NUM_PORTS;
static const uint32_t LATCH_BYTES     = ((uint32_t)LATCH_US * (SPI_HZ / 1000UL) + 7999UL) / 8000UL;
static const uint32_t PORT_BYTES      = (uint32_t)PORT_PIXELS * BYTES_PER_PIXEL + LATCH_BYTES;
static const float    WATTS_PER_COUNT = STRIP_W_PER_M / PIXELS_PER_M / ACTIVE_CHANNELS / 255.0f;  // one channel, one count
static const float    STRIP_IDLE_W    = STRIP_IDLE_W_PER_M * STRIP_LENGTH_M;
static const float    FRAME_WIRE_US   = (float)PORT_BYTES * 8.0f * 1.0e6f / (float)SPI_HZ;

static_assert(NUM_PIXELS > 0, "NUM_PIXELS must be positive");
static_assert(PORT_BYTES <= 65535UL, "one DMA block is limited to 65535 bytes; use USE_TWO_PORTS or fewer pixels");
static_assert(BRIGHTNESS >= 0 && BRIGHTNESS <= 255, "BRIGHTNESS is 0-255");
static_assert(LED_BUDGET_WATTS > 0.0f, "no power left for the LEDs");

// ============================================================================
// Frame buffer: what you draw into.  frame[i] = {r, g, b, cw, ww}, 0-255.
// cw / ww are ignored on PIXEL_RGB strips.
// ============================================================================
struct Pixel { uint8_t r, g, b, cw, ww; };
static Pixel frame[NUM_PIXELS];

static inline void setPixel(int i, uint8_t r, uint8_t g, uint8_t b, uint8_t cw = 0, uint8_t ww = 0) {
    if (i >= 0 && i < NUM_PIXELS) { frame[i].r = r; frame[i].g = g; frame[i].b = b; frame[i].cw = cw; frame[i].ww = ww; }
}

// ============================================================================
// Output ports
// ============================================================================
struct LedPort {
    SerialDriver *com;        // ConnectorCOM0 / ConnectorCOM1
    DmaChannels   rxChannel;  // the SPI RX DMA channel; its ENABLE bit clears when the transfer is done
    int           firstPixel;
    int           numPixels;
    uint32_t      bytes;      // numPixels * BYTES_PER_PIXEL + LATCH_BYTES
    uint8_t      *buf[2];     // double buffer
};

// DMA buffers. Each frame ends with LATCH_BYTES of 0x00, so MOSI rests low
// between frames and the strip latches on its own.
static uint8_t spiBuf[NUM_PORTS][2][PORT_BYTES] __attribute__((aligned(4)));
static LedPort ports[NUM_PORTS];

// Look-up tables
static uint8_t  lut[256][BYTES_PER_COLOR];      // data byte -> SPI bit pattern (MSB first)
static uint8_t  gammaLut[256];
static uint8_t  wireFrame[NUM_PIXELS * WIRE_BYTES_PER_PIXEL];   // gamma + brightness applied, in wire order

// Stats
static float    g_estWatts   = 0.0f;   // estimated strip draw of the last frame, after limiting
static uint8_t  g_powerScale = 255;    // 255 = limiter not active
static uint32_t g_frames     = 0;
static uint32_t g_lastReport = 0;

// ----------------------------------------------------------------------------
static inline uint8_t scale8(uint8_t v, uint8_t s) {
    // v * s / 255, FastLED-style: exact for s == 255, max error 1 count otherwise
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
        gammaLut[v] = GAMMA_CORRECT ? (uint8_t)(powf((float)v / 255.0f, 2.2f) * 255.0f + 0.5f)
                                    : (uint8_t)v;
    }
}

static void portInit(LedPort &p, SerialDriver &com, DmaChannels rxChannel,
                     int firstPixel, int numPixels, uint8_t *buf0, uint8_t *buf1) {
    p.com        = &com;
    p.rxChannel  = rxChannel;
    p.firstPixel = firstPixel;
    p.numPixels  = numPixels;
    p.bytes      = (uint32_t)numPixels * BYTES_PER_PIXEL + LATCH_BYTES;
    p.buf[0]     = buf0;
    p.buf[1]     = buf1;
    memset(buf0, 0, PORT_BYTES);
    memset(buf1, 0, PORT_BYTES);

    // Same sequence as Teknic's SpiComPortInitialization example.
    com.Mode(Connector::SPI);                 // 5 V SPI master on the RJ-45
    com.Speed(SPI_HZ);                        // 10 MHz GCLK7 / (2*(BAUD+1)) -> exact 2.5 or 5 MHz
    com.DataOrder(SerialDriver::COM_MSB_FIRST);
    com.SpiClock(SerialDriver::SCK_LOW, SerialDriver::LEAD_CHANGE);
    com.PortOpen();
    com.SpiSsMode(SerialDriver::LINE_OFF);    // SS parked; the strip does not use it

    // Hold the line low for a latch period so the strip starts from a clean reset.
    com.SpiTransferData(buf0, nullptr, LATCH_BYTES);
}

static inline bool portBusy(const LedPort &p) {
    return DmaManager::Channel(p.rxChannel)->CHCTRLA.bit.ENABLE != 0;
}

static inline void portWait(LedPort &p) {
    p.com->SpiAsyncWaitComplete();
}

static inline void portStart(LedPort &p, int which) {
    if (!p.com->SpiTransferDataAsync(p.buf[which], nullptr, (int32_t)p.bytes)) {
        // No DMA available on this port (should not happen on COM-0/COM-1): send blocking.
        p.com->SpiTransferData(p.buf[which], nullptr, (int32_t)p.bytes);
    }
}

static inline void encodeByte(uint8_t *dst, uint8_t v) {
    memcpy(dst, lut[v], BYTES_PER_COLOR);
}

// Convert frame[] -> wireFrame[] (gamma, brightness, byte order) and return
// the sum of all channel counts, the quantity the power estimate is based on.
static uint32_t prepareFrame() {
    static const uint8_t idx[6][3] = {
        {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}
    };
    const uint8_t *o = idx[COLOR_ORDER];
    uint32_t sum = 0;
    uint8_t *w = wireFrame;
    for (int i = 0; i < NUM_PIXELS; i++) {
        const Pixel &px = frame[i];
        uint8_t c[3] = { px.r, px.g, px.b };
        uint8_t v0 = scale8(gammaLut[c[o[0]]], BRIGHTNESS);
        uint8_t v1 = scale8(gammaLut[c[o[1]]], BRIGHTNESS);
        uint8_t v2 = scale8(gammaLut[c[o[2]]], BRIGHTNESS);
        sum += (uint32_t)v0 + v1 + v2;
        *w++ = v0; *w++ = v1; *w++ = v2;
        if (ICS_PER_PIXEL == 2) {
            // Second WS2811 on the segment: two whites, third output unused.
            uint8_t cw = scale8(gammaLut[px.cw], BRIGHTNESS);
            uint8_t ww = scale8(gammaLut[px.ww], BRIGHTNESS);
            sum += (uint32_t)cw + ww;
            if (CCT_ORDER == CCT_CW_WW) { *w++ = cw; *w++ = ww; }
            else                        { *w++ = ww; *w++ = cw; }
            *w++ = 0;
        }
    }
    return sum;
}

// Power limiter: scale the whole frame so the estimated strip draw stays
// inside LED_BUDGET_WATTS. Global scaling keeps the colour balance.
static uint8_t powerScaleFor(uint32_t sum) {
    float est = (float)sum * WATTS_PER_COUNT + STRIP_IDLE_W;
    float budget = LED_BUDGET_WATTS;
    if (est <= budget || est <= STRIP_IDLE_W) {
        g_estWatts = est;
        return 255;
    }
    float s = (budget - STRIP_IDLE_W) / (est - STRIP_IDLE_W);
    if (s < 0.0f) s = 0.0f;
    uint8_t s8 = (uint8_t)(s * 255.0f);          // truncate: never exceed the budget
    g_estWatts = (float)sum * ((float)(s8 + 1) / 256.0f) * WATTS_PER_COUNT + STRIP_IDLE_W;
    return s8;
}

// Encode wireFrame[] into the DMA buffers (buffer index `which`).
static void encodeFrame(int which, uint8_t powerScale) {
    for (int pi = 0; pi < NUM_PORTS; pi++) {
        LedPort &p = ports[pi];
        uint8_t *dst = p.buf[which];
        const uint8_t *src = &wireFrame[p.firstPixel * WIRE_BYTES_PER_PIXEL];
        const int n = p.numPixels * WIRE_BYTES_PER_PIXEL;
        if (powerScale == 255) {
            for (int k = 0; k < n; k++) {
                encodeByte(dst, src[k]);
                dst += BYTES_PER_COLOR;
            }
        } else {
            for (int k = 0; k < n; k++) {
                encodeByte(dst, scale8(src[k], powerScale));
                dst += BYTES_PER_COLOR;
            }
        }
        // Latch bytes after the pixel data were zeroed at init and are never written.
    }
}

// Push frame[] to the strip. Returns as soon as the DMA transfer has STARTED.
// Call rate is bounded by the wire time (28.8 us per WS2811 IC + LATCH_US).
static void show() {
    static int which = 0;
    uint32_t sum = prepareFrame();
    g_powerScale = powerScaleFor(sum);
    encodeFrame(which, g_powerScale);            // fills the buffer the DMA is NOT reading
    for (int pi = 0; pi < NUM_PORTS; pi++) portWait(ports[pi]);   // previous frame fully out
    for (int pi = 0; pi < NUM_PORTS; pi++) portStart(ports[pi], which);
    which ^= 1;
    g_frames++;
}

// True while the previous frame is still going out (for non-blocking loops).
static inline bool showBusy() {
    for (int pi = 0; pi < NUM_PORTS; pi++) if (portBusy(ports[pi])) return true;
    return false;
}

// ============================================================================
// Demo effect: a rainbow that scrolls along the strip at a fixed speed
// (independent of frame rate). Replace render() with your own.
// ============================================================================
struct Rgb { uint8_t r, g, b; };
static Rgb wheel[256];

static void buildWheel() {
    for (int i = 0; i < 256; i++) {
        uint8_t p = (uint8_t)i;
        if (p < 85)       wheel[i] = { (uint8_t)(255 - p * 3), (uint8_t)(p * 3), 0 };
        else if (p < 170) { p -= 85; wheel[i] = { 0, (uint8_t)(255 - p * 3), (uint8_t)(p * 3) }; }
        else              { p -= 170; wheel[i] = { (uint8_t)(p * 3), 0, (uint8_t)(255 - p * 3) }; }
    }
}

static void render(uint32_t nowMs) {
#if DEMO_MODE == 1
    (void)nowMs;
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (i & 1) setPixel(i, 0, 0, 0, 0, 255);     // odd: warm white
        else       setPixel(i, 0, 0, 0, 255, 0);     // even: cool white
    }
#else
    const uint32_t offset = (nowMs % 2000UL) * 256UL / 2000UL;  // one full hue cycle every 2 s
    for (int i = 0; i < NUM_PIXELS; i++) {
        uint8_t h = (uint8_t)((i * 512UL / NUM_PIXELS + offset) & 0xFF);   // two rainbows over the strip
        setPixel(i, wheel[h].r, wheel[h].g, wheel[h].b, 0, 0);
    }
#endif
}

// ============================================================================
void setup() {
#if REPORT_STATS
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 1500) { /* give a PC a moment, but never block */ }
#endif

    buildLuts();
    buildWheel();
    memset(frame, 0, sizeof(frame));

    portInit(ports[0], ConnectorCOM0, DMA_SERCOM7_SPI_RX, 0, PORT_PIXELS,
             spiBuf[0][0], spiBuf[0][1]);                  // COM-0 is SERCOM7
#if USE_TWO_PORTS
    portInit(ports[1], ConnectorCOM1, DMA_SERCOM0_SPI_RX, PORT_PIXELS, NUM_PIXELS - PORT_PIXELS,
             spiBuf[1][0], spiBuf[1][1]);                  // COM-1 is SERCOM0
#endif

    show();     // all black first
    show();     // ...and again so both DMA buffers hold a valid frame

#if REPORT_STATS
    Serial.print("WS2811 on ");            Serial.print(NUM_PORTS);          Serial.println(" COM port(s)");
    Serial.print("segments: ");            Serial.print(NUM_PIXELS);
    Serial.print("  ICs per segment: ");   Serial.print(ICS_PER_PIXEL);
    Serial.print("  segments per port: "); Serial.println(PORT_PIXELS);
    Serial.print("SPI clock: ");           Serial.print(SPI_HZ);             Serial.println(" Hz");
    Serial.print("bytes per frame/port: ");Serial.print(PORT_BYTES);
    Serial.print("  wire time: ");         Serial.print(FRAME_WIRE_US / 1000.0f, 2);
    Serial.print(" ms  -> max ");          Serial.print(1.0e6f / FRAME_WIRE_US, 1); Serial.println(" fps");
    Serial.print("LED power budget: ");    Serial.print(LED_BUDGET_WATTS, 1);
    Serial.print(" W   strip all-on: ");   Serial.print(STRIP_W_PER_M * STRIP_LENGTH_M, 1); Serial.println(" W");
#endif
}

void loop() {
    render(millis());
    show();                 // blocks only until the previous frame has left the wire

#if REPORT_STATS
    uint32_t now = millis();
    if (now - g_lastReport >= 1000) {
        Serial.print("fps ");          Serial.print(g_frames);
        Serial.print("  est ");        Serial.print(g_estWatts, 1);
        Serial.print(" W  limiter ");  Serial.print(g_powerScale == 255 ? 100 : (g_powerScale * 100) / 255);
        Serial.println(" %");
        g_frames = 0;
        g_lastReport = now;
    }
#endif
}
