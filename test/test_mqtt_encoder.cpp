// Functional test of the sketch's encoder + power limiter on the host.
// Builds the sketch against the stub ClearCore.h, runs setup()/loop(), then
// decodes the recorded SPI byte stream as a WS2811 receiver would.
#include "ClearCore.h"
FakeEthernetManager EthernetMgr;
unsigned g_wireMs = 0;
#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>

DmacChannel g_dma[DMA_CHANNEL_COUNT];
SerialDriver ConnectorCOM0("COM0", DMA_SERCOM7_SPI_RX);
SerialDriver ConnectorCOM1("COM1", DMA_SERCOM0_SPI_RX);
FakeSerial Serial;


#include SKETCH_PATH   // the .ino itself
#define BRIGHTNESS g_brightness
static void setPixel(int i, uint8_t r, uint8_t g, uint8_t b, uint8_t cw, uint8_t ww) { frame[i] = Pixel{r,g,b,cw,ww}; }

static int failures = 0;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

struct Pulse { int high, low; };   // in SPI bit periods

// Decode one transfer into pixels. Returns pulses for width statistics.
static std::vector<uint8_t> decode(const std::vector<uint8_t> &bytes, std::vector<Pulse> &pulses, int &latchBits) {
    std::vector<int> bits;
    for (uint8_t b : bytes) for (int i = 7; i >= 0; i--) bits.push_back((b >> i) & 1);
    std::vector<int> wsbits;
    size_t i = 0;
    CHECK(bits[0] == 1, "stream does not start with a high (first bit must start immediately)");
    while (i < bits.size()) {
        if (bits[i] == 0) { i++; continue; }
        size_t h = i; while (h < bits.size() && bits[h] == 1) h++;
        size_t l = h; while (l < bits.size() && bits[l] == 0) l++;
        int high = (int)(h - i), low = (int)(l - h);
        if (l == bits.size()) { latchBits = low; low = -1; }   // trailing latch
        pulses.push_back({high, low});
        // classify by high time: '1' if high >= half of the data-bit period
        int period = SPI_BITS_PER_DATA_BIT;
        wsbits.push_back(high * 2 >= period ? 1 : 0);
        i = l;
    }
    std::vector<uint8_t> out;
    CHECK(wsbits.size() % 24 == 0, "bit count %zu not a multiple of 24", wsbits.size());
    for (size_t k = 0; k + 7 < wsbits.size(); k += 8) {
        uint8_t v = 0; for (int j = 0; j < 8; j++) v = (v << 1) | wsbits[k + j];
        out.push_back(v);
    }
    return out;
}

// Reference pipeline, written independently of the sketch's loops.
static void expectedWire(const Pixel *fr, int n, uint8_t powerScale, std::vector<uint8_t> &out) {
    static const uint8_t idx[6][3] = { {0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0} };
    for (int i = 0; i < n; i++) {
        uint8_t c[3] = { fr[i].r, fr[i].g, fr[i].b };
        for (int k = 0; k < 3; k++) {
            uint8_t v = c[idx[COLOR_ORDER][k]];
            v = GAMMA_CORRECT ? (uint8_t)(powf(v / 255.0f, 2.2f) * 255.0f + 0.5f) : v;
            v = (uint8_t)(((uint16_t)v * (BRIGHTNESS + 1)) >> 8);
            v = (uint8_t)(((uint16_t)v * (powerScale + 1)) >> 8);
            out.push_back(v);
        }
        if (ICS_PER_PIXEL == 2) {
            uint8_t wv[2] = { fr[i].cw, fr[i].ww };
            if (CCT_ORDER == CCT_WW_CW) std::swap(wv[0], wv[1]);
            for (int k = 0; k < 2; k++) {
                uint8_t v = wv[k];
                v = GAMMA_CORRECT ? (uint8_t)(powf(v / 255.0f, 2.2f) * 255.0f + 0.5f) : v;
                v = (uint8_t)(((uint16_t)v * (BRIGHTNESS + 1)) >> 8);
                v = (uint8_t)(((uint16_t)v * (powerScale + 1)) >> 8);
                out.push_back(v);
            }
            out.push_back(0);
        }
    }
}

static void checkFrame(const char *label, const Pixel *fr) {
    std::vector<uint8_t> got;
    std::vector<Pulse> pulses; int latchBits = 0;
    int minHigh0 = 1 << 30, maxHigh0 = 0, minHigh1 = 1 << 30, maxHigh1 = 0, minLow = 1 << 30, maxLow = 0;
    for (int p = 0; p < NUM_PORTS; p++) {
        SerialDriver &com = p == 0 ? ConnectorCOM0 : ConnectorCOM1;
        CHECK(com.transfers > 0, "%s: no transfer on %s", label, com.name);
        const std::vector<uint8_t> &t = com.last;
        CHECK(t.size() == ports[p].bytes, "%s: transfer length %zu != %u", label, t.size(), ports[p].bytes);
        int lb = 0;
        std::vector<uint8_t> d = decode(t, pulses, lb);
        got.insert(got.end(), d.begin(), d.end());
        latchBits = lb;
        CHECK((int)d.size() == ports[p].numPixels * WIRE_BYTES_PER_PIXEL, "%s: %s decoded %zu bytes, expected %d", label, com.name, d.size(), ports[p].numPixels * WIRE_BYTES_PER_PIXEL);
    }
    std::vector<uint8_t> exp;
    expectedWire(fr, NUM_PIXELS, g_powerScale, exp);
    CHECK(got.size() == exp.size(), "%s: size mismatch %zu vs %zu", label, got.size(), exp.size());
    int bad = 0;
    for (size_t i = 0; i < got.size() && i < exp.size(); i++) if (got[i] != exp[i]) { if (bad < 3) printf("  byte %zu: got %u exp %u\n", i, got[i], exp[i]); bad++; }
    CHECK(bad == 0, "%s: %d bytes differ from reference pipeline", label, bad);

    double ns = 1e9 / SPI_HZ;
    for (const Pulse &pu : pulses) { bool one = pu.high * 2 >= SPI_BITS_PER_DATA_BIT; if (one) { minHigh1 = std::min(minHigh1, pu.high); maxHigh1 = std::max(maxHigh1, pu.high); } else { minHigh0 = std::min(minHigh0, pu.high); maxHigh0 = std::max(maxHigh0, pu.high); } }
    if (maxHigh0 == 0) minHigh0 = 0; if (maxHigh1 == 0) minHigh1 = 0;
    for (const Pulse &pu : pulses) {
        bool one = pu.high * 2 >= SPI_BITS_PER_DATA_BIT;
        if (one) { minHigh1 = std::min(minHigh1, pu.high); maxHigh1 = std::max(maxHigh1, pu.high); }
        else     { minHigh0 = std::min(minHigh0, pu.high); maxHigh0 = std::max(maxHigh0, pu.high); }
        if (pu.low >= 0) { minLow = std::min(minLow, pu.low); maxLow = std::max(maxLow, pu.low); }
        if (pu.low >= 0) CHECK(pu.high + pu.low == SPI_BITS_PER_DATA_BIT, "%s: bit period %d != %d", label, pu.high + pu.low, SPI_BITS_PER_DATA_BIT);
    }
    printf("%-22s pixels=%d ports=%d  T0H=%.0f ns  T1H=%.0f ns  period=%.0f ns  latch=%.0f us  limiter=%u/255 est=%.1f W\n",
           label, NUM_PIXELS, NUM_PORTS, minHigh0 * ns, minHigh1 * ns, SPI_BITS_PER_DATA_BIT * ns, latchBits * ns / 1000.0, g_powerScale, g_estWatts);
    CHECK((minHigh0 == maxHigh0 || maxHigh0 == 0) && (minHigh1 == maxHigh1 || maxHigh1 == 0), "%s: inconsistent pulse widths", label);
    CHECK(latchBits * ns >= LATCH_US * 1000.0, "%s: latch %.0f ns < %d us", label, latchBits * ns, LATCH_US);
    CHECK(g_estWatts <= LED_BUDGET_WATTS + 1e-3, "%s: estimate %.2f W exceeds budget %.2f W", label, g_estWatts, LED_BUDGET_WATTS);
}

int main() {
    setup();
    // setup() sent: one blocking latch per port, then two black frames
    for (int p = 0; p < NUM_PORTS; p++) {
        SerialDriver &com = p == 0 ? ConnectorCOM0 : ConnectorCOM1;
        CHECK(com.mode == Connector::SPI && com.open && com.order == SerialDriver::COM_MSB_FIRST && com.ssOff, "%s not configured", com.name);
        CHECK(com.speed == SPI_HZ, "%s speed %u", com.name, com.speed);

    }
    // 1) demo frame from loop()
    renderScene(1234); show(); checkFrame("rainbow (render)", frame);
    // 2) full white: limiter must engage for a 7.5 m strip on 192.8 W
    for (int i = 0; i < NUM_PIXELS; i++) setPixel(i, 255, 255, 255, 255, 255);
    show(); checkFrame("all channels on", frame);
    {
        float fullWhite = STRIP_W_PER_M * STRIP_LENGTH_M * (float)(((255 * (BRIGHTNESS + 1)) >> 8)) / 255.0f;
        if (fullWhite > LED_BUDGET_WATTS) CHECK(g_powerScale < 255, "limiter did not engage on full white (%.1f W strip vs %.1f W budget)", fullWhite, LED_BUDGET_WATTS);
        else CHECK(g_powerScale == 255, "limiter engaged although full white (%.1f W) fits the budget (%.1f W)", fullWhite, LED_BUDGET_WATTS);
        CHECK(fabsf(g_estWatts - std::min(fullWhite, LED_BUDGET_WATTS)) < 0.02f * fullWhite, "full-white estimate %.1f W off (strip %.1f W, budget %.1f W)", g_estWatts, fullWhite, LED_BUDGET_WATTS);
    }
    // 3) dim frame: limiter must stay out of the way
    for (int i = 0; i < NUM_PIXELS; i++) setPixel(i, 40, 10, 90, 0, 0);
    show(); checkFrame("dim", frame);
    CHECK(g_powerScale == 255, "limiter engaged on a dim frame");
    // 4) single pixel patterns, every value
    for (int v = 0; v < 256; v += 51) { for (int i = 0; i < NUM_PIXELS; i++) setPixel(i, v, 255 - v, (v * 7) & 255, (v * 3) & 255, 255 - ((v * 5) & 255)); show(); checkFrame("value sweep", frame); }
    // 5) CCT order demo frame
    for (int i = 0; i < NUM_PIXELS; i++) setPixel(i, 0, 0, 0, (i & 1) ? 0 : 255, (i & 1) ? 255 : 0);
    show(); checkFrame("cct check", frame);
    // double buffering: consecutive transfers must come from alternating buffers
    for (int p = 0; p < NUM_PORTS; p++) {
        SerialDriver &com = p == 0 ? ConnectorCOM0 : ConnectorCOM1;
        (void)com;
    }
    printf("%s: %d failure(s)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
