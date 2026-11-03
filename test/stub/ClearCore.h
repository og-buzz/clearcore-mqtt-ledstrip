// Host-side stub of the ClearCore API surface used by the sketch, for a
// functional test of the encoder / limiter. Records every SPI transfer.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <stdlib.h>

struct Connector { enum ConnectorModes { TTL, RS232, SPI, CCIO }; };

typedef enum { DMA_SERCOM0_SPI_RX, DMA_SERCOM0_SPI_TX, DMA_SERCOM7_SPI_RX, DMA_SERCOM7_SPI_TX, DMA_CHANNEL_COUNT } DmaChannels;

struct DmacChannel { struct { struct { unsigned ENABLE : 1; } bit; } CHCTRLA; };
extern DmacChannel g_dma[DMA_CHANNEL_COUNT];
struct DmaManager { static DmacChannel *Channel(DmaChannels c) { return &g_dma[c]; } };

struct SerialDriver {
    enum DataOrders { COM_MSB_FIRST, COM_LSB_FIRST };
    enum SpiClockPolarities { SCK_LOW, SCK_HIGH };
    enum SpiClockPhases { LEAD_SAMPLE, LEAD_CHANGE };
    enum CtrlLineModes { LINE_OFF, LINE_ON, LINE_HW };

    const char *name; DmaChannels rx;
    SerialDriver(const char *n, DmaChannels r) : name(n), rx(r) {}
    bool open = false; Connector::ConnectorModes mode = Connector::TTL; uint32_t speed = 0;
    DataOrders order = COM_LSB_FIRST; bool ssOff = false;
    std::vector<std::vector<uint8_t>> transfers;   // every transfer, in order
    bool asyncPending = false;

    bool Mode(Connector::ConnectorModes m) { mode = m; return true; }
    bool Speed(uint32_t s) { speed = s; return true; }
    void DataOrder(DataOrders o) { order = o; }
    void SpiClock(SpiClockPolarities, SpiClockPhases) {}
    void PortOpen() { open = true; }
    bool SpiSsMode(CtrlLineModes m) { ssOff = (m == LINE_OFF); return true; }
    int32_t SpiTransferData(const uint8_t *w, uint8_t *, int32_t len) {
        if (!open || mode != Connector::SPI) return 0;
        transfers.emplace_back(w, w + len); return len;
    }
    bool SpiTransferDataAsync(const uint8_t *w, uint8_t *, int32_t len) {
        if (!open || mode != Connector::SPI) return false;
        if (asyncPending) { fprintf(stderr, "FAIL: %s async started while previous still pending\n", name); exit(2); }
        transfers.emplace_back(w, w + len);
        asyncPending = true; g_dma[rx].CHCTRLA.bit.ENABLE = 1;
        return true;
    }
    bool SpiAsyncWaitComplete() { asyncPending = false; g_dma[rx].CHCTRLA.bit.ENABLE = 0; return true; }
};
extern SerialDriver ConnectorCOM0, ConnectorCOM1;

struct FakeSerial {
    void begin(unsigned long) {}
    operator bool() const { return true; }
    template <typename T> void print(T v) { (void)v; }
    void print(float v, int) { (void)v; }
    void print(const char *s) { fputs(s, stdout); }
    void println(const char *s) { puts(s); }
    template <typename T> void println(T v) { (void)v; puts(""); }
};
extern FakeSerial Serial;
extern uint32_t g_millis;
static inline uint32_t millis() { return g_millis; }
