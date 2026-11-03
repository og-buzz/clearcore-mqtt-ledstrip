// Host-side stub of the ClearCore API used by ClearCore_WS2811_MQTT.ino.
// LED side: records SPI transfers. Network side: real POSIX sockets, so the
// sketch's MQTT client talks to a real broker.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <poll.h>

static inline uint32_t millis() {
    using namespace std::chrono;
    static const steady_clock::time_point t0 = steady_clock::now();
    return (uint32_t)duration_cast<milliseconds>(steady_clock::now() - t0).count();
}

struct Connector { enum ConnectorModes { TTL, RS232, SPI, CCIO }; };
typedef enum { DMA_SERCOM0_SPI_RX, DMA_SERCOM0_SPI_TX, DMA_SERCOM7_SPI_RX, DMA_SERCOM7_SPI_TX, DMA_CHANNEL_COUNT } DmaChannels;
struct DmacChannel { struct { struct { unsigned ENABLE : 1; } bit; } CHCTRLA; };
extern DmacChannel g_dma[DMA_CHANNEL_COUNT];
struct DmaManager { static DmacChannel *Channel(DmaChannels c) { return &g_dma[c]; } };

extern unsigned g_wireMs;          // simulated wire time per frame

struct SerialDriver {
    enum DataOrders { COM_MSB_FIRST, COM_LSB_FIRST };
    enum SpiClockPolarities { SCK_LOW, SCK_HIGH };
    enum SpiClockPhases { LEAD_SAMPLE, LEAD_CHANGE };
    enum CtrlLineModes { LINE_OFF, LINE_ON, LINE_HW };
    const char *name; DmaChannels rx;
    SerialDriver(const char *n, DmaChannels r) : name(n), rx(r) {}
    bool open = false; Connector::ConnectorModes mode = Connector::TTL; uint32_t speed = 0;
    DataOrders order = COM_LSB_FIRST; bool ssOff = false;
    std::vector<uint8_t> last;          // last transfer only
    unsigned transfers = 0;
    bool asyncPending = false;
    bool Mode(Connector::ConnectorModes m) { mode = m; return true; }
    bool Speed(uint32_t s) { speed = s; return true; }
    void DataOrder(DataOrders o) { order = o; }
    void SpiClock(SpiClockPolarities, SpiClockPhases) {}
    void PortOpen() { open = true; }
    bool SpiSsMode(CtrlLineModes m) { ssOff = (m == LINE_OFF); return true; }
    int32_t SpiTransferData(const uint8_t *w, uint8_t *, int32_t len) {
        if (!open || mode != Connector::SPI) return 0;
        last.assign(w, w + len); transfers++; return len;
    }
    bool SpiTransferDataAsync(const uint8_t *w, uint8_t *, int32_t len) {
        if (!open || mode != Connector::SPI) return false;
        if (asyncPending) { fprintf(stderr, "FAIL: async overlap on %s\n", name); exit(2); }
        last.assign(w, w + len); transfers++;
        asyncPending = true; g_dma[rx].CHCTRLA.bit.ENABLE = 1;
        return true;
    }
    bool SpiAsyncWaitComplete() {
        if (asyncPending && g_wireMs) std::this_thread::sleep_for(std::chrono::milliseconds(g_wireMs));
        asyncPending = false; g_dma[rx].CHCTRLA.bit.ENABLE = 0; return true;
    }
};
extern SerialDriver ConnectorCOM0, ConnectorCOM1;

struct FakeSerial {
    void begin(unsigned long) {}
    operator bool() const { return true; }
    void print(const char *s) { fputs(s, stdout); }
    void println(const char *s) { puts(s); fflush(stdout); }
    void print(int v) { printf("%d", v); }
    void print(unsigned v) { printf("%u", v); }
    void print(unsigned long v) { printf("%lu", v); }
    void print(long v) { printf("%ld", v); }
    void print(double v, int d) { printf("%.*f", d, v); }
    void println(int v) { printf("%d\n", v); fflush(stdout); }
    void println(unsigned v) { printf("%u\n", v); fflush(stdout); }
    void println(unsigned long v) { printf("%lu\n", v); fflush(stdout); }
    void println(double v, int d) { printf("%.*f\n", d, v); fflush(stdout); }
};
extern FakeSerial Serial;

// ---------------------------------------------------------------- network
class IpAddress {
public:
    IpAddress() { m_s[0] = '\0'; }
    explicit IpAddress(const char *s) { strncpy(m_s, s, sizeof(m_s) - 1); m_s[sizeof(m_s) - 1] = '\0'; }
    char *StringValue() { return m_s; }
private:
    char m_s[20];
};

struct FakeEthernetManager {
    bool linkUp = true; bool haveIp = false;
    void Setup() {}
    bool DhcpBegin() { haveIp = true; return true; }
    void Refresh() {}
    const bool &PhyLinkActive() { return linkUp; }
    IpAddress LocalIp() { return IpAddress("127.0.0.1"); }
    void LocalIp(IpAddress) { haveIp = true; }
    void NetmaskIp(IpAddress) {}
    void GatewayIp(IpAddress) {}
};
extern FakeEthernetManager EthernetMgr;

// Mirrors the real class semantics: Connect() blocks up to the timeout,
// BytesAvailable() is non-blocking, Read() drains what is there, Send() returns
// bytes queued, Connected() goes false after the peer closes.
class EthernetTcpClient {
public:
    void ConnectionTimeout(uint16_t t) { m_timeout = t; }
    bool Connect(IpAddress ip, uint16_t port) {
        if (m_fd >= 0 && Connected()) return false;
        if (m_fd >= 0) Close();
        m_fd = socket(AF_INET, SOCK_STREAM, 0);
        int one = 1; setsockopt(m_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        fcntl(m_fd, F_SETFL, O_NONBLOCK);
        sockaddr_in a; memset(&a, 0, sizeof(a)); a.sin_family = AF_INET; a.sin_port = htons(port);
        inet_pton(AF_INET, ip.StringValue(), &a.sin_addr);
        int r = connect(m_fd, (sockaddr *)&a, sizeof(a));
        if (r < 0 && errno != EINPROGRESS) { Close(); return false; }
        pollfd p = { m_fd, POLLOUT, 0 };
        if (poll(&p, 1, m_timeout) <= 0) { Close(); return false; }
        int err = 0; socklen_t el = sizeof(err); getsockopt(m_fd, SOL_SOCKET, SO_ERROR, &err, &el);
        if (err) { Close(); return false; }
        m_peerClosed = false;
        return true;
    }
    bool Connected() {
        if (m_fd < 0 || m_peerClosed) return false;
        char c; ssize_t n = recv(m_fd, &c, 1, MSG_PEEK | MSG_DONTWAIT);
        if (n == 0) { m_peerClosed = true; return false; }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) { m_peerClosed = true; return false; }
        return true;
    }
    int16_t BytesAvailable() {
        if (m_fd < 0) return 0;
        int n = 0; ioctl(m_fd, FIONREAD, &n);
        return (int16_t)(n > 600 ? 600 : n);          // real ring buffer is 600 bytes
    }
    int16_t Read(uint8_t *dst, uint32_t len) {
        if (m_fd < 0) return -1;
        ssize_t n = recv(m_fd, dst, len, MSG_DONTWAIT);
        return n < 0 ? 0 : (int16_t)n;
    }
    uint32_t Send(const uint8_t *buf, uint32_t size) {
        if (m_fd < 0 || m_peerClosed) return 0;
        ssize_t n = send(m_fd, buf, size, MSG_NOSIGNAL | MSG_DONTWAIT);
        return n < 0 ? 0 : (uint32_t)n;
    }
    void Close() { if (m_fd >= 0) { close(m_fd); m_fd = -1; } m_peerClosed = true; }
private:
    int m_fd = -1; uint16_t m_timeout = 2000; bool m_peerClosed = true;
};
