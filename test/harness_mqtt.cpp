// Runs the MQTT sketch on the host against a real broker. Stops when the file
// named by argv[1] appears, or after argv[2] seconds.
#include "ClearCore.h"
#include <sys/stat.h>
DmacChannel g_dma[DMA_CHANNEL_COUNT];
SerialDriver ConnectorCOM0("COM0", DMA_SERCOM7_SPI_RX);
SerialDriver ConnectorCOM1("COM1", DMA_SERCOM0_SPI_RX);
FakeSerial Serial;
FakeEthernetManager EthernetMgr;
unsigned g_wireMs = 6;
#include SKETCH_PATH

// Expose a few internals for the test log (same translation unit)
int main(int argc, char **argv) {
    const char *stopFile = argc > 1 ? argv[1] : "/tmp/stop";
    int maxSec = argc > 2 ? atoi(argv[2]) : 120;
    setup();
    uint32_t t0 = millis();
    uint32_t lastDump = 0;
    while (millis() - t0 < (uint32_t)maxSec * 1000) {
        loop();
        struct stat st;
        if (stat(stopFile, &st) == 0) break;
        if (millis() - lastDump > 1000) {        // dump the first pixels so the test can see rendering
            lastDump = millis();
            FILE *f = fopen("frame_dump.txt", "w");
            if (f) {
                fprintf(f, "fx=%s bri=%u ", FX_NAMES[g_scene.fx], g_brightness);
                for (int i = 0; i < 6; i++) fprintf(f, "[%u,%u,%u,%u,%u]", frame[i].r, frame[i].g, frame[i].b, frame[i].cw, frame[i].ww);
                fprintf(f, "\n");
                fclose(f);
            }
        }
    }
    printf("harness: exiting, %u frames\n", ConnectorCOM0.transfers);
    return 0;
}
