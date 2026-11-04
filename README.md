# ClearCore WS2811 COB strip driver with MQTT control

Teknic ClearCore (SAME53, Arduino wrapper) driving an addressable 24 V LED
strip from one COM port, with Ethernet + MQTT control. Strip presets:

* BTF-LIGHTING WS2805 RGBCCT 5050, 60 LEDs/m, 10 pixels/m, 5 m (one 5-channel
  WS2805 per pixel: R G B + two whites). The default.
* BTF-LIGHTING FCOB RGBCCT "Dual IC", 7.5 m (28 ICs/m, two WS2811 per
  segment: RGB + cool/warm white).
* Ordinary RGB WS2811 strips.

The `ClearCore_WS2811_COB` demo sketch supports the WS2811 strips only.

| Sketch | What it is |
|---|---|
| `ClearCore_WS2811_MQTT/` | Ethernet + MQTT control, 14 effects, power limiter. **Use this one.** |
| `ClearCore_WS2811_COB/` | Same LED engine, no network, scrolling rainbow demo. |
| `test/` | Host-side tests (no hardware): WS2811 waveform decoder, limiter, MQTT end-to-end against a real broker. |

## How the signal is made

The ClearCore has no logic-level GPIO fast enough for a WS2811 (the I/O
connectors are 24 V protected FET outputs), but each COM port is a 5 V SPI
master with DMA channels. The sketch synthesises the WS2811 waveform on the
COM port's **MOSI** line: each data bit becomes 3 SPI bits at 2.5 MHz
(`0` = 400 ns high / 800 ns low, `1` = 800 / 400 ns), and the whole frame is
pushed out by DMA (`SerialBase::SpiTransferDataAsync`). The CPU renders the
next frame while the current one is on the wire. 300 µs of zeros after each
frame latches the strip.

SPI clock comes from GCLK7 = 10 MHz in the ClearCore library, so 2.5 MHz and
5 MHz are exact. `TIMING_PROFILE 1` gives the WS2811-datasheet 200/600 ns
pulses at 5 MHz; `STRIP_KHZ 400` handles low-speed-mode parts.

WS2805 strips use `TIMING_PROFILE 2` (chosen automatically): 7 SPI bits at
5 MHz, `0` = 400 ns high / 1000 ns low, `1` = 800 / 600 ns, 1.4 µs per bit,
40 bits per pixel in the order R G B W1 W2. The WS2805 datasheet asks for a
`0` high time of 220–380 ns; a 10 MHz SPI clock can only make 200 or 400 ns,
and 400 ns is the safer of the two. All other times are inside the datasheet
ranges, and the reset is 300 µs (datasheet: at least 280 µs).

Frame rate ceiling: 28.8 µs per WS2811 IC + 0.3 ms. 105 RGBCCT segments
(210 ICs) → ~158 fps on COM-0, ~298 fps with `USE_TWO_PORTS 1`.

## Wiring

* COM-0 in SPI mode, RJ-45 pin numbers (ClearCore Hardware Manual, "COM-0, 1
  configured for SPI devices"):

  | Pin | SPI mode | Strip |
  |---|---|---|
  | 5 | MOSI (130 Ω series) | DIN |
  | 4 | GND | GND (run next to the data wire) |
  | 5 | | also BIN (backup data) on dual-data strips: BTF's WS2805 manual joins DIN and BIN to the data output |
  | 2, 6 | 5 V | not connected |
  | 1, 3, 7, 8 | SS, MISO, GND, SCK | not connected |

  With an RJ-45 screw-terminal breakout and a straight-through patch cable,
  terminal N is pin N: check for 5 V from terminal 4 to terminals 2 and 6
  before connecting the strip. Breakouts made for RS-485 often carry a 120 Ω
  termination resistor; if it sits across 4 and 5 it pulls the data line down
  to about 2.4 V, so remove it or switch it off.
* Keep the data cable short. Teknic rates the COM ports at 115.2 kBaud,
  "up to 2 MBaud achievable depending on cable length"; this runs MOSI at
  2.5 Mbit/s. **Scope the strip's DIN on first power-up**: clean ~5 V pulses of
  400/800 ns. Rounded 400 ns pulses → shorten the cable.
* 24 V into the strip at both ends (and the middle for 7.5 m).
* `USE_TWO_PORTS 1`: cut the strip's data trace at the midpoint, COM-1 MOSI
  into the second half's DIN. Power stays common.

## Power budget

Strip and ClearCore share one 24 V supply, `PSU_WATTS` (default 360 W, 24 V 15 A):

```
LED_BUDGET_WATTS = 360 W × 0.80 headroom − 7.2 W (ClearCore, 300 mA @ 24 V) = 280.8 W
```

Every frame's draw is estimated from the pixel data (all five channels on
CCT strips) and the frame is scaled down to stay inside the budget; colour
balance is preserved. The BTF FCOB strip is 18–23 W/m → 172.5 W all-on at 7.5 m,
and the WS2805 strip is at most 12 W/m → 60 W at 5 m: both inside the budget. Set `STRIP_W_PER_M` from your strip's label (or measure
it with everything on).

## MQTT

Set `MQTT_BROKER_IP` (an IP address; the ClearCore client has no DNS) and
`MQTT_PREFIX` in the sketch. DHCP by default; static IP via `USE_DHCP 0`.

| Topic | Direction | Content |
|---|---|---|
| `clearcore/leds/cmd` | → ClearCore | one text command per message |
| `clearcore/leds/state` | ← (retained) | JSON: effect, colours, speed, rate, size, dir, kelvin, brightness, fps, watts, limiter %, segments, ip |
| `clearcore/leds/reply` | ← | `ok …` / `error …` for each command |
| `clearcore/leds/status` | ← (retained) | `online` / `offline` (last will) |

Colours: `#RRGGBB`, `#RRGGBBCCWW`, `r,g,b`, `r,g,b,cw,ww`, or a name
(`red green blue yellow cyan magenta orange purple pink amber white warm cool rgbwhite black`).
`white` means the real white ICs on CCT strips.

| Command | Effect |
|---|---|
| `off` / `on` | dark / restore what ran before `off` |
| `solid <colour>` | one colour everywhere (alias `color`) |
| `white [kelvin] [level]` | tunable white, 2700–6500 K, level 0–255 |
| `chase [colour] [bg] [speed]` | comet with a fading tail (`size=` tail length) |
| `scanner [colour] [bg] [speed]` | bouncing bar (`size=` bar width) |
| `marquee [c1] [c2] [speed]` | scrolling blocks (`size=` block length) |
| `alternate [c1] [c2] [rate]` | blocks that swap colours (`size=` block length) |
| `flash [colour] [bg] [rate]` | strobe, Hz |
| `breathe [colour] [rate]` | sine fade, Hz |
| `rainbow [speed]` | scrolling hue wheel (`size=` pixels per cycle) |
| `wipe [colour] [bg] [speed]` | fill from the start, then empty |
| `sparkle [colour] [bg] [rate]` | random sparks, rate = sparks/s |
| `fire [rate]` | flames, rate = intensity 0.3–3 |
| `police [c1] [c2] [rate]` | halves flash alternately |
| `cycle [seconds]` | rotate through the built-in playlist |
| `brightness <0-255>` | global dimmer |
| `speed <px/s>`, `rate <Hz>`, `size <px>`, `dir fwd\|rev`, `kelvin`, `level` | tweak the running effect |
| `status` | publish state now |

Any command accepts `key=value` options: `speed= rate= size= dir= bri= kelvin= level= c1= c2= bg=`.

```
chase #ff4000 speed=80 size=12
alternate red green rate=0.5 size=6
white 3000 180
sparkle white #000020 rate=40
cycle 20
```

While the network is down the last segment blinks: red = no link,
orange = no IP, yellow = no broker (`NET_STATUS_PIXEL 0` disables it).
Animation never pauses for the network; MQTT is serviced between frames.

## Configuration checklist

In `ClearCore_WS2811_MQTT.ino`, section 1–4:

* `STRIP_PRESET` — `PRESET_BTF_WS2805_5M` (default), `PRESET_BTF_FCOB_DUAL_IC_7M5`,
  or `0` to set `PIXEL_TYPE` (`PIXEL_WS2805`, `PIXEL_RGBCCT_2IC`, `PIXEL_RGB`),
  `STRIP_LENGTH_M`, `PIXELS_PER_M` and `STRIP_W_PER_M` by hand
* `PIXELS_PER_M` — addressable *segments* per metre (BTF FCOB: 14, BTF WS2805: 10), not LEDs
* `STRIP_W_PER_M` — watts per metre with every channel full (BTF WS2805 24 V 60 LED/m: 12 W/m)
* `STRIP_WW_KELVIN`, `STRIP_CW_KELVIN` — the white LEDs' colour temperatures (for `white <kelvin>`)
* `CCT_ORDER` — BTF does not document which white byte is which. Send
  `white 3000` and `white 6500`; if they come out reversed, flip it.
* `PSU_WATTS` — the supply shared by the strip and the ClearCore
* `COLOR_ORDER` — if colours are swapped
* `MQTT_BROKER_IP`, `MQTT_PREFIX`, `MQTT_USER`/`MQTT_PASS`

## Footprint

Linked against Teknic's libClearCore + LwIP with the board's linker script:
212 KB flash of 496 KB, 49 KB static RAM of 192 KB (GCC 15; the Teknic board
package uses GCC 7, expect a few percent difference).

## Tests

```
python3 -m venv .venv && .venv/bin/pip install amqtt paho-mqtt
PATH="$PWD/.venv/bin:$PATH" test/run_tests.sh
```

Runs on a PC, no hardware. The encoder tests decode the SPI byte stream the
sketch hands to DMA exactly as the strip's ICs would, for every strip preset,
and compare it to an independent reference pipeline; they also check pulse
widths (against the WS2805 datasheet for the WS2805 preset), latch length and
that the limiter never exceeds the budget. The MQTT test starts a real
broker (amqtt), runs the sketch's `setup()`/`loop()` on POSIX sockets, and
drives it with paho-mqtt: connect, last will, every command, state JSON,
keep-alive over 20 s idle, reconnect after a broker restart.

## Known behaviour

* These strips flash white at power-up until the first frame arrives
  (inherent to the WS2811 ICs, see WLED issue #4641). The sketch sends black
  immediately at boot, but the ClearCore's boot time still shows a brief flash.
* `DhcpBegin()` blocks for up to ~7.5 s at boot when the cable is in but no
  DHCP server answers; LEDs start afterwards.

## References

* [ClearCore Hardware Manual](https://teknic.com/files/downloads/clearcore_user_manual.pdf) — COM port circuits, 5 V levels, baud statement, 300 mA draw
* [ClearCore-library](https://github.com/Teknic-Inc/ClearCore-library) — `SerialBase.cpp` (10 MHz SPI clock, DMA), `SysManager.cpp` (COM-0 = SERCOM7, COM-1 = SERCOM0), `EthernetTcpClient.cpp`
* [WS2811 datasheet](https://www.superlightingled.com/PDF/WS2811.pdf) — timing, ±150 ns, RGB order
* [WS2805 datasheet](https://github.com/Aircoookie/WLED/files/13753418/Worldsemi-WS2805.EN.pdf) — 40-bit RGBW1W2 frame, timing, reset ≥ 280 µs, dual data input
* [Adafruit DMA-driven NeoPixels](https://learn.adafruit.com/dma-driven-neopixels) — 3-bit SPI encoding precedent on SAMD51
* [BTF-LIGHTING FCOB RGBCCT dual-IC strip](https://www.btf-lighting.com/products/fcob-ws2811-dual-ic-rgbcct-addressable-led-strip) — 28 ICs/m, 60 LEDs / 71 mm per segment
* [WLED issue #4641](https://github.com/wled/WLED/issues/4641) — dual-IC layout (RGB + CW/WW + blank), startup flash
