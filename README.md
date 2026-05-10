# ESP32 Motorcycle Intercom — Multi-Rider Edition

A peer-to-peer, wireless intercom system for motorcycle riders built on the ESP32. No phone, no cellular, and no Wi-Fi router required. Riders form ad-hoc groups using ESP-NOW broadcast and can communicate in real time up to ~200 m line-of-sight.

## Features

- **True peer-to-peer** — ESP-NOW broadcast; no router or internet needed.
- **Multi-rider groups** — Up to 6 riders per group.
- **Group isolation** — Random 32-bit Group IDs let multiple groups ride in the same area without cross-talk.
- **VOX (voice-activated transmit)** — Hands-free operation; optional PTT button override.
- **Persistent groups** — Group membership is saved to NVS and survives reboots.
- **Add / Join on the road** — Simple one-button pairing to create a group or let a new rider join mid-ride.
- **Rider keepalive & timeout** — Automatic detection when a rider drops out.
- **Optional volume pot** — Hardware volume control via ADC.

## Hardware Requirements

| Component | Purpose |
|-----------|---------|
| ESP32 Dev Module | Main controller (CPU: 240 MHz) |
| INMP441 MEMS microphone | Audio input (I2S) |
| MAX98357A I2S amplifier | Audio output to speaker |
| 8 Ω speaker | Playback |
| 2× momentary buttons | PTT (GPIO 12) & PAIR (GPIO 13) |
| 3× LEDs (green, red, blue) + 220 Ω resistors | Status indicators |
| 10 kΩ potentiometer (optional) | Volume control (GPIO 34) |

## Wiring

### INMP441 Microphone → ESP32

| INMP441 | ESP32 |
|---------|-------|
| VDD | 3.3 V |
| GND | GND |
| SCK | GPIO 14 |
| WS | GPIO 15 |
| SD | GPIO 32 |
| L/R | GND *(selects left channel)* |

### MAX98357A Amplifier → ESP32

| MAX98357A | ESP32 |
|-----------|-------|
| VIN | 5 V *(or 3.3 V)* |
| GND | GND |
| BCLK | GPIO 26 |
| LRC | GPIO 25 |
| DIN | GPIO 22 |
| SD | 3.3 V *(always enabled)* |
| GAIN | GND *(6 dB)* |

### Controls & Indicators

| Function | Pin | Notes |
|----------|-----|-------|
| PTT button | GPIO 12 | To GND, internal pull-up |
| PAIR button | GPIO 13 | To GND, internal pull-up |
| Green LED | GPIO 2 | 220 Ω to GND |
| Red LED | GPIO 4 | 220 Ω to GND |
| Blue LED | GPIO 5 | 220 Ω to GND |
| Volume pot | GPIO 34 | 10 kΩ between 3.3 V and GND, wiper to GPIO 34 |

## Pairing & Group Operations

All pairing is done with the **PAIR button** (GPIO 13).

| Situation | Action | Result |
|-----------|--------|--------|
| No group yet | Hold PAIR 3 s | Create a new group + open 30 s Add-Rider window |
| Already in a group | Hold PAIR 3 s | Open 30 s Add-Rider window so others can join |
| New rider (no group) | Hold PAIR 3 s | Enter Join mode for 30 s; scans for a nearby group |
| Any state | Hold PAIR 10 s | Factory reset — clears saved group and reboots to idle |

When the Add-Rider window is open, existing group members broadcast their Group ID. A rider in Join mode hears the advertisement, requests to join, and is accepted automatically.

## LED Status Patterns

| LEDs | Pattern | Meaning |
|------|---------|---------|
| Green | Slow blink | Idle, no group |
| Green + Red | Fast blink | Add-Rider mode (advertising) |
| Blue | Fast blink | Join mode (scanning) |
| Green | Solid | Connected to group, idle |
| Green + Red | Solid | Transmitting (TX) |
| Green + Blue | Solid | Receiving (RX) |
| All three | Flash × 5 | Pairing succeeded |
| All three | Flash × 10 fast | Factory reset |

## Serial Debug Commands

Open the Serial Monitor at **115200 baud** and send single-character commands:

| Key | Action |
|-----|--------|
| `p` | Print rider table |
| `a` | Enter Add-Rider mode |
| `j` | Enter Join mode |
| `r` | Factory reset |

## Software Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) or [PlatformIO](https://platformio.org/)
- **ESP32 Arduino Core >= 2.0.0**
- Board selection: **ESP32 Dev Module**
- CPU frequency: **240 MHz**

### Required Arduino Libraries

All libraries used are bundled with the ESP32 Arduino core:
- `WiFi.h`
- `esp_now.h`
- `driver/i2s.h`
- `Preferences.h`

No additional library installation is required.

## Building & Flashing

1. Connect the ESP32 via USB.
2. Open `esp32_moto_intercom.ino` in the Arduino IDE.
3. Select **Tools → Board → ESP32 Arduino → ESP32 Dev Module**.
4. Select the correct **COM port**.
5. Set **CPU Frequency** to **240 MHz**.
6. Click **Upload**.
7. Open the Serial Monitor at **115200 baud** to verify boot messages.

## How It Works

1. **Audio path** — The INMP441 streams 16 kHz PCM via I2S into the ESP32. RMS level is computed for VOX gating. When voice is detected (or PTT is pressed), audio is packetized into 240-byte chunks and broadcast over ESP-NOW.
2. **Receive path** — Incoming packets are filtered by Group ID, then played out through the MAX98357A via I2S.
3. **Rider table** — Each rider’s MAC and ID are tracked. Keepalive pings are sent every 3 s; riders are removed if silent for > 10 s.
4. **Persistence** — The `Preferences` library stores the current Group ID and Rider ID in NVS so the device rejoins automatically after power cycling.

## License

This project is provided as-is for personal, educational, and open-source use.
