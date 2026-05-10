/*
 * ============================================================
 *  ESP32 Motorcycle Intercom  –  Multi-Rider Edition
 * ============================================================
 *
 *  Protocol : ESP-NOW broadcast (no WiFi router needed)
 *  Audio In : INMP441 MEMS microphone  (I2S_NUM_0)
 *  Audio Out: MAX98357A I2S amplifier  (I2S_NUM_1)
 *  Riders   : Up to 6 per group
 *  Range    : ~200 m line-of-sight
 *
 *  HOW GROUPS WORK
 *  ---------------
 *  Each group shares a random 32-bit Group ID.  All audio is
 *  broadcast over ESP-NOW; packets with a non-matching Group ID
 *  are silently discarded.  This lets multiple groups ride in
 *  the same area without hearing each other.
 *
 *  PAIRING OPERATIONS  (PAIR button = GPIO 13)
 *  -------------------------------------------
 *  No group yet, hold 3 s  →  Create a new group (you become host)
 *                              AND start Add-Rider window (30 s)
 *  In a group,  hold 3 s  →  Open Add-Rider window (30 s)
 *                              (advertises your Group ID so a new
 *                               rider can join)
 *  New rider,   hold 3 s  →  Join mode: scan for an advertisement
 *  Hold 10 s anywhere     →  Factory reset (clears saved group)
 *
 *  LED PATTERNS
 *  ------------
 *  Green slow blink           = idle, no group
 *  Green + Red fast blink     = Add-Rider mode (advertising)
 *  Blue fast blink            = Join mode (scanning)
 *  Green solid                = connected, idle
 *  Green + Red solid          = transmitting (TX)
 *  Green + Blue solid         = receiving   (RX)
 *  All 3 flash × 5            = pairing succeeded
 *  All 3 flash × 10 fast      = factory reset
 *
 *  SERIAL DEBUG COMMANDS  (115200 baud)
 *  ------------------------------------
 *  p  →  print rider table
 *  a  →  enter Add-Rider mode
 *  j  →  enter Join mode
 *  r  →  factory reset
 *
 *  WIRING
 *  ------
 *  INMP441 microphone:
 *    VDD  -> 3.3 V
 *    GND  -> GND
 *    SCK  -> GPIO 14
 *    WS   -> GPIO 15
 *    SD   -> GPIO 32
 *    L/R  -> GND  (selects left channel)
 *
 *  MAX98357A amplifier:
 *    VIN  -> 5 V (or 3.3 V)
 *    GND  -> GND
 *    BCLK -> GPIO 26
 *    LRC  -> GPIO 25
 *    DIN  -> GPIO 22
 *    SD   -> 3.3 V (always enabled)
 *    GAIN -> GND  (6 dB)
 *
 *  Controls / Indicators:
 *    PTT  button -> GPIO 12  (to GND; uses PULLUP)
 *    PAIR button -> GPIO 13  (to GND; uses PULLUP)
 *    Green LED   -> GPIO  2  (+ 220 Ω to GND)
 *    Red   LED   -> GPIO  4  (+ 220 Ω to GND)
 *    Blue  LED   -> GPIO  5  (+ 220 Ω to GND)
 *    Vol pot     -> GPIO 34  (optional: 10 kΩ between 3.3 V and GND,
 *                             wiper to GPIO 34)
 *
 *  REQUIRED
 *  --------
 *  ESP32 Arduino core >= 2.0.0
 *  Board: "ESP32 Dev Module", CPU: 240 MHz
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>
#include <Preferences.h>
#include <math.h>

// ──────────────────────────────────────────────
//  PIN DEFINITIONS
// ──────────────────────────────────────────────
#define MIC_SCK_PIN     14
#define MIC_WS_PIN      15
#define MIC_SD_PIN      32

#define SPK_BCLK_PIN    26
#define SPK_LRC_PIN     25
#define SPK_DIN_PIN     22

#define PTT_PIN         12
#define PAIR_PIN        13
#define LED_GREEN_PIN    2   // status / group connected
#define LED_RED_PIN      4   // TX / advertising
#define LED_BLUE_PIN     5   // RX / joining
#define VOL_ADC_PIN     34   // optional volume potentiometer

// ──────────────────────────────────────────────
//  AUDIO CONFIG
// ──────────────────────────────────────────────
#define SAMPLE_RATE         16000
#define DMA_BUF_COUNT           4
#define DMA_BUF_LEN           256
#define MIC_SAMPLES           256
// ESP-NOW max payload = 250 bytes; header = 10 bytes
#define MAX_AUDIO_BYTES       240

// ──────────────────────────────────────────────
//  VOX CONFIG
// ──────────────────────────────────────────────
#define VOX_THRESHOLD        800   // RMS trigger level
#define VOX_HOLD_MS          600   // keep TX open after silence

// ──────────────────────────────────────────────
//  GROUP / PAIRING CONFIG
// ──────────────────────────────────────────────
#define MAX_RIDERS             6
#define RIDER_TIMEOUT_MS   10000   // remove silent rider from table
#define PAIR_HOLD_MS        3000   // hold duration → pairing action
#define RESET_HOLD_MS      10000   // hold duration → factory reset
#define ADD_RIDER_WINDOW   30000   // how long to advertise (ms)
#define JOIN_WINDOW        30000   // how long to scan (ms)
#define ADV_INTERVAL_MS      800   // advertisement broadcast period

// ──────────────────────────────────────────────
//  PACKET TYPES
// ──────────────────────────────────────────────
#define PKT_AUDIO       0x00   // voice (broadcast)
#define PKT_PING        0x01   // keepalive (broadcast)
#define PKT_GROUP_ADV   0x02   // group advertisement
#define PKT_JOIN_REQ    0x03   // request to join
#define PKT_JOIN_ACK    0x04   // accept join + send group ID
#define PKT_LEAVE       0x05   // graceful leave

// ──────────────────────────────────────────────
//  STATE MACHINE
// ──────────────────────────────────────────────
typedef enum {
    STATE_IDLE,          // not in any group
    STATE_ADD_RIDER,     // advertising group ID for new joiner
    STATE_JOINING,       // scanning for a group to join
    STATE_CONNECTED,     // normal idle in group
    STATE_TX,            // transmitting audio
    STATE_RX,            // receiving audio
} State;

// ──────────────────────────────────────────────
//  PACKET LAYOUT  (10-byte header + audio data)
//
//  Byte  0     : type     (1)
//  Bytes 1–4   : groupID  (4)
//  Byte  5     : riderID  (1)
//  Bytes 6–7   : seq      (2)
//  Bytes 8–9   : len      (2)
//  Bytes 10–249: data     (up to 240)
// ──────────────────────────────────────────────
#pragma pack(push, 1)
typedef struct {
    uint8_t  type;
    uint32_t groupID;
    uint8_t  riderID;
    uint16_t seq;
    uint16_t len;
    uint8_t  data[MAX_AUDIO_BYTES];
} Packet;
#pragma pack(pop)

static const int PKT_HDR = 10;   // header-only size (no data)

// ──────────────────────────────────────────────
//  RIDER TRACKING TABLE
// ──────────────────────────────────────────────
typedef struct {
    bool     active;
    uint8_t  riderID;
    uint8_t  mac[6];
    uint32_t lastHeardMs;
} RiderEntry;

// ──────────────────────────────────────────────
//  GLOBALS
// ──────────────────────────────────────────────
static State       gState        = STATE_IDLE;
static bool        gInGroup      = false;
static uint32_t    gGroupID      = 0;
static uint8_t     gMyRiderID    = 0;
static uint16_t    gTxSeq        = 0;
static int         gRiderCount   = 0;

static RiderEntry  gRiders[MAX_RIDERS];

static volatile bool gNewRxPkt   = false;
static Packet        gRxPkt;
static Packet        gTxPkt;

static uint32_t    gLastRxMs     = 0;
static uint32_t    gVoxHoldMs    = 0;
static uint32_t    gPairWindowEnd = 0;

static int16_t     gMicBuf[MIC_SAMPLES];
static Preferences gPrefs;

static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ──────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ──────────────────────────────────────────────
void    setupI2SMic();
void    setupI2SSpeaker();
void    setupESPNow();
void    broadcastPkt(Packet *p, int totalBytes);
void    unicastPkt(const uint8_t *mac, Packet *p, int totalBytes);
void    handlePairButton();
void    enterAddRiderMode();
void    enterJoinMode();
void    factoryReset();
void    pairingTick();
bool    readMic();
void    txAudio();
void    rxAudio();
int     rmsLevel(const int16_t *buf, int n);
void    applyVolume(int16_t *buf, int n, float gain);
void    updateRiderTable(uint8_t id, const uint8_t *mac);
void    pruneRiders();
void    printStatus();
void    saveGroup();
void    loadGroup();
void    flashAll(int count, int onMs, int offMs);
void    updateLEDs();
uint32_t makeGroupID();
uint8_t  makeRiderID();

// ──────────────────────────────────────────────
//  ESP-NOW RX CALLBACK
//  (runs in WiFi task context – keep it short)
// ──────────────────────────────────────────────
void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < PKT_HDR) return;

    Packet pkt;
    memcpy(&pkt, data, min(len, (int)sizeof(Packet)));

    // ── Pairing packets – process regardless of group membership ──

    if (pkt.type == PKT_GROUP_ADV) {
        if (gState == STATE_JOINING) {
            // Found a group – send join request back to the advertiser
            Packet req  = {};
            req.type    = PKT_JOIN_REQ;
            req.groupID = pkt.groupID;
            req.riderID = gMyRiderID;
            unicastPkt(mac, &req, PKT_HDR);
            Serial.printf("[PAIR] Saw group %08X – join request sent\n", pkt.groupID);
        }
        return;
    }

    if (pkt.type == PKT_JOIN_REQ) {
        if (gState == STATE_ADD_RIDER && gInGroup) {
            // Accept the new rider: send ACK with our group ID
            Packet ack  = {};
            ack.type    = PKT_JOIN_ACK;
            ack.groupID = gGroupID;
            ack.riderID = gMyRiderID;
            unicastPkt(mac, &ack, PKT_HDR);
            updateRiderTable(pkt.riderID, mac);
            Serial.printf("[PAIR] Rider %02X accepted into group\n", pkt.riderID);
            // Stay in ADD_RIDER for the rest of the window (allow more joiners)
        }
        return;
    }

    if (pkt.type == PKT_JOIN_ACK) {
        if (gState == STATE_JOINING) {
            gGroupID = pkt.groupID;
            gInGroup = true;
            saveGroup();
            gState = STATE_CONNECTED;
            updateRiderTable(pkt.riderID, mac);
            Serial.printf("[PAIR] Joined group %08X  RiderID=%02X\n",
                          gGroupID, gMyRiderID);
            // flashAll is not ISR-safe; set a flag instead
            // (handled in loop via gJustJoined)
        }
        return;
    }

    // ── Group-filtered packets ────────────────────────────────

    if (!gInGroup)                   return;
    if (pkt.groupID != gGroupID)     return;   // different group
    if (pkt.riderID == gMyRiderID)   return;   // own echo

    updateRiderTable(pkt.riderID, mac);

    if (pkt.type == PKT_AUDIO) {
        memcpy(&gRxPkt, &pkt, min(len, (int)sizeof(Packet)));
        gNewRxPkt = true;
        gLastRxMs = millis();
        if (gState == STATE_CONNECTED) gState = STATE_RX;
    }

    if (pkt.type == PKT_LEAVE) {
        for (int i = 0; i < MAX_RIDERS; i++) {
            if (gRiders[i].active && gRiders[i].riderID == pkt.riderID) {
                gRiders[i].active = false;
                gRiderCount--;
                Serial.printf("[GROUP] Rider %02X left (total %d)\n",
                              pkt.riderID, gRiderCount);
                break;
            }
        }
    }
}

void onSent(const uint8_t *mac, esp_now_send_status_t s) {}

// ──────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== ESP32 Moto Intercom – Multi-Rider ===");

    memset(gRiders, 0, sizeof(gRiders));

    pinMode(PTT_PIN,      INPUT_PULLUP);
    pinMode(PAIR_PIN,     INPUT_PULLUP);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN,   OUTPUT);
    pinMode(LED_BLUE_PIN,  OUTPUT);

    flashAll(3, 100, 80);   // boot indicator

    gPrefs.begin("intercom", false);
    loadGroup();

    setupI2SMic();
    setupI2SSpeaker();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    setupESPNow();

    if (gInGroup) {
        gState = STATE_CONNECTED;
        Serial.printf("Group: %08X  My RiderID: %02X\n", gGroupID, gMyRiderID);
        Serial.println("Ready.  Hold PAIR 3 s to add a rider.");
    } else {
        gState = STATE_IDLE;
        Serial.println("No group.  Hold PAIR 3 s to create/join.");
    }

    printStatus();
}

// ──────────────────────────────────────────────
//  MAIN LOOP
// ──────────────────────────────────────────────

// Flag set by RX callback when join succeeds (can't call flashAll in ISR context)
static volatile bool gJustJoined = false;

void loop() {
    // ── Serial debug ──────────────────────────────────────────
    if (Serial.available()) {
        switch (Serial.read()) {
            case 'p': printStatus();      break;
            case 'a': enterAddRiderMode();break;
            case 'j': enterJoinMode();    break;
            case 'r': factoryReset();     break;
        }
    }

    // ── Flash on successful join ──────────────────────────────
    if (gJustJoined) {
        gJustJoined = false;
        flashAll(5, 60, 60);
    }

    // ── PAIR button ───────────────────────────────────────────
    handlePairButton();

    // ── Pairing window timeout ────────────────────────────────
    if ((gState == STATE_ADD_RIDER || gState == STATE_JOINING) &&
         millis() > gPairWindowEnd) {
        Serial.println("[PAIR] Window expired.");
        gState = gInGroup ? STATE_CONNECTED : STATE_IDLE;
    }

    // ── RX silence timeout ────────────────────────────────────
    if (gState == STATE_RX && millis() - gLastRxMs > 500) {
        gState = STATE_CONNECTED;
        digitalWrite(LED_BLUE_PIN, LOW);
    }

    // ── Play incoming audio ───────────────────────────────────
    if (gNewRxPkt) {
        gNewRxPkt = false;
        if (gState == STATE_RX) rxAudio();
    }

    // ── Prune stale riders every 5 s ─────────────────────────
    static uint32_t lastPrune = 0;
    if (millis() - lastPrune > 5000) { pruneRiders(); lastPrune = millis(); }

    // ── Non-TX states: update LEDs and return ─────────────────
    if (gState == STATE_RX      ||
        gState == STATE_IDLE    ||
        gState == STATE_ADD_RIDER ||
        gState == STATE_JOINING) {
        updateLEDs();
        return;
    }

    // ── Read microphone ───────────────────────────────────────
    if (!readMic()) { updateLEDs(); return; }

    // ── VOX ───────────────────────────────────────────────────
    int  rms = rmsLevel(gMicBuf, MIC_SAMPLES);
    bool vox = false;
    if (rms > VOX_THRESHOLD)                         { gVoxHoldMs = millis(); vox = true; }
    else if (millis() - gVoxHoldMs < VOX_HOLD_MS)    { vox = true; }

    // ── Transmit ──────────────────────────────────────────────
    bool ptt = (digitalRead(PTT_PIN) == LOW);
    if (gInGroup && (ptt || vox)) {
        gState = STATE_TX;

        // Optional volume pot on ADC
        float gain = 1.0f;
#ifdef VOL_ADC_PIN
        gain = 0.2f + (analogRead(VOL_ADC_PIN) / 4095.0f) * 1.8f;
        applyVolume(gMicBuf, MIC_SAMPLES, gain);
#endif
        txAudio();
    } else {
        if (gState == STATE_TX) gState = STATE_CONNECTED;
        digitalWrite(LED_RED_PIN, LOW);
    }

    // ── Periodic ping ─────────────────────────────────────────
    static uint32_t lastPing = 0;
    if (gInGroup && millis() - lastPing > 3000) {
        Packet ping  = {};
        ping.type    = PKT_PING;
        ping.groupID = gGroupID;
        ping.riderID = gMyRiderID;
        broadcastPkt(&ping, PKT_HDR);
        lastPing = millis();
    }

    updateLEDs();
}

// ──────────────────────────────────────────────
//  PAIR BUTTON HANDLER
// ──────────────────────────────────────────────
void handlePairButton() {
    static uint32_t pressStart = 0;
    static bool     handled    = false;

    bool pressed = (digitalRead(PAIR_PIN) == LOW);

    if (pressed) {
        if (pressStart == 0) { pressStart = millis(); handled = false; }
        uint32_t held = millis() - pressStart;

        if (!handled && held >= RESET_HOLD_MS) {
            handled = true;
            factoryReset();
        } else if (!handled && held >= PAIR_HOLD_MS) {
            handled = true;
            if (!gInGroup) {
                // No group: create one and start advertising
                gGroupID   = makeGroupID();
                gMyRiderID = makeRiderID();
                gInGroup   = true;
                saveGroup();
                Serial.printf("[PAIR] New group %08X created (RiderID %02X)\n",
                              gGroupID, gMyRiderID);
                Serial.println("[PAIR] Other rider: hold PAIR 3 s to join.");
                enterAddRiderMode();
            } else {
                // Already in group: open add-rider window
                enterAddRiderMode();
            }
        }
    } else {
        pressStart = 0;
        handled    = false;
    }
}

// ──────────────────────────────────────────────
//  PAIRING MODES
// ──────────────────────────────────────────────
void enterAddRiderMode() {
    gState        = STATE_ADD_RIDER;
    gPairWindowEnd = millis() + ADD_RIDER_WINDOW;
    Serial.println("[PAIR] ADD-RIDER mode open for 30 s.");
    Serial.println("[PAIR] New rider: hold PAIR 3 s now.");
    flashAll(2, 120, 80);
}

void enterJoinMode() {
    gState        = STATE_JOINING;
    gPairWindowEnd = millis() + JOIN_WINDOW;
    Serial.println("[PAIR] JOIN mode: scanning for 30 s...");
    flashAll(2, 80, 80);
}

// Broadcasts group advertisement and drives join scanning LED
// Called every loop iteration from updateLEDs()
void pairingTick() {
    static uint32_t lastAdv = 0;
    if (millis() - lastAdv < ADV_INTERVAL_MS) return;
    lastAdv = millis();

    if (gState == STATE_ADD_RIDER && gInGroup) {
        Packet adv  = {};
        adv.type    = PKT_GROUP_ADV;
        adv.groupID = gGroupID;
        adv.riderID = gMyRiderID;
        broadcastPkt(&adv, PKT_HDR);
        Serial.print(".");
    }
    // STATE_JOINING: just wait; onReceive() handles the incoming adv
}

// ──────────────────────────────────────────────
//  FACTORY RESET
// ──────────────────────────────────────────────
void factoryReset() {
    Serial.println("[RESET] Factory reset!");

    // Announce to group before clearing
    if (gInGroup) {
        Packet bye  = {};
        bye.type    = PKT_LEAVE;
        bye.groupID = gGroupID;
        bye.riderID = gMyRiderID;
        broadcastPkt(&bye, PKT_HDR);
        delay(50);
    }

    gPrefs.clear();
    gGroupID    = 0;
    gMyRiderID  = makeRiderID();
    gInGroup    = false;
    gRiderCount = 0;
    memset(gRiders, 0, sizeof(gRiders));
    gState      = STATE_IDLE;

    flashAll(10, 40, 40);
    Serial.println("[RESET] Done. Hold PAIR 3 s to create a new group.");
}

// ──────────────────────────────────────────────
//  ESP-NOW
// ──────────────────────────────────────────────
void setupESPNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("FATAL: esp_now_init() failed");
        while (true) delay(1000);
    }
    esp_now_register_recv_cb(onReceive);
    esp_now_register_send_cb(onSent);

    // Broadcast peer – used for group audio and advertisements
    esp_now_peer_info_t bp = {};
    memcpy(bp.peer_addr, BCAST, 6);
    bp.channel = 0;
    bp.encrypt = false;
    esp_now_add_peer(&bp);

    Serial.println("ESP-NOW ready");
}

void broadcastPkt(Packet *p, int totalBytes) {
    esp_now_send(BCAST, (uint8_t *)p, totalBytes);
}

void unicastPkt(const uint8_t *mac, Packet *p, int totalBytes) {
    // Temporarily add unicast peer if not present
    esp_now_peer_info_t tmp = {};
    memcpy(tmp.peer_addr, mac, 6);
    tmp.channel = 0;
    tmp.encrypt = false;
    esp_now_add_peer(&tmp);   // silently fails if already exists
    esp_now_send(mac, (uint8_t *)p, totalBytes);
}

// ──────────────────────────────────────────────
//  I2S – MICROPHONE  (I2S_NUM_0, RX)
// ──────────────────────────────────────────────
void setupI2SMic() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = MIC_SCK_PIN,
        .ws_io_num    = MIC_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = MIC_SD_PIN
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_0, &pins));
    ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_0));
    Serial.println("Mic  I2S ready");
}

// ──────────────────────────────────────────────
//  I2S – SPEAKER  (I2S_NUM_1, TX)
// ──────────────────────────────────────────────
void setupI2SSpeaker() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = SPK_BCLK_PIN,
        .ws_io_num    = SPK_LRC_PIN,
        .data_out_num = SPK_DIN_PIN,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1, &pins));
    ESP_ERROR_CHECK(i2s_zero_dma_buffer(I2S_NUM_1));
    Serial.println("Spkr I2S ready");
}

// ──────────────────────────────────────────────
//  AUDIO TX
// ──────────────────────────────────────────────
void txAudio() {
    if (!gInGroup) return;
    const uint8_t *src  = (const uint8_t *)gMicBuf;
    int remaining       = MIC_SAMPLES * (int)sizeof(int16_t);
    int offset          = 0;

    while (remaining > 0) {
        int chunk       = min(remaining, (int)MAX_AUDIO_BYTES);
        gTxPkt.type     = PKT_AUDIO;
        gTxPkt.groupID  = gGroupID;
        gTxPkt.riderID  = gMyRiderID;
        gTxPkt.seq      = gTxSeq++;
        gTxPkt.len      = (uint16_t)chunk;
        memcpy(gTxPkt.data, src + offset, chunk);
        broadcastPkt(&gTxPkt, PKT_HDR + chunk);
        offset    += chunk;
        remaining -= chunk;
    }
    digitalWrite(LED_RED_PIN, HIGH);
}

// ──────────────────────────────────────────────
//  AUDIO RX
// ──────────────────────────────────────────────
void rxAudio() {
    if (gRxPkt.len == 0 || gRxPkt.len > MAX_AUDIO_BYTES) return;
    size_t written = 0;
    i2s_write(I2S_NUM_1, gRxPkt.data, gRxPkt.len,
              &written, pdMS_TO_TICKS(20));
    digitalWrite(LED_BLUE_PIN, HIGH);
}

// ──────────────────────────────────────────────
//  MIC READ  (32-bit I2S → 16-bit PCM)
// ──────────────────────────────────────────────
bool readMic() {
    int32_t raw[MIC_SAMPLES];
    size_t  bytesRead = 0;
    if (i2s_read(I2S_NUM_0, raw, sizeof(raw),
                 &bytesRead, pdMS_TO_TICKS(15)) != ESP_OK || bytesRead == 0)
        return false;
    int n = (int)(bytesRead / sizeof(int32_t));
    for (int i = 0; i < n; i++) {
        // INMP441: data in upper 24 bits → shift right 11 to get 16-bit range
        int32_t s = raw[i] >> 11;
        gMicBuf[i] = (int16_t)constrain(s, -32768, 32767);
    }
    return true;
}

// ──────────────────────────────────────────────
//  DSP HELPERS
// ──────────────────────────────────────────────
int rmsLevel(const int16_t *buf, int n) {
    int64_t sum = 0;
    for (int i = 0; i < n; i++) sum += (int64_t)buf[i] * buf[i];
    return (int)sqrtf((float)(sum / n));
}

void applyVolume(int16_t *buf, int n, float gain) {
    for (int i = 0; i < n; i++)
        buf[i] = (int16_t)constrain((int32_t)((float)buf[i] * gain), -32768, 32767);
}

// ──────────────────────────────────────────────
//  RIDER TABLE
// ──────────────────────────────────────────────
void updateRiderTable(uint8_t id, const uint8_t *mac) {
    for (int i = 0; i < MAX_RIDERS; i++) {
        if (gRiders[i].active && gRiders[i].riderID == id) {
            gRiders[i].lastHeardMs = millis();
            return;
        }
    }
    for (int i = 0; i < MAX_RIDERS; i++) {
        if (!gRiders[i].active) {
            gRiders[i].active      = true;
            gRiders[i].riderID     = id;
            gRiders[i].lastHeardMs = millis();
            memcpy(gRiders[i].mac, mac, 6);
            gRiderCount++;
            Serial.printf("[GROUP] Rider %02X online (total %d)\n", id, gRiderCount);
            return;
        }
    }
    Serial.println("[GROUP] Table full – cannot add more riders");
}

void pruneRiders() {
    for (int i = 0; i < MAX_RIDERS; i++) {
        if (gRiders[i].active &&
            millis() - gRiders[i].lastHeardMs > RIDER_TIMEOUT_MS) {
            Serial.printf("[GROUP] Rider %02X timed out\n", gRiders[i].riderID);
            gRiders[i].active = false;
            gRiderCount--;
        }
    }
}

void printStatus() {
    Serial.println("─────────────────────────────────────────");
    Serial.printf("State    : %d\n", (int)gState);
    Serial.printf("Group ID : %08X  %s\n", gGroupID, gInGroup ? "(joined)" : "(none)");
    Serial.printf("Rider ID : %02X\n", gMyRiderID);
    Serial.printf("Riders   : %d / %d\n", gRiderCount, MAX_RIDERS);
    for (int i = 0; i < MAX_RIDERS; i++) {
        if (!gRiders[i].active) continue;
        Serial.printf("  [%d] ID=%02X  MAC=%02X:%02X:%02X:%02X:%02X:%02X  age=%lus\n",
            i, gRiders[i].riderID,
            gRiders[i].mac[0], gRiders[i].mac[1], gRiders[i].mac[2],
            gRiders[i].mac[3], gRiders[i].mac[4], gRiders[i].mac[5],
            (millis() - gRiders[i].lastHeardMs) / 1000UL);
    }
    Serial.println("─────────────────────────────────────────");
}

// ──────────────────────────────────────────────
//  PERSISTENCE (NVS via Preferences)
// ──────────────────────────────────────────────
void saveGroup() {
    gPrefs.putBool("inGroup",  gInGroup);
    gPrefs.putUInt("groupID",  gGroupID);
    gPrefs.putUChar("riderID", gMyRiderID);
    Serial.printf("[NVS] Saved group=%08X riderID=%02X\n", gGroupID, gMyRiderID);
}

void loadGroup() {
    gInGroup   = gPrefs.getBool("inGroup",  false);
    gGroupID   = gPrefs.getUInt("groupID",  0);
    gMyRiderID = gPrefs.getUChar("riderID", 0);

    if (gInGroup && gGroupID != 0) {
        Serial.printf("[NVS] Loaded group=%08X riderID=%02X\n", gGroupID, gMyRiderID);
    } else {
        gInGroup   = false;
        gGroupID   = 0;
        gMyRiderID = makeRiderID();   // assign an ID even before joining
    }
}

// ──────────────────────────────────────────────
//  RANDOM ID GENERATORS  (use hardware RNG)
// ──────────────────────────────────────────────
uint32_t makeGroupID() {
    uint32_t id = esp_random();
    return (id == 0) ? 0xA1B2C3D4 : id;
}

uint8_t makeRiderID() {
    uint8_t id = (uint8_t)(esp_random() & 0xFF);
    return (id == 0) ? 0xA5 : id;
}

// ──────────────────────────────────────────────
//  LED MANAGER
// ──────────────────────────────────────────────
void flashAll(int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) {
        digitalWrite(LED_GREEN_PIN, HIGH);
        digitalWrite(LED_RED_PIN,   HIGH);
        digitalWrite(LED_BLUE_PIN,  HIGH);
        delay(onMs);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_RED_PIN,   LOW);
        digitalWrite(LED_BLUE_PIN,  LOW);
        delay(offMs);
    }
}

void updateLEDs() {
    // Drive pairing advertisement on every iteration
    if (gState == STATE_ADD_RIDER || gState == STATE_JOINING)
        pairingTick();

    uint32_t t = millis();
    switch (gState) {
        case STATE_IDLE:
            // Slow green blink – no group
            digitalWrite(LED_GREEN_PIN, (t / 800) % 2);
            digitalWrite(LED_RED_PIN,   LOW);
            digitalWrite(LED_BLUE_PIN,  LOW);
            break;

        case STATE_ADD_RIDER:
            // Green solid + red fast blink – advertising
            digitalWrite(LED_GREEN_PIN, HIGH);
            digitalWrite(LED_RED_PIN,   (t / 150) % 2);
            digitalWrite(LED_BLUE_PIN,  LOW);
            break;

        case STATE_JOINING:
            // Blue fast blink – scanning
            digitalWrite(LED_GREEN_PIN, LOW);
            digitalWrite(LED_RED_PIN,   LOW);
            digitalWrite(LED_BLUE_PIN,  (t / 150) % 2);
            break;

        case STATE_CONNECTED:
            // Green solid – idle in group
            digitalWrite(LED_GREEN_PIN, HIGH);
            digitalWrite(LED_RED_PIN,   LOW);
            digitalWrite(LED_BLUE_PIN,  LOW);
            break;

        case STATE_TX:
            // Green + red solid – transmitting
            digitalWrite(LED_GREEN_PIN, HIGH);
            digitalWrite(LED_RED_PIN,   HIGH);
            digitalWrite(LED_BLUE_PIN,  LOW);
            break;

        case STATE_RX:
            // Green + blue solid – receiving
            digitalWrite(LED_GREEN_PIN, HIGH);
            digitalWrite(LED_RED_PIN,   LOW);
            digitalWrite(LED_BLUE_PIN,  HIGH);
            break;
    }
}
