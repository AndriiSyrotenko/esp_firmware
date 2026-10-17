#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Preferences.h>
#include <DHT.h>
#include <esp_system.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

// =====================================================
// Build information
// =====================================================

#ifndef FW_VERSION
#define FW_VERSION "0.1.0"
#endif

#ifndef FW_GIT_HASH
#define FW_GIT_HASH "nogit"
#endif

// =====================================================
// Hardware: TTGO LoRa32 T3 v1.6.1 + SX1276
// =====================================================

// GPIO12 is a strapping pin (flash voltage). Use another pin for DHT
// (e.g. 32) if the sensor pulls the line high at boot.
#define DHT_PIN       13
#define DHT_TYPE      DHT11

#define LORA_SCK       5
#define LORA_MISO     19
#define LORA_MOSI     27
#define LORA_CS       18
#define LORA_RST      23
#define LORA_DIO0     26
#define LORA_DIO1     33

#define LORA_FREQ_MHZ 915.0

SPIClass loraSPI(VSPI);

SX1276 radio = new Module(
    LORA_CS,
    LORA_DIO0,
    LORA_RST,
    LORA_DIO1,
    loraSPI
);

DHT dht(DHT_PIN, DHT_TYPE);
Preferences prefs;

// =====================================================
// Configuration
// =====================================================

static constexpr uint8_t CFG_VERSION = 2;

struct Config {
    uint32_t version;
    uint32_t sequence;
    uint32_t telemetryIntervalSec;
    int8_t txPowerDbm;
    float dhtOffset;
    uint32_t crc;
};

struct ConfigV1 {
    uint32_t version;
    uint32_t sequence;
    uint32_t txIntervalMs;
    int8_t txPowerDbm;
    uint32_t crc;
};

Config config;

static constexpr uint32_t DEFAULT_INTERVAL_SEC = 10;
static constexpr int8_t DEFAULT_TX_POWER = 10;
static constexpr float DEFAULT_DHT_OFFSET = 0.0f;

bool radioReady = false;
bool sensorReady = false;
bool nvsReady = false;

// =====================================================
// POST failure bitmask
// Bit set = corresponding check failed
// =====================================================

enum PostBit : uint16_t {
    POST_NVS    = 1 << 0,
    POST_DHT    = 1 << 1,
    POST_RADIO  = 1 << 2,
    POST_CONFIG = 1 << 3
};

uint16_t postMask = 0;

// =====================================================
// Ring log
// =====================================================

enum LogLevel : uint8_t {
    LOG_ERROR = 0,
    LOG_WARN  = 1,
    LOG_INFO  = 2,
    LOG_DEBUG = 3
};

LogLevel currentLogLevel = LOG_INFO;

static constexpr size_t LOG_COUNT = 32;
static constexpr size_t LOG_LENGTH = 112;

char logRing[LOG_COUNT][LOG_LENGTH] = {};
size_t logWriteIndex = 0;
size_t logStored = 0;

const char* levelName(LogLevel level) {
    switch (level) {
        case LOG_ERROR: return "ERROR";
        case LOG_WARN:  return "WARN";
        case LOG_INFO:  return "INFO";
        case LOG_DEBUG: return "DEBUG";
        default:        return "?";
    }
}

void logMessage(LogLevel level, const char* message) {
    char line[LOG_LENGTH];

    snprintf(
        line,
        sizeof(line),
        "[%lu][%s] %s",
        (unsigned long)millis(),
        levelName(level),
        message
    );

    // Always retain messages in the ring.
    strlcpy(logRing[logWriteIndex], line, LOG_LENGTH);
    logWriteIndex = (logWriteIndex + 1) % LOG_COUNT;

    if (logStored < LOG_COUNT) {
        logStored++;
    }

    if (level <= currentLogLevel) {
        Serial.println(line);
    }
}

void showLogs() {
    size_t start = (logWriteIndex + LOG_COUNT - logStored)
                   % LOG_COUNT;

    for (size_t i = 0; i < logStored; i++) {
        Serial.println(logRing[(start + i) % LOG_COUNT]);
    }
}

// =====================================================
// CRC32 for configuration validation
// =====================================================

uint32_t calculateCRC(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (int bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

// Append a uint32_t in explicit little-endian order.
void appendU32LE(uint8_t* out, size_t& pos, uint32_t value) {
    out[pos++] = (uint8_t)(value);
    out[pos++] = (uint8_t)(value >> 8);
    out[pos++] = (uint8_t)(value >> 16);
    out[pos++] = (uint8_t)(value >> 24);
}

// New V2 CRC: version, sequence, interval, power, float bits.
uint32_t configCRC(const Config& cfg) {
    uint8_t bytes[17];
    size_t pos = 0;

    appendU32LE(bytes, pos, cfg.version);
    appendU32LE(bytes, pos, cfg.sequence);
    appendU32LE(bytes, pos, cfg.telemetryIntervalSec);

    bytes[pos++] = (uint8_t)cfg.txPowerDbm;

    uint32_t offsetBits = 0;
    static_assert(sizeof(float) == sizeof(uint32_t),
                  "Expected 32-bit float");
    memcpy(&offsetBits, &cfg.dhtOffset, sizeof(offsetBits));
    appendU32LE(bytes, pos, offsetBits);

    return calculateCRC(bytes, pos);
}

// New V1 CRC: version, sequence, interval in ms, power.
uint32_t configV1CRC(const ConfigV1& cfg) {
    uint8_t bytes[13];
    size_t pos = 0;

    appendU32LE(bytes, pos, cfg.version);
    appendU32LE(bytes, pos, cfg.sequence);
    appendU32LE(bytes, pos, cfg.txIntervalMs);

    bytes[pos++] = (uint8_t)cfg.txPowerDbm;

    return calculateCRC(bytes, pos);
}


bool configIsValid(const Config& cfg) {
    return cfg.version == CFG_VERSION
        && cfg.telemetryIntervalSec >= 2
        && cfg.telemetryIntervalSec <= 3600
        && cfg.txPowerDbm >= 2
        && cfg.txPowerDbm <= 17
        && isfinite(cfg.dhtOffset)
        && cfg.dhtOffset >= -10.0f
        && cfg.dhtOffset <= 10.0f
        && cfg.crc == configCRC(cfg);
}

// =====================================================
// Configuration persistence
//
// Two slots: A and B.
// Keep the previous valid slot until the new slot
// has been written and verified.
// =====================================================

bool readConfigSlot(const char* key, Config& result) {
    if (!prefs.isKey(key)) {
        return false;
    }

    if (prefs.getBytesLength(key) != sizeof(Config)) {
        return false;
    }

    Config candidate{};

    if (prefs.getBytes(key, &candidate, sizeof(candidate))
        != sizeof(candidate)) {
        return false;
    }

    if (!configIsValid(candidate)) {
        return false;
    }

    result = candidate;
    return true;
}

bool saveConfig() {
    Config next = config;

    next.version = CFG_VERSION;
    next.sequence = config.sequence + 1;
    next.crc = configCRC(next);

    Config a{}, b{};
    bool validA = readConfigSlot("cfgA", a);
    bool validB = readConfigSlot("cfgB", b);

    // Write to the older/invalid slot.
    const char* target;

    if (!validA) {
        target = "cfgA";
    } else if (!validB) {
        target = "cfgB";
    } else {
        target = (a.sequence <= b.sequence) ? "cfgA" : "cfgB";
    }

    size_t written = prefs.putBytes(target, &next, sizeof(next));

    if (written != sizeof(next)) {
        logMessage(LOG_ERROR, "Config write failed");
        return false;
    }

    Config verify{};

    if (!readConfigSlot(target, verify)
        || verify.sequence != next.sequence) {
        logMessage(LOG_ERROR, "Config readback/CRC failed");
        return false;
    }

    config = verify;
    logMessage(LOG_INFO, "Config saved and verified");
    return true;
}

void setDefaultConfig() {
    config.version = CFG_VERSION;
    config.sequence = 0;
    config.telemetryIntervalSec = DEFAULT_INTERVAL_SEC;
    config.txPowerDbm = DEFAULT_TX_POWER;
    config.dhtOffset = DEFAULT_DHT_OFFSET;
    config.crc = configCRC(config);
}

bool migrateLegacyConfig() {
    if (!prefs.isKey("cfg_v1")) {
        logMessage(LOG_INFO, "No legacy cfg_v1 key present");
        return false;
    }

    if (prefs.getBytesLength("cfg_v1") != sizeof(ConfigV1)) {
        logMessage(LOG_WARN, "Legacy config size invalid");
        return false;
    }

    ConfigV1 old{};

    if (prefs.getBytes("cfg_v1", &old, sizeof(old))
        != sizeof(old)) {
        logMessage(LOG_WARN, "Legacy config read failed");
        return false;
    }

    if (old.version != 1 || old.crc != configV1CRC(old)) {
        logMessage(LOG_WARN, "Legacy config CRC/version invalid");
        return false;
    }

    // Migration V1 -> V2:
    // milliseconds become seconds; new dhtOffset defaults to 0.
    config.version = CFG_VERSION;
    config.sequence = old.sequence;
    config.telemetryIntervalSec = old.txIntervalMs / 1000;

    if (config.telemetryIntervalSec < 2
        || config.telemetryIntervalSec > 3600) {
        config.telemetryIntervalSec = DEFAULT_INTERVAL_SEC;
    }

    config.txPowerDbm = old.txPowerDbm;
    config.dhtOffset = DEFAULT_DHT_OFFSET;
    config.crc = configCRC(config);

    if (!saveConfig()) {
        logMessage(LOG_ERROR, "Config migration save failed");
        return false;
    }

    prefs.remove("cfg_v1");
    logMessage(LOG_INFO, "Config migrated V1 -> V2");
    return true;
}

bool loadConfig() {
    Config a{}, b{};
    bool validA = readConfigSlot("cfgA", a);
    bool validB = readConfigSlot("cfgB", b);

    char message[LOG_LENGTH];

    snprintf(
        message,
        sizeof(message),
        "loadConfig: validA=%d validB=%d legacy=%d",
        validA,
        validB,
        prefs.isKey("cfg_v1")
    );
    logMessage(LOG_INFO, message);

    if (validA && validB) {
        config = (a.sequence >= b.sequence) ? a : b;
        logMessage(LOG_INFO, "loadConfig: using newer of slots A/B");
        return true;
    }

    if (validA) {
        config = a;
        logMessage(LOG_INFO, "loadConfig: using slot A");
        return true;
    }

    if (validB) {
        config = b;
        logMessage(LOG_INFO, "loadConfig: using slot B");
        return true;
    }

    if (migrateLegacyConfig()) {
        return true;
    }

    setDefaultConfig();

    logMessage(LOG_WARN, "No valid config; using defaults");

    return saveConfig();
}

// =====================================================
// Sensor and telemetry
// =====================================================

float lastTemperature = NAN;
float lastHumidity = NAN;
uint16_t telemetrySequence = 0;

// Binary telemetry frame, little-endian:
//
// Byte 0:     magic 0xA5
// Byte 1:     protocol version 1
// Byte 2:     message type 0x01 (telemetry)
// Byte 3-4:   sequence, uint16
// Byte 5-6:   POST failure mask, uint16
// Byte 7-8:   temperature x10, int16; 0x7FFF = invalid
// Byte 9-10:  humidity x10, uint16; 0xFFFF = invalid
// Byte 11:    config version
// Byte 12-13: CRC16-CCITT over bytes 0-11

uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000)
                ? (crc << 1) ^ 0x1021
                : (crc << 1);
        }
    }

    return crc;
}

void putU16LE(uint8_t* out, uint16_t value) {
    out[0] = value & 0xFF;
    out[1] = (value >> 8) & 0xFF;
}

void readSensor() {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature)) {
        sensorReady = false;
        logMessage(LOG_WARN, "DHT11 read failed");
        return;
    }

    lastHumidity = humidity;
    lastTemperature = temperature + config.dhtOffset;
    sensorReady = true;

    char message[LOG_LENGTH];

    snprintf(
        message,
        sizeof(message),
        "DHT11: %.1f C, %.1f %%RH",
        lastTemperature,
        lastHumidity
    );

    logMessage(LOG_INFO, message);
}

bool sendTelemetry() {
    uint8_t frame[14] = {};

    frame[0] = 0xA5;
    frame[1] = 1;
    frame[2] = 0x01;

    putU16LE(&frame[3], telemetrySequence++);
    putU16LE(&frame[5], postMask);

    int16_t temperature = 0x7FFF;
    uint16_t humidity = 0xFFFF;

    if (sensorReady) {
        temperature = (int16_t)lroundf(lastTemperature * 10.0f);
        humidity = (uint16_t)lroundf(lastHumidity * 10.0f);
    }

    putU16LE(&frame[7], (uint16_t)temperature);
    putU16LE(&frame[9], humidity);
    frame[11] = config.version;

    uint16_t crc = crc16(frame, 12);
    putU16LE(&frame[12], crc);

    if (!radioReady) {
        logMessage(LOG_ERROR, "Telemetry skipped: radio unavailable");
        return false;
    }

    int16_t state = radio.transmit(frame, sizeof(frame));

    if (state != RADIOLIB_ERR_NONE) {
        char message[LOG_LENGTH];

        snprintf(
            message,
            sizeof(message),
            "LoRa transmit failed: %d",
            state
        );

        logMessage(LOG_ERROR, message);
        return false;
    }

    logMessage(LOG_INFO, "Telemetry sent");
    return true;
}

// =====================================================
// POST self-test
// =====================================================

void runPOST() {
    postMask = 0;

    // NVS/config storage check.
    if (!nvsReady) {
        postMask |= POST_NVS;
    }

    // Sensor check: attempt a fresh measurement.
    readSensor();

    if (!sensorReady) {
        postMask |= POST_DHT;
    }

    // Radio check: successful radio initialization.
    // Actual RF reception requires a second receiver.
    if (!radioReady) {
        postMask |= POST_RADIO;
    }

    if (!configIsValid(config)) {
        postMask |= POST_CONFIG;
    }

    char message[LOG_LENGTH];

    snprintf(
        message,
        sizeof(message),
        "POST complete: failure_mask=0x%04X",
        postMask
    );

    logMessage(postMask ? LOG_WARN : LOG_INFO, message);

    // POST result is included in the next telemetry frame.
}

// =====================================================
// Serial commands
// =====================================================

void printVersion() {
    Serial.printf(
        "FW_VERSION=%s GIT_HASH=%s CFG_VERSION=%u\n",
        FW_VERSION,
        FW_GIT_HASH,
        config.version
    );
}

void printConfig() {
    Serial.printf("cfg_version=%lu\n", (unsigned long)config.version);
    Serial.printf("sequence=%lu\n", (unsigned long)config.sequence);
    Serial.printf("telemetry_interval_sec=%lu\n",
                  (unsigned long)config.telemetryIntervalSec);
    Serial.printf("tx_power_dbm=%d\n", config.txPowerDbm);
    Serial.printf("dht_offset=%.1f\n", config.dhtOffset);
}

// Diagnostic: show what is physically stored in NVS.
void printSlots() {
    Config a{}, b{};

    bool keyA = prefs.isKey("cfgA");
    bool keyB = prefs.isKey("cfgB");
    bool keyV1 = prefs.isKey("cfg_v1");

    bool validA = readConfigSlot("cfgA", a);
    bool validB = readConfigSlot("cfgB", b);

    Serial.printf("cfgA: key=%d valid=%d seq=%lu\n",
                  keyA, validA,
                  validA ? (unsigned long)a.sequence : 0UL);
    Serial.printf("cfgB: key=%d valid=%d seq=%lu\n",
                  keyB, validB,
                  validB ? (unsigned long)b.sequence : 0UL);
    Serial.printf("cfg_v1: key=%d\n", keyV1);
}

bool saveNewConfig(Config candidate) {
    candidate.version = CFG_VERSION;

    if (candidate.telemetryIntervalSec < 2
        || candidate.telemetryIntervalSec > 3600) {
        logMessage(LOG_WARN, "Interval range: 2..3600 sec");
        return false;
    }

    if (candidate.txPowerDbm < 2 || candidate.txPowerDbm > 17) {
        logMessage(LOG_WARN, "TX power range: 2..17 dBm");
        return false;
    }

    if (!isfinite(candidate.dhtOffset)
        || candidate.dhtOffset < -10.0f
        || candidate.dhtOffset > 10.0f) {
        logMessage(LOG_WARN, "DHT offset range: -10..10 C");
        return false;
    }

    Config old = config;
    config = candidate;

    if (!saveConfig()) {
        config = old;
        return false;
    }

    return true;
}

// Test only: write a V1 config with distinctive values,
// remove the V2 slots, verify the legacy blob, restart.
// Expected after reboot: sequence=42, interval=15, power=7.
void demoMigration() {
    ConfigV1 legacy;
    memset(&legacy, 0, sizeof(legacy));

    legacy.version = 1;
    legacy.sequence = 41;
    legacy.txIntervalMs = 15000;
    legacy.txPowerDbm = 7;
    legacy.crc = configV1CRC(legacy);

    // Remove V2 slots so the next boot has to use the legacy key.
    if (prefs.isKey("cfgA")) prefs.remove("cfgA");
    if (prefs.isKey("cfgB")) prefs.remove("cfgB");

    size_t written = prefs.putBytes("cfg_v1", &legacy, sizeof(legacy));

    ConfigV1 check;
    memset(&check, 0, sizeof(check));
    size_t got = prefs.getBytes("cfg_v1", &check, sizeof(check));

    bool ok = (written == sizeof(legacy))
           && (got == sizeof(check))
           && check.version == 1
           && check.crc == configV1CRC(check);

    if (!ok) {
        logMessage(LOG_ERROR, "Demo aborted: cfg_v1 write/readback failed");
        return;
    }

    logMessage(LOG_WARN, "Migration demo prepared; restarting");
    Serial.flush();
    prefs.end();
    delay(300);
    ESP.restart();
}

// Test only: write config back-to-back so a power cut lands inside a write.
void configStress(unsigned count) {
    logMessage(LOG_WARN, "Stress start: pull power at any time");

    for (unsigned i = 0; i < count; i++) {
        Config c = config;
        uint32_t nextSeq = config.sequence + 1;

        // Interval is derived from sequence, so consistency is checkable.
        c.telemetryIntervalSec = 10 + (nextSeq % 50);

        Serial.printf("[stress] writing seq=%lu\n", (unsigned long)nextSeq);

        if (!saveNewConfig(c)) {
            Serial.println("[stress] write FAILED");
            return;
        }

        Serial.printf("[stress] ok seq=%lu interval=%lu\n",
                      (unsigned long)config.sequence,
                      (unsigned long)config.telemetryIntervalSec);
    }

    Serial.println("[stress] done");
}

void handleCommand(String command) {
    command.trim();

    if (command == "version") {
        printVersion();
    }
    else if (command == "config get") {
        printConfig();
    }
    else if (command == "config slots") {
        printSlots();
    }
    else if (command == "config reset") {
        Config old = config;
        setDefaultConfig();

        if (!saveConfig()) {
            config = old;
            Serial.println("ERROR: reset save failed");
        } else {
            Serial.println("OK: defaults saved");
        }
    }
    else if (command.startsWith("config stress ")) {
        long n = command.substring(14).toInt();
        if (n < 1) n = 1;
        if (n > 500) n = 500;
        configStress((unsigned)n);
    }
    else if (command == "post") {
        runPOST();
        Serial.printf("POST failure mask: 0x%04X\n", postMask);
    }
    else if (command == "telemetry") {
        readSensor();
        sendTelemetry();
    }
    else if (command == "config demo_migration") {
        demoMigration();
    }
    else if (command == "log show") {
        showLogs();
    }
    else if (command.startsWith("log level ")) {
        String value = command.substring(10);
        value.trim();
        value.toUpperCase();

        if (value == "ERROR") currentLogLevel = LOG_ERROR;
        else if (value == "WARN") currentLogLevel = LOG_WARN;
        else if (value == "INFO") currentLogLevel = LOG_INFO;
        else if (value == "DEBUG") currentLogLevel = LOG_DEBUG;
        else {
            Serial.println("Usage: log level ERROR|WARN|INFO|DEBUG");
            return;
        }

        Serial.printf("Log level set to %s\n", value.c_str());
    }
    else if (command.startsWith("config set ")) {
        String args = command.substring(11);
        args.trim();

        int separator = args.indexOf(' ');

        if (separator < 0) {
            Serial.println("Usage: config set interval|power|offset VALUE");
            return;
        }

        String key = args.substring(0, separator);
        String value = args.substring(separator + 1);
        value.trim();

        Config candidate = config;

        if (key == "interval") {
            char* end = nullptr;
            unsigned long number = strtoul(value.c_str(), &end, 10);

            if (end == value.c_str() || *end != '\0') {
                Serial.println("ERROR: invalid interval");
                return;
            }

            candidate.telemetryIntervalSec = number;
        }
        else if (key == "power") {
            char* end = nullptr;
            long number = strtol(value.c_str(), &end, 10);

            if (end == value.c_str() || *end != '\0'
                || number < -128 || number > 127) {
                Serial.println("ERROR: invalid power");
                return;
            }

            candidate.txPowerDbm = (int8_t)number;
        }
        else if (key == "offset") {
            char* end = nullptr;
            float number = strtof(value.c_str(), &end);

            if (end == value.c_str() || *end != '\0'
                || !isfinite(number)) {
                Serial.println("ERROR: invalid offset");
                return;
            }

            candidate.dhtOffset = number;
        }
        else {
            Serial.println("Unknown config key");
            return;
        }

        if (saveNewConfig(candidate)) {
            Serial.println("OK: config saved");
        } else {
            Serial.println("ERROR: config rejected/not saved");
        }
    }
    else if (command == "help") {
        Serial.println("version");
        Serial.println("config get");
        Serial.println("config slots");
        Serial.println("config set interval <2..3600>");
        Serial.println("config set power <2..17>");
        Serial.println("config set offset <-10..10>");
        Serial.println("config reset");
        Serial.println("config demo_migration");
        Serial.println("post");
        Serial.println("telemetry");
        Serial.println("log level ERROR|WARN|INFO|DEBUG");
        Serial.println("log show");
    }
    else {
        Serial.println("Unknown command. Type help");
    }
}

// =====================================================
// FreeRTOS tasks
// =====================================================

void sensorTask(void* parameter) {
    (void)parameter;

    for (;;) {
        readSensor();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void telemetryTask(void* parameter) {
    (void)parameter;

    for (;;) {
        uint32_t interval = config.telemetryIntervalSec;

        // Do not send faster than the DHT11's update rate.
        if (interval < 2) interval = 2;

        vTaskDelay(pdMS_TO_TICKS(interval * 1000UL));

        sendTelemetry();
    }
}

void serialTask(void* parameter) {
    (void)parameter;

    String input;

    for (;;) {
        while (Serial.available()) {
            char c = (char)Serial.read();

            if (c == '\r') continue;

            if (c == '\n') {
                if (input.length() > 0) {
                    handleCommand(input);
                    input = "";
                }
            } else if (input.length() < 120) {
                input += c;
            } else {
                input = "";
                Serial.println("ERROR: command too long");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =====================================================
// Setup / loop
// =====================================================

void setup() {
    Serial.begin(115200);
    delay(500);

    logMessage(LOG_INFO, "Booting telemetry node");

    dht.begin();

    prefs.begin("telemetry", false);
    nvsReady = true;

    if (!loadConfig()) {
        logMessage(LOG_ERROR, "Failed to initialize config");
    }

    printVersion();

    loraSPI.begin(
        LORA_SCK,
        LORA_MISO,
        LORA_MOSI,
        LORA_CS
    );

    int16_t radioState = radio.begin(LORA_FREQ_MHZ);

    if (radioState == RADIOLIB_ERR_NONE) {
        radioReady = true;
        radio.setOutputPower(config.txPowerDbm);
        logMessage(LOG_INFO, "SX1276 initialized");
    } else {
        char message[LOG_LENGTH];

        snprintf(
            message,
            sizeof(message),
            "SX1276 init failed: %d",
            radioState
        );

        logMessage(LOG_ERROR, message);
    }

    runPOST();

    xTaskCreate(
        sensorTask,
        "SensorTask",
        4096,
        nullptr,
        1,
        nullptr
    );

    xTaskCreate(
        telemetryTask,
        "TelemetryTask",
        4096,
        nullptr,
        1,
        nullptr
    );

    xTaskCreate(
        serialTask,
        "SerialTask",
        4096,
        nullptr,
        1,
        nullptr
    );

    Serial.println("Ready. Type help");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}