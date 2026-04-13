/*
 * ============================================================
 *  ESP32 Power Monitor
 *  Hardware : ESP32 + PZEM-004T v3.0 + SD Card Module
 *  Author   : Aviral Dubey (@aviraldb)
 *  Framework: Arduino (ESP32 Arduino Core)
 *
 *  REQUIRED LIBRARIES (install via Arduino Library Manager):
 *    - PZEM004Tv30  by olehs   (search "PZEM004T")
 *    - SD / FS / WiFi / WebServer — built into ESP32 Arduino core
 *    - ESPmDNS                  — built into ESP32 Arduino core
 *    - esp_task_wdt.h           — built into ESP32 Arduino core
 *
 *  SECURITY NOTE:
 *    Designed for LAN use only — do NOT expose to the internet.
 *    HTTP only (no HTTPS); no login on the dashboard.
 *
 *  WIRING:
 *  ┌─────────────┬──────────┬──────────────────────────────┐
 *  │ PZEM TX     │ GPIO16   │ Via level shifter (5V→3.3V)  │
 *  │ PZEM RX     │ GPIO17   │ Direct OK                    │
 *  │ PZEM VCC    │ 5V/VIN   │ NOT 3.3V                     │
 *  │ PZEM GND    │ GND      │                              │
 *  ├─────────────┼──────────┼──────────────────────────────┤
 *  │ SD CS       │ GPIO5    │                              │
 *  │ SD MOSI     │ GPIO23   │                              │
 *  │ SD CLK      │ GPIO18   │                              │
 *  │ SD MISO     │ GPIO19   │                              │
 *  │ SD VCC      │ 3.3V     │                              │
 *  └─────────────┴──────────┴──────────────────────────────┘
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <PZEM004Tv30.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>
#include <math.h>
#include <esp_task_wdt.h>

// Generic value defination 
#define GENERIC_PZEM

// ─── WiFi Config ─────────────────────────────────────────────
const char* WIFI_SSID     = "aviraldubey";
const char* WIFI_PASSWORD = "hotspot7";

// AP fallback — used when STA connection fails
const char* AP_SSID       = "PowerMonitor-AP";
const char* AP_PASS       = "esp32power";

// mDNS hostname — accessible as http://powermonitor.local
const char* MDNS_HOST     = "powermonitor";

// ─── NTP ─────────────────────────────────────────────────────
const char* NTP_SERVER    = "pool.ntp.org";
const long  GMT_OFFSET_S  = 19800;    // UTC+5:30 India
const int   DAYLIGHT_S    = 0;

// ─── Pin Definitions ─────────────────────────────────────────
#define PZEM_RX_PIN     16
#define PZEM_TX_PIN     17
#define SD_CS_PIN        5

// ─── Timing ──────────────────────────────────────────────────
#define READ_INTERVAL_MS      5000     // sensor read period
#define LOG_INTERVAL_MS     300000     // SD log period (5 min)
#define WIFI_CHECK_MS        30000     // WiFi health check
#define SD_RETRY_MS          15000     // retry failed SD init
#define NTP_RESYNC_MS      3600000     // NTP re-sync every 1 hr
#define WDT_TIMEOUT_S          120     // watchdog timeout (s)

// ─── Mock / Demo Mode ────────────────────────────────────────
// Uncomment to bypass the real PZEM and use simulated values.
// Comment out once PZEM hardware is confirmed working.
//#define GENERIC_PZEM

// ─── Data Validation Limits (India 230V grid) ────────────────
#define V_MIN    170.0f
#define V_MAX    270.0f
#define I_MIN      0.0f
#define I_MAX    100.0f
#define P_MIN      0.0f
#define P_MAX  23000.0f
#define F_MIN     45.0f
#define F_MAX     55.0f
#define PF_MIN     0.0f
#define PF_MAX     1.0f

// ─── RAM Ring Buffer ─────────────────────────────────────────
#define BUFFER_SIZE   20

// ─── SD ──────────────────────────────────────────────────────
#define LOG_FILENAME   "/log_file.csv"
#define SD_FLUSH_N      5

// ─── Logging ─────────────────────────────────────────────────
#define LOG(fmt, ...)  Serial.printf(fmt "\n", ##__VA_ARGS__)

// ─────────────────────────────────────────────────────────────
//  Data Structures
// ─────────────────────────────────────────────────────────────

struct PowerData {
    float    voltage;
    float    current;
    float    power;
    float    energy;
    float    frequency;
    float    pf;
    float    apparent;
    float    reactive;
    float    qFactor;
    bool     valid;
    uint32_t timestamp;
};

struct RingBuffer {
    PowerData entries[BUFFER_SIZE];
    uint8_t   head;
    uint8_t   count;
} ringBuf = {{}, 0, 0};

PowerData latest = {0};

// ─────────────────────────────────────────────────────────────
//  State
// ─────────────────────────────────────────────────────────────

enum NetMode_t { NET_STA_OK, NET_AP_MODE, NET_NONE };

PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN, 0x01);
WebServer    server(80);

bool         sdAvailable    = false;
NetMode_t    netMode        = NET_NONE;
uint32_t     lastReadMs     = 0;
uint32_t     lastLogMs      = 0;
uint32_t     lastWifiMs     = 0;
uint32_t     lastSdRetryMs  = 0;
uint32_t     lastNtpMs      = 0;
uint32_t     logWriteCount  = 0;
uint8_t      pzemFailCount  = 0;
uint32_t     totalReadCount = 0;
uint32_t     totalFailCount = 0;
bool         ntpSynced      = false;

#define PZEM_FAIL_REBOOT  10

// ─────────────────────────────────────────────────────────────
//  Watchdog
// ─────────────────────────────────────────────────────────────

void initWatchdog() {
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = (uint32_t)(WDT_TIMEOUT_S) * 1000U,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    esp_task_wdt_reconfigure(&wdt_cfg);
    esp_task_wdt_add(NULL);
    LOG("[WDT] Watchdog armed (%ds)", WDT_TIMEOUT_S);
}

inline void feedWatchdog() { esp_task_wdt_reset(); }

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────

void getTimestamp(char* buf, size_t len) {
    struct tm ti;
    if (!getLocalTime(&ti)) { snprintf(buf, len, "NO_TIME"); return; }
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", &ti);
}

void computeDerived(PowerData& d) {
    d.apparent = d.voltage * d.current;
    float s2   = d.apparent * d.apparent;
    float p2   = d.power    * d.power;
    d.reactive = (s2 >= p2) ? sqrtf(s2 - p2) : 0.0f;
    d.qFactor  = (d.power > 0.01f) ? (d.reactive / d.power) : 0.0f;
}

void pushBuffer(const PowerData& d) {
    ringBuf.entries[ringBuf.head] = d;
    ringBuf.head = (ringBuf.head + 1) % BUFFER_SIZE;
    if (ringBuf.count < BUFFER_SIZE) ringBuf.count++;
}

// ─────────────────────────────────────────────────────────────
//  NTP
// ─────────────────────────────────────────────────────────────

void syncNTP() {
    if (netMode != NET_STA_OK) return;
    configTime(GMT_OFFSET_S, DAYLIGHT_S, NTP_SERVER);
    struct tm ti;
    uint32_t t0 = millis();
    while (!getLocalTime(&ti) && millis() - t0 < 5000) {
        feedWatchdog();
        delay(200);
    }
    if (getLocalTime(&ti)) {
        char ts[32];
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &ti);
        LOG("[NTP] Synced — %s", ts);
        ntpSynced = true;
    } else {
        LOG("[NTP] Sync timed out");
        ntpSynced = false;
    }
    lastNtpMs = millis();
}

// ─────────────────────────────────────────────────────────────
//  Sensor Reading
// ─────────────────────────────────────────────────────────────

#ifdef GENERIC_PZEM

static float _mock_energy = 1.234f;

static float mockVal(float base, float range) {
    uint32_t t    = millis() / 1000;
    float    phi  = (float)(t % 60) / 60.0f;
    float    wave = (phi < 0.5f) ? (phi * 2.0f) : (2.0f - phi * 2.0f);
    return base - range + wave * 2.0f * range;
}

void readSensor() {
    totalReadCount++;
    PowerData d;
    d.timestamp = millis();
    d.voltage   = mockVal(220.0f,  4.0f);
    d.current   = mockVal(  2.36f, 0.15f);
    d.power     = mockVal(480.0f, 20.0f);
    d.frequency = mockVal( 49.9f,  0.15f);
    d.pf        = mockVal(  0.92f, 0.03f);
    _mock_energy += (d.power / 1000.0f) * ((float)READ_INTERVAL_MS / 3600000.0f);
    d.energy = _mock_energy;
    d.valid  = true;
    computeDerived(d);
    latest = d;
    pushBuffer(d);

    char ts[32]; getTimestamp(ts, sizeof(ts));
    Serial.println(F("──────────────────────────────────"));
    Serial.printf("  Time        : %s\n", ts);
    Serial.printf("  Voltage     : %7.2f V\n",   d.voltage);
    Serial.printf("  Current     : %7.3f A\n",   d.current);
    Serial.printf("  Active P    : %7.2f W\n",   d.power);
    Serial.printf("  Apparent S  : %7.2f VA\n",  d.apparent);
    Serial.printf("  Reactive Q  : %7.2f VAR\n", d.reactive);
    Serial.printf("  Q Factor    : %7.4f\n",      d.qFactor);
    Serial.printf("  Power Factor: %7.3f\n",      d.pf);
    Serial.printf("  Frequency   : %7.2f Hz\n",  d.frequency);
    Serial.printf("  Energy      : %7.3f kWh\n", d.energy);
    Serial.println(F("──────────────────────────────────"));
}

#else  // real PZEM

bool readOnce(PowerData& d) {
    d.timestamp = millis();
    d.voltage   = pzem.voltage();
    d.current   = pzem.current();
    d.power     = pzem.power();
    d.energy    = pzem.energy();
    d.frequency = pzem.frequency();
    d.pf        = pzem.pf();

    if (isnan(d.voltage) || isnan(d.current) || isnan(d.power) ||
        isnan(d.energy)  || isnan(d.frequency)|| isnan(d.pf)) {
        LOG("[PZEM] NaN in reading");
        return false;
    }
    if (d.voltage   < V_MIN  || d.voltage   > V_MAX)  { LOG("[PZEM] Voltage out of range: %.2f",   d.voltage);   return false; }
    if (d.current   < I_MIN  || d.current   > I_MAX)  { LOG("[PZEM] Current out of range: %.3f",   d.current);   return false; }
    if (d.power     < P_MIN  || d.power     > P_MAX)  { LOG("[PZEM] Power out of range: %.2f",     d.power);     return false; }
    if (d.frequency < F_MIN  || d.frequency > F_MAX)  { LOG("[PZEM] Frequency out of range: %.2f", d.frequency); return false; }
    if (d.pf        < PF_MIN || d.pf        > PF_MAX) { LOG("[PZEM] PF out of range: %.3f",        d.pf);        return false; }
    return true;
}

void readSensor() {
    PowerData d;
    bool      ok        = false;
    uint32_t  readStart = millis();

    for (uint8_t attempt = 1; attempt <= 3; attempt++) {
        ok = readOnce(d);
        if (ok) break;
        LOG("[PZEM] Attempt %u/3 failed", attempt);
        delay(200);
    }

    uint32_t readMs = millis() - readStart;
    totalReadCount++;

    if (!ok) {
        pzemFailCount++;
        totalFailCount++;
        latest.valid = false;
        LOG("[PZEM] Read failed (consecutive: %u  total: %lu  took: %lums)",
            pzemFailCount, totalFailCount, readMs);
        if (pzemFailCount >= PZEM_FAIL_REBOOT) {
            LOG("[PZEM] %u consecutive failures — rebooting", PZEM_FAIL_REBOOT);
            SD.end();
            delay(200);
            ESP.restart();
        }
        return;
    }

    pzemFailCount = 0;
    d.valid = true;
    computeDerived(d);
    latest = d;
    pushBuffer(d);

    char ts[32]; getTimestamp(ts, sizeof(ts));
    Serial.println(F("──────────────────────────────────"));
    Serial.printf("  Time        : %s\n", ts);
    Serial.printf("  Voltage     : %7.2f V\n",   d.voltage);
    Serial.printf("  Current     : %7.3f A\n",   d.current);
    Serial.printf("  Active P    : %7.2f W\n",   d.power);
    Serial.printf("  Apparent S  : %7.2f VA\n",  d.apparent);
    Serial.printf("  Reactive Q  : %7.2f VAR\n", d.reactive);
    Serial.printf("  Q Factor    : %7.4f\n",      d.qFactor);
    Serial.printf("  Power Factor: %7.3f\n",      d.pf);
    Serial.printf("  Frequency   : %7.2f Hz\n",  d.frequency);
    Serial.printf("  Energy      : %7.3f kWh\n", d.energy);
    Serial.printf("  Read time   : %lu ms\n",     readMs);
    Serial.printf("  OK/Total    : %lu/%lu\n",
        totalReadCount - totalFailCount, totalReadCount);
    Serial.println(F("──────────────────────────────────"));
}

#endif  // GENERIC_PZEM

// ─────────────────────────────────────────────────────────────
//  SD Card
// ─────────────────────────────────────────────────────────────

void initSD() {
    SD.end();
    if (!SD.begin(SD_CS_PIN)) {
        LOG("[SD] Mount failed — will retry in %ds", SD_RETRY_MS / 1000);
        sdAvailable = false;
        return;
    }
    sdAvailable = true;

    uint8_t     cardType = SD.cardType();
    uint64_t    cardSize = SD.cardSize() / (1024 * 1024);
    const char* typeStr  = (cardType == CARD_MMC)  ? "MMC"  :
                           (cardType == CARD_SD)   ? "SDSC" :
                           (cardType == CARD_SDHC) ? "SDHC" : "UNKNOWN";
    LOG("[SD] Mounted — type: %s  size: %lluMB", typeStr, cardSize);

    if (!SD.exists(LOG_FILENAME)) {
        File f = SD.open(LOG_FILENAME, FILE_WRITE);
        if (f) {
            f.println("timestamp,voltage_V,current_A,active_power_W,"
                      "apparent_power_VA,reactive_power_VAR,q_factor,"
                      "power_factor,frequency_Hz,energy_kWh");
            f.close();
            LOG("[SD] Log file created: %s", LOG_FILENAME);
        }
    } else {
        File f = SD.open(LOG_FILENAME, FILE_READ);
        if (f) {
            LOG("[SD] Log file exists — %lu bytes  ~%lu rows",
                (unsigned long)f.size(), (unsigned long)(f.size() / 80));
            f.close();
        }
    }
}

bool writeRow(File& f, const PowerData& d) {
    char ts[32];
    getTimestamp(ts, sizeof(ts));
    char row[160];
    snprintf(row, sizeof(row),
        "%s,%.2f,%.3f,%.2f,%.2f,%.2f,%.4f,%.3f,%.2f,%.3f\n",
        ts,
        d.voltage, d.current, d.power,
        d.apparent, d.reactive, d.qFactor,
        d.pf, d.frequency, d.energy);
    return (f.print(row) > 0);
}

void logToSD() {
    if (!latest.valid) return;

    if (!sdAvailable) {
        uint32_t now = millis();
        if (now - lastSdRetryMs >= SD_RETRY_MS) {
            lastSdRetryMs = now;
            LOG("[SD] Retrying mount...");
            initSD();
        }
        if (!sdAvailable) {
            LOG("[SD] Unavailable — %u readings in RAM buffer", ringBuf.count);
            return;
        }
    }

    File f = SD.open(LOG_FILENAME, FILE_APPEND);
    if (!f) {
        LOG("[SD] Open failed — marking unavailable");
        sdAvailable = false;
        return;
    }

    uint8_t flushed = 0;
    if (ringBuf.count > 1) {
        uint8_t oldest  = (ringBuf.head + BUFFER_SIZE - ringBuf.count) % BUFFER_SIZE;
        uint8_t toFlush = ringBuf.count - 1;
        for (uint8_t i = 0; i < toFlush; i++) {
            writeRow(f, ringBuf.entries[(oldest + i) % BUFFER_SIZE]);
            flushed++;
        }
        ringBuf.count = 1;
        LOG("[SD] Flushed %u buffered rows", flushed);
    }

    bool ok = writeRow(f, latest);
    logWriteCount++;
    if (logWriteCount % SD_FLUSH_N == 0) f.flush();
    f.close();

    if (ok) {
        char ts[32]; getTimestamp(ts, sizeof(ts));
        LOG("[SD] Row #%lu logged at %s", logWriteCount, ts);
    } else {
        LOG("[SD] Write error — marking unavailable");
        sdAvailable = false;
    }
}

// ─────────────────────────────────────────────────────────────
//  WiFi — STA with AP Fallback
// ─────────────────────────────────────────────────────────────

void startAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    netMode = NET_AP_MODE;
    LOG("[WiFi] AP mode — SSID: '%s'  IP: http://%s",
        AP_SSID, WiFi.softAPIP().toString().c_str());
}

void setupWiFi() {
    LOG("[WiFi] Connecting to '%s'", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        feedWatchdog();
        delay(300);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        netMode = NET_STA_OK;
        LOG("[WiFi] Connected — IP: http://%s  RSSI: %d dBm",
            WiFi.localIP().toString().c_str(), WiFi.RSSI());
        if (MDNS.begin(MDNS_HOST)) {
            MDNS.addService("http", "tcp", 80);
            LOG("[mDNS] http://%s.local", MDNS_HOST);
        }
        syncNTP();
    } else {
        LOG("[WiFi] STA failed — starting AP");
        startAP();
    }
}

void checkWiFi() {
    if (netMode == NET_AP_MODE) return;
    if (WiFi.status() != WL_CONNECTED) {
        LOG("[WiFi] Lost connection — reconnecting...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
            feedWatchdog();
            delay(300);
        }
        if (WiFi.status() == WL_CONNECTED) {
            netMode = NET_STA_OK;
            LOG("[WiFi] Reconnected — IP: %s", WiFi.localIP().toString().c_str());
        } else {
            LOG("[WiFi] Reconnect failed — switching to AP");
            startAP();
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  Web Dashboard — HTML in PROGMEM
// ─────────────────────────────────────────────────────────────

static const char HTML_HEAD[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Power Monitor</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',Arial,sans-serif;background:#0f172a;color:#e2e8f0;min-height:100vh;padding:16px}
h1{text-align:center;font-size:1.4rem;font-weight:600;color:#38bdf8;margin:16px 0 24px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:12px;max-width:720px;margin:0 auto 20px}
.card{background:#1e293b;border:1px solid #334155;border-radius:10px;padding:14px 12px;text-align:center}
.card .label{font-size:11px;color:#94a3b8;text-transform:uppercase;letter-spacing:.05em;margin-bottom:6px}
.card .value{font-size:1.6rem;font-weight:700;color:#f8fafc;line-height:1}
.card .unit{font-size:12px;color:#64748b;margin-top:4px}
.card.warn .value{color:#fb923c}.card.good .value{color:#34d399}
.status{max-width:720px;margin:0 auto 8px;display:flex;gap:12px;flex-wrap:wrap;justify-content:center;font-size:12px;color:#64748b}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px}
.dot.ok{background:#34d399}.dot.err{background:#f87171}.dot.warn{background:#fb923c}
#ts{text-align:center;font-size:11px;color:#475569;margin-top:8px}
.banner{display:none;max-width:720px;margin:0 auto 16px;border-radius:8px;padding:10px;text-align:center;font-size:13px}
.banner.err{background:#7f1d1d;color:#fca5a5}.banner.warn{background:#78350f;color:#fcd34d}
.banner.mock{background:#1e3a5f;color:#7dd3fc;border:1px solid #3b82f6}
.btns{text-align:center;margin-top:12px;display:flex;gap:8px;justify-content:center;flex-wrap:wrap}
button{padding:6px 18px;border-radius:6px;border:1px solid #334155;background:#1e293b;color:#94a3b8;cursor:pointer;font-size:12px}
button:hover{background:#334155}
</style></head><body>
<h1>&#9889; ESP32 Power Monitor</h1>
<div class="banner err" id="err">Sensor read error &mdash; check PZEM wiring</div>
<div class="banner mock" id="mock-banner" style="display:none">&#128268; SIMULATED DATA &mdash; GENERIC_PZEM mode active</div>
<div class="banner warn" id="warn"></div>
<div class="grid">
<div class="card"><div class="label">Voltage</div><div class="value" id="v">&#8212;</div><div class="unit">V</div></div>
<div class="card"><div class="label">Current</div><div class="value" id="i">&#8212;</div><div class="unit">A</div></div>
<div class="card good"><div class="label">Active Power</div><div class="value" id="p">&#8212;</div><div class="unit">W</div></div>
<div class="card"><div class="label">Apparent Power</div><div class="value" id="s">&#8212;</div><div class="unit">VA</div></div>
<div class="card warn"><div class="label">Reactive Power</div><div class="value" id="q">&#8212;</div><div class="unit">VAR</div></div>
<div class="card"><div class="label">Q Factor</div><div class="value" id="qf">&#8212;</div><div class="unit">tan &phi;</div></div>
<div class="card"><div class="label">Power Factor</div><div class="value" id="pf">&#8212;</div><div class="unit">cos &phi;</div></div>
<div class="card"><div class="label">Frequency</div><div class="value" id="hz">&#8212;</div><div class="unit">Hz</div></div>
<div class="card"><div class="label">Energy</div><div class="value" id="en">&#8212;</div><div class="unit">kWh</div></div>
<div class="card"><div class="label">Uptime</div><div class="value" id="up">&#8212;</div><div class="unit">s</div></div>
</div>
<div class="status">
<span><span class="dot" id="sd-dot"></span>SD: <span id="sd-st">&#8212;</span></span>
<span>Rows: <span id="logs">&#8212;</span></span>
<span>Buffer: <span id="buf">&#8212;</span></span>
<span>Fails: <span id="fail">&#8212;</span></span>
<span>Total: <span id="tfail">&#8212;</span></span>
<span><span class="dot" id="net-dot"></span><span id="net-st">&#8212;</span></span>
<span>RSSI: <span id="rssi">&#8212;</span> dBm</span>
<span>NTP: <span id="ntp">&#8212;</span></span>
</div>
<div class="btns">
<button onclick="fetch('/reset').then(r=>r.text()).then(t=>alert(t))">Reset Energy Counter</button>
<button onclick="fetch('/export').then(r=>r.blob()).then(b=>{const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download='powerlog.csv';a.click()})">Download CSV</button>
</div>
<div id="ts">Updating...</div>
)HTML";

static const char HTML_SCRIPT[] PROGMEM = R"HTML(<script>
const BUFSIZE=20;
async function update(){
  try{
    const r=await fetch('/data');
    if(!r.ok)throw new Error('HTTP '+r.status);
    const d=await r.json();
    const E=id=>document.getElementById(id);
    if(d.error){E('err').style.display='block';return;}
    E('err').style.display='none';
    E('v').textContent=d.voltage.toFixed(1);
    E('i').textContent=d.current.toFixed(3);
    E('p').textContent=d.power.toFixed(1);
    E('s').textContent=d.apparent.toFixed(1);
    E('q').textContent=d.reactive.toFixed(1);
    E('qf').textContent=d.qfactor.toFixed(3);
    E('pf').textContent=d.pf.toFixed(3);
    E('hz').textContent=d.frequency.toFixed(2);
    E('en').textContent=d.energy.toFixed(3);
    E('up').textContent=d.uptime;
    E('sd-dot').className='dot '+(d.sd?'ok':'err');
    E('sd-st').textContent=d.sd?'OK':'Unavailable';
    E('logs').textContent=d.logs;
    E('buf').textContent=d.buf+'/'+BUFSIZE;
    E('fail').textContent=d.fails;
    E('tfail').textContent=d.totalfails;
    E('rssi').textContent=d.rssi!==0?d.rssi:'N/A';
    E('ntp').textContent=d.ntp?'Synced':'No sync';
    const nm=d.netmode;
    E('net-dot').className='dot '+(nm==='STA'?'ok':nm==='AP'?'warn':'err');
    E('net-st').textContent=nm==='STA'?'WiFi OK':nm==='AP'?'AP mode':'No network';
    const w=E('warn');
    if(d.fails>3){w.style.display='block';w.textContent='PZEM failures: '+d.fails+' consecutive';}
    else w.style.display='none';
    E('mock-banner').style.display=d.mock?'block':'none';
    E('ts').textContent='Last update: '+new Date().toLocaleTimeString();
  }catch(e){
    document.getElementById('ts').textContent='Connection error: '+e.message;
  }
}
update();setInterval(update,3000);
</script></body></html>)HTML";

void handleRoot() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");
    server.sendContent_P(HTML_HEAD);
    server.sendContent_P(HTML_SCRIPT);
    server.client().flush();
}

void handleJson() {
    if (!latest.valid) {
        server.send(503, "application/json", "{\"error\":\"No sensor data\"}");
        return;
    }
    const char* nm   = (netMode == NET_STA_OK) ? "STA"
                     : (netMode == NET_AP_MODE) ? "AP" : "NONE";
    int         rssi = (netMode == NET_STA_OK) ? WiFi.RSSI() : 0;

    char buf[400];
    snprintf(buf, sizeof(buf),
        "{\"voltage\":%.2f,\"current\":%.3f,\"power\":%.2f,"
        "\"apparent\":%.2f,\"reactive\":%.2f,\"qfactor\":%.4f,"
        "\"pf\":%.3f,\"frequency\":%.2f,\"energy\":%.3f,"
        "\"uptime\":%lu,\"sd\":%s,\"logs\":%lu,"
        "\"buf\":%u,\"fails\":%u,\"totalfails\":%lu,"
        "\"rssi\":%d,\"ntp\":%s,\"netmode\":\"%s\",\"mock\":%s}",
        latest.voltage, latest.current, latest.power,
        latest.apparent, latest.reactive, latest.qFactor,
        latest.pf, latest.frequency, latest.energy,
        millis() / 1000,
        sdAvailable ? "true" : "false",
        logWriteCount,
        ringBuf.count,
        pzemFailCount,
        totalFailCount,
        rssi,
        ntpSynced ? "true" : "false",
        nm,
#ifdef GENERIC_PZEM
        "true"
#else
        "false"
#endif
    );
    server.send(200, "application/json", buf);
}

void handleExport() {
    if (!sdAvailable) {
        server.send(503, "text/plain", "SD card unavailable");
        return;
    }
    File f = SD.open(LOG_FILENAME, FILE_READ);
    if (!f) {
        server.send(404, "text/plain", "Log file not found");
        return;
    }
    server.sendHeader("Content-Disposition", "attachment; filename=\"powerlog.csv\"");
    server.setContentLength(f.size());
    server.send(200, "text/csv", "");
    uint8_t chunk[512];
    while (f.available()) {
        size_t n = f.read(chunk, sizeof(chunk));
        server.client().write(chunk, n);
    }
    f.close();
}

void handleResetEnergy() {
#ifdef GENERIC_PZEM
    server.send(200, "text/plain", "Reset not available in simulation mode");
#else
    bool ok = pzem.resetEnergy();
    server.send(200, "text/plain", ok ? "Energy counter reset OK" : "Reset failed");
    LOG("[PZEM] Energy reset %s", ok ? "OK" : "FAILED");
#endif
}

void startWebServer() {
    server.on("/",       handleRoot);
    server.on("/data",   handleJson);
    server.on("/reset",  handleResetEnergy);
    server.on("/export", handleExport);
    server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
    server.begin();
    LOG("[HTTP] Web server started on port 80");
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println(F("\n====== ESP32 Power Monitor ======"));
    Serial.printf("  CPU freq  : %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("  Free heap : %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    Serial.println(F("=================================\n"));

    initWatchdog();

#ifdef GENERIC_PZEM
    Serial.println(F("[PZEM] GENERIC_PZEM — sensor bypassed, using simulated data"));
#else
    Serial.println(F("[PZEM] Starting UART2 @ 9600 baud..."));
    Serial2.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    while (Serial2.available()) Serial2.read();
#endif

    feedWatchdog();
    initSD();
    feedWatchdog();
    setupWiFi();
    feedWatchdog();
    startWebServer();

    LOG("[INIT] Ready — dashboard: http://%s",
        (netMode == NET_STA_OK) ? WiFi.localIP().toString().c_str()
                                : WiFi.softAPIP().toString().c_str());
}

// ─────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────

void loop() {
    feedWatchdog();
    server.handleClient();

    uint32_t now = millis();

    if (now - lastReadMs >= READ_INTERVAL_MS) {
        lastReadMs = now;
        readSensor();
        feedWatchdog();
    }
    if (now - lastLogMs >= LOG_INTERVAL_MS) {
        lastLogMs = now;
        logToSD();
        feedWatchdog();
    }
    if (now - lastWifiMs >= WIFI_CHECK_MS) {
        lastWifiMs = now;
        checkWiFi();
        feedWatchdog();
    }
    if (netMode == NET_STA_OK && now - lastNtpMs >= NTP_RESYNC_MS) {
        syncNTP();
        feedWatchdog();
    }
}
