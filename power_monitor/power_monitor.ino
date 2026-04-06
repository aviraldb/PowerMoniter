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
 *    - esp_task_wdt.h            — built into ESP32 Arduino core
 * ============================================================
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
#include <PZEM004Tv30.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>
#include <math.h>
#include <esp_task_wdt.h>   

// ─── WiFi / Auth Config ──────────────────────────────────────
const char* WIFI_SSID      = "aviraldubey";
const char* WIFI_PASSWORD  = "hotspot7";

// Dashboard basic-auth credentials
const char* DASH_USER      = "admin";
const char* DASH_PASS      = "monitor123";

// AP fallback — used when STA connection fails
const char* AP_SSID        = "PowerMonitor-AP";
const char* AP_PASS        = "esp32power";     // min 8 chars

// ─── NTP ─────────────────────────────────────────────────────
const char* NTP_SERVER     = "pool.ntp.org";
const long  GMT_OFFSET_S   = 19800;    // UTC+5:30 India
const int   DAYLIGHT_S     = 0;

// ─── Pin Definitions ─────────────────────────────────────────
#define PZEM_RX_PIN     16
#define PZEM_TX_PIN     17
#define SD_CS_PIN        5

// ─── Timing ──────────────────────────────────────────────────
#define READ_INTERVAL_MS     5000     // sensor read period
#define LOG_INTERVAL_MS    300000     // SD log period (5 min)
#define WIFI_CHECK_MS       30000     // WiFi health check period
#define SD_RETRY_MS         15000     // retry failed SD init
#define WDT_TIMEOUT_S         120     // watchdog timeout (in sec)

// ─── Data Validation Limits (India 230V grid) ────────────────
// Any reading outside these ranges is discarded
#define V_MIN    170.0f    // V
#define V_MAX    270.0f    // V
#define I_MIN      0.0f    // A
#define I_MAX    100.0f    // A  
#define P_MIN      0.0f    // W
#define P_MAX  23000.0f    // W  
#define F_MIN     45.0f    // Hz
#define F_MAX     55.0f    // Hz
#define PF_MIN     0.0f
#define PF_MAX     1.0f

// ─── RAM Ring Buffer ─────────────────────────────────────────
// [4] Stores last N readings — survives SD failure & power cuts
#define BUFFER_SIZE   20

// ─── SD ──────────────────────────────────────────────────────
#define LOG_FILENAME   "/log_file.csv"
#define SD_FLUSH_N      5             // flush every N writes

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

// Ring buffer — written every READ_INTERVAL, read on log/retry
struct RingBuffer {
    PowerData entries[BUFFER_SIZE];
    uint8_t   head;     // next write index
    uint8_t   count;    // how many valid entries
} ringBuf = {{}, 0, 0};

PowerData latest = {0};

// ─────────────────────────────────────────────────────────────
//  State
// ─────────────────────────────────────────────────────────────
enum WiFiMode_t { WIFI_STA_OK, WIFI_AP_MODE, WIFI_NONE };

PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
WebServer    server(80);

bool         sdAvailable   = false;
WiFiMode_t   netMode       = WIFI_NONE;
uint32_t     lastReadMs    = 0;
uint32_t     lastLogMs     = 0;
uint32_t     lastWifiMs    = 0;
uint32_t     lastSdRetryMs = 0;
uint32_t     logWriteCount = 0;
uint8_t      pzemFailCount = 0;
#define      PZEM_FAIL_REBOOT  10   // reboot after this many consecutive fails

// ─────────────────────────────────────────────────────────────
//  Watchdog
// ─────────────────────────────────────────────────────────────

void initWatchdog() {
    esp_task_wdt_init(WDT_TIMEOUT_S, true);   // panic=true → reboot on timeout
    esp_task_wdt_add(NULL);                    // watch main task
    Serial.println("[WDT] Hardware watchdog armed (" + String(WDT_TIMEOUT_S) + "s)");
}

// Call this in loop() to reset the watchdog timer
inline void feedWatchdog() {
    esp_task_wdt_reset();
}

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────

void getTimestamp(char* buf, size_t len) {
    struct tm ti;
    if (!getLocalTime(&ti)) {
        snprintf(buf, len, "NO_TIME");
        return;
    }
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", &ti);
}

void computeDerived(PowerData& d) {
    d.apparent = d.voltage * d.current;
    float s2   = d.apparent * d.apparent;
    float p2   = d.power    * d.power;
    d.reactive = (s2 >= p2) ? sqrtf(s2 - p2) : 0.0f;
    d.qFactor  = (d.power > 0.01f) ? (d.reactive / d.power) : 0.0f;
}

// Push a validated reading into the ring buffer
void pushBuffer(const PowerData& d) {
    ringBuf.entries[ringBuf.head] = d;
    ringBuf.head = (ringBuf.head + 1) % BUFFER_SIZE;
    if (ringBuf.count < BUFFER_SIZE) ringBuf.count++;
}

// ─────────────────────────────────────────────────────────────
//  Sensor Reading with Validation + Retry
// ─────────────────────────────────────────────────────────────

bool readOnce(PowerData& d) {
    d.timestamp = millis();
    d.voltage   = pzem.voltage();
    d.current   = pzem.current();
    d.power     = pzem.power();
    d.energy    = pzem.energy();
    d.frequency = pzem.frequency();
    d.pf        = pzem.pf();

    // NaN check
    if (isnan(d.voltage) || isnan(d.current) || isnan(d.power) ||
        isnan(d.energy)  || isnan(d.frequency)|| isnan(d.pf)) {
        return false;
    }

    // Range validation — discard physically impossible values
    if (d.voltage   < V_MIN  || d.voltage   > V_MAX)  { Serial.printf("[VAL] Voltage out of range: %.2f\n",   d.voltage);   return false; }
    if (d.current   < I_MIN  || d.current   > I_MAX)  { Serial.printf("[VAL] Current out of range: %.3f\n",   d.current);   return false; }
    if (d.power     < P_MIN  || d.power     > P_MAX)  { Serial.printf("[VAL] Power out of range: %.2f\n",     d.power);     return false; }
    if (d.frequency < F_MIN  || d.frequency > F_MAX)  { Serial.printf("[VAL] Frequency out of range: %.2f\n", d.frequency); return false; }
    if (d.pf        < PF_MIN || d.pf        > PF_MAX) { Serial.printf("[VAL] PF out of range: %.3f\n",        d.pf);        return false; }

    return true;
}

void readSensor() {
    PowerData d;
    bool      ok = false;

    // Up to 3 read attempts before giving up this cycle
    for (uint8_t attempt = 1; attempt <= 3; attempt++) {
        ok = readOnce(d);
        if (ok) break;
        Serial.printf("[PZEM] Attempt %u/3 failed\n", attempt);
        delay(200);
    }

    if (!ok) {
        pzemFailCount++;
        latest.valid = false;
        Serial.printf("[PZEM] Read failed (consecutive failures: %u)\n", pzemFailCount);

        // [2] Too many consecutive failures → reboot (watchdog would catch freeze,
        //     but this handles silent-bad-data scenario)
        if (pzemFailCount >= PZEM_FAIL_REBOOT) {
            Serial.println("[PZEM] Too many failures — rebooting");
            delay(500);
            ESP.restart();
        }
        return;
    }

    pzemFailCount = 0;
    d.valid = true;
    computeDerived(d);
    latest = d;

    // [4] Store in ring buffer regardless of SD state
    pushBuffer(d);

    Serial.println("─────────────────────────────────");
    Serial.printf("  Voltage     : %.2f V\n",   d.voltage);
    Serial.printf("  Current     : %.3f A\n",   d.current);
    Serial.printf("  Active P    : %.2f W\n",   d.power);
    Serial.printf("  Apparent S  : %.2f VA\n",  d.apparent);
    Serial.printf("  Reactive Q  : %.2f VAR\n", d.reactive);
    Serial.printf("  Q Factor    : %.4f\n",      d.qFactor);
    Serial.printf("  Power Factor: %.3f\n",      d.pf);
    Serial.printf("  Frequency   : %.2f Hz\n",  d.frequency);
    Serial.printf("  Energy      : %.3f kWh\n", d.energy);
    Serial.println("─────────────────────────────────");
}

// ─────────────────────────────────────────────────────────────
//  SD Card 
// ─────────────────────────────────────────────────────────────

void initSD() {
    SD.end();  
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[SD] Mount failed — will retry");
        sdAvailable = false;
        return;
    }
    sdAvailable = true;
    Serial.println("[SD] Mounted OK");

    if (!SD.exists(LOG_FILENAME)) {
        File f = SD.open(LOG_FILENAME, FILE_WRITE);
        if (f) {
            f.println("timestamp,voltage_V,current_A,active_power_W,"
                      "apparent_power_VA,reactive_power_VAR,q_factor,"
                      "power_factor,frequency_Hz,energy_kWh");
            f.close();
            Serial.println("[SD] Log file created with header");
        }
    }
}

// Write a single PowerData row to SD; returns true on success
bool writeRow(File& f, const PowerData& d) {
    char ts[32];
    getTimestamp(ts, sizeof(ts));
    // Use stack buffer instead of String to avoid heap churn [7]
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

    // [2] If SD was lost, attempt re-init before logging
    if (!sdAvailable) {
        uint32_t now = millis();
        if (now - lastSdRetryMs >= SD_RETRY_MS) {
            lastSdRetryMs = now;
            Serial.println("[SD] Retrying mount...");
            initSD();
        }
        if (!sdAvailable) {
            Serial.println("[SD] Still unavailable — data held in RAM buffer");
            return;
        }
    }

    File f = SD.open(LOG_FILENAME, FILE_APPEND);
    if (!f) {
        Serial.println("[SD] Open for append failed — marking unavailable");
        sdAvailable = false;   // will trigger retry next cycle
        return;
    }

    // Flush any buffered rows that were held during SD outage
    // Walk the ring buffer from oldest to newest
    if (ringBuf.count > 1) {
        uint8_t oldest = (ringBuf.head + BUFFER_SIZE - ringBuf.count) % BUFFER_SIZE;
        for (uint8_t i = 0; i < ringBuf.count - 1; i++) {    // -1: latest written below
            uint8_t idx = (oldest + i) % BUFFER_SIZE;
            writeRow(f, ringBuf.entries[idx]);
        }
        ringBuf.count = 1;    // keep only the latest entry
        Serial.printf("[SD] Flushed %u buffered rows\n", ringBuf.count);
    }

    // Write current reading
    bool ok = writeRow(f, latest);
    logWriteCount++;

    if (logWriteCount % SD_FLUSH_N == 0) f.flush();
    f.close();

    if (ok) {
        char ts[32]; getTimestamp(ts, sizeof(ts));
        Serial.printf("[SD] Row #%lu logged at %s\n", logWriteCount, ts);
    } else {
        Serial.println("[SD] Write error");
        sdAvailable = false;
    }
}

// ─────────────────────────────────────────────────────────────
//  WiFi — STA with AP Fallback
// ─────────────────────────────────────────────────────────────

void startAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    netMode = WIFI_AP_MODE;
    Serial.printf("[WiFi] AP mode started — connect to '%s'\n", AP_SSID);
    Serial.printf("[WiFi] AP IP: http://%s\n", WiFi.softAPIP().toString().c_str());
}

void setupWiFi() {
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);    // [2] auto-reconnect on drop
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Non-blocking wait — cap at 15 seconds so setup() doesn't hang [6 fix]
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        feedWatchdog();    // [1] keep watchdog happy during connect wait
        delay(300);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        netMode = WIFI_STA_OK;
        Serial.printf("\n[WiFi] Connected! IP: http://%s\n",
                      WiFi.localIP().toString().c_str());
        configTime(GMT_OFFSET_S, DAYLIGHT_S, NTP_SERVER);
        Serial.println("[NTP] Time sync requested");
    } else {
        Serial.println("\n[WiFi] STA failed — starting AP fallback");
        startAP();    // [5]
    }
}

// Periodic WiFi health check — reconnect if dropped
void checkWiFi() {
    if (netMode == WIFI_AP_MODE) return;    // AP doesn't drop
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Connection lost — reconnecting...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
            feedWatchdog();
            delay(300);
        }
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("[WiFi] Reconnected");
        } else {
            Serial.println("[WiFi] Reconnect failed — switching to AP mode");
            startAP();
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  Basic Authentication helper
// ─────────────────────────────────────────────────────────────

bool isAuthenticated() {
    if (!server.authenticate(DASH_USER, DASH_PASS)) {
        server.requestAuthentication(DIGEST_AUTH, "PowerMonitor");
        return false;
    }
    return true;
}

// ─────────────────────────────────────────────────────────────
//  Web Dashboard — HTML in PROGMEM
// ─────────────────────────────────────────────────────────────

// Stored in flash, not RAM
static const char HTML_HEAD[] PROGMEM = R"(<!DOCTYPE html>
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
button{margin-top:12px;padding:6px 18px;border-radius:6px;border:1px solid #334155;background:#1e293b;color:#94a3b8;cursor:pointer;font-size:12px}
button:hover{background:#334155}
</style></head><body>
<h1>&#9889; ESP32 Power Monitor</h1>
<div class="banner err" id="err">Sensor read error &mdash; check PZEM wiring</div>
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
<span><span class="dot" id="net-dot"></span><span id="net-st">&#8212;</span></span>
</div>
<div style="text-align:center">
<button onclick="fetch('/reset').then(r=>r.text()).then(t=>alert(t))">Reset Energy Counter</button>
</div>
<div id="ts">Updating...</div>
)";

static const char HTML_SCRIPT[] PROGMEM = R"(<script>
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
    E('buf').textContent=d.buf+'/20';
    E('fail').textContent=d.fails;
    const nm=d.netmode;
    E('net-dot').className='dot '+(nm==='STA'?'ok':nm==='AP'?'warn':'err');
    E('net-st').textContent=nm==='STA'?'WiFi OK':nm==='AP'?'AP mode':'No network';
    const w=E('warn');
    if(d.fails>3){w.style.display='block';w.textContent='PZEM failures: '+d.fails+' consecutive';}
    else w.style.display='none';
    E('ts').textContent='Last update: '+new Date().toLocaleTimeString();
  }catch(e){
    document.getElementById('ts').textContent='Connection error: '+e.message;
  }
}
update();setInterval(update,3000);
</script></body></html>)";

// Chunked response — streams HTML in pieces, no single large alloc
void handleRoot() {
    if (!isAuthenticated()) return;
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");
    // Stream head from PROGMEM
    server.sendContent_P(HTML_HEAD);
    server.sendContent_P(HTML_SCRIPT);
    server.client().flush();
}

// JSON endpoint — stack-allocated char buffer, no String heap fragmentation [7]
void handleJson() {
    if (!isAuthenticated()) return;
    if (!latest.valid) {
        server.send(503, "application/json", "{\"error\":\"No sensor data\"}");
        return;
    }
    const char* nm = (netMode == WIFI_STA_OK) ? "STA"
                   : (netMode == WIFI_AP_MODE) ? "AP" : "NONE";
    char buf[320];
    snprintf(buf, sizeof(buf),
        "{\"voltage\":%.2f,\"current\":%.3f,\"power\":%.2f,"
        "\"apparent\":%.2f,\"reactive\":%.2f,\"qfactor\":%.4f,"
        "\"pf\":%.3f,\"frequency\":%.2f,\"energy\":%.3f,"
        "\"uptime\":%lu,\"sd\":%s,\"logs\":%lu,"
        "\"buf\":%u,\"fails\":%u,\"netmode\":\"%s\"}",
        latest.voltage, latest.current, latest.power,
        latest.apparent, latest.reactive, latest.qFactor,
        latest.pf, latest.frequency, latest.energy,
        millis() / 1000,
        sdAvailable ? "true" : "false",
        logWriteCount,
        ringBuf.count,
        pzemFailCount,
        nm);
    server.send(200, "application/json", buf);
}

void handleResetEnergy() {
    if (!isAuthenticated()) return;
    bool ok = pzem.resetEnergy();
    server.send(200, "text/plain", ok ? "Energy counter reset OK" : "Reset failed");
    Serial.println(ok ? "[PZEM] Energy reset OK" : "[PZEM] Energy reset FAILED");
}

void startWebServer() {
    server.on("/",      handleRoot);
    server.on("/data",  handleJson);
    server.on("/reset", handleResetEnergy);
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });
    server.begin();
    Serial.println("[HTTP] Web server started");
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n====== ESP32 Power Monitor ======");

    // Arm watchdog first — catches hangs anywhere in setup
    initWatchdog();

    // UART for PZEM
    Serial2.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    Serial.println("[PZEM] UART2 @ 9600 baud");
    feedWatchdog();

    // SD card
    initSD();
    feedWatchdog();

    // WiFi (non-blocking, AP fallback) [5]
    setupWiFi();
    feedWatchdog();

    // Web server
    startWebServer();

    Serial.println("[INIT] Setup complete");
    Serial.println("[INFO] Dashboard credentials: " + String(DASH_USER) + " / " + String(DASH_PASS));
}

// ─────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────

void loop() {
    feedWatchdog(); 

    server.handleClient();

    uint32_t now = millis();

    // Sensor read with retry
    if (now - lastReadMs >= READ_INTERVAL_MS) {
        lastReadMs = now;
        readSensor();
        feedWatchdog();    // PZEM read can take up to ~500ms
    }

    // SD log with retry 
    if (now - lastLogMs >= LOG_INTERVAL_MS) {
        lastLogMs = now;
        logToSD();
        feedWatchdog();
    }

    // WiFi health check
    if (now - lastWifiMs >= WIFI_CHECK_MS) {
        lastWifiMs = now;
        checkWiFi();
        feedWatchdog();
    }
}
