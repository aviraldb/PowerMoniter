/*
 * ============================================================
 *  ESP32 Power Monitor
 *  Hardware : ESP32 + PZEM-004T v3.0 + SD Card Module
 *  Author   : Aviral Dubey (@aviraldb)
 *  Framework: Arduino (ESP32 Arduino Core)
 *  Features : Live sensor readings, SD card CSV logging,
 *             built-in web dashboard, Q-factor calculation
 * ============================================================
 *
 *  REQUIRED LIBRARIES (install via Arduino Library Manager):
 *    - PZEM004Tv30  by olehs          (search "PZEM004T")
 *    - SD           built-in ESP32 core
 *    - FS           built-in ESP32 core
 *    - WiFi         built-in ESP32 core
 *    - WebServer    built-in ESP32 core
 *    - time.h       built-in
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PZEM004Tv30.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>
#include <math.h>

// ─── WiFi Credentials ────────────────────────────────────────
const char* WIFI_SSID     = "aviraldubey";
const char* WIFI_PASSWORD = "hotspot7";

// ─── NTP (for timestamps on SD logs) ─────────────────────────
const char* NTP_SERVER    = "pool.ntp.org";
const long  GMT_OFFSET_S  = 19800;    // UTC+5.30  (India) — adjust for your timezone
const int   DAYLIGHT_S    = 0;        // Daylight saving — set 0 if not applicable

// ─── Pin Definitions ─────────────────────────────────────────
#define PZEM_RX_PIN   16    // ESP32 RX ← PZEM TX (use level shifter!)
#define PZEM_TX_PIN   17    // ESP32 TX → PZEM RX
#define SD_CS_PIN      5    // SD card chip select

// ─── Intervals ───────────────────────────────────────────────
#define READ_INTERVAL_MS    5000    // Sensor read interval (ms)
#define LOG_INTERVAL_MS     300000  // SD card log interval (ms)
#define SD_FLUSH_EVERY_N    10      // Flush SD file every N log writes

// ─── SD Log filename ─────────────────────────────────────────
#define LOG_FILENAME    "/log_file.csv"

// ─── Objects ─────────────────────────────────────────────────
// PZEM on UART2 (Serial2)
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);

WebServer server(80);

// ─── Sensor Data Struct ───────────────────────────────────────
struct PowerData {
    float voltage;      // V
    float current;      // A
    float power;        // W  (active power P)
    float energy;       // kWh (cumulative)
    float frequency;    // Hz
    float pf;           // Power factor (cos φ)
    float apparent;     // VA  (apparent power S = V × I)
    float reactive;     // VAR (reactive power Q = √(S²-P²))
    float qFactor;      // Q factor = tan(φ) = Q/P
    bool  valid;        // false if sensor returned NaN
    unsigned long timestamp; // millis() of last read
};

PowerData latest = {0};

// ─── State ───────────────────────────────────────────────────
bool     sdAvailable    = false;
bool     wifiConnected  = false;
uint32_t lastReadMs     = 0;
uint32_t lastLogMs      = 0;
uint32_t logWriteCount  = 0;
File     logFile;

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────

String getTimestamp() {
    struct tm ti;
    if (!getLocalTime(&ti)) return "NO_TIME";
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &ti);
    return String(buf);
}

// Compute derived quantities from raw PZEM readings
void computeDerived(PowerData& d) {
    // Apparent power S = V × I
    d.apparent = d.voltage * d.current;

    // Reactive power Q = sqrt(S² - P²)  [clamped to 0 to avoid NaN on rounding errors]
    float s2 = d.apparent * d.apparent;
    float p2 = d.power * d.power;
    d.reactive = (s2 >= p2) ? sqrtf(s2 - p2) : 0.0f;

    // Q factor (displacement factor tan φ)
    // tan φ = Q / P  — guard against division by zero
    if (d.power > 0.01f) {
        d.qFactor = d.reactive / d.power;
    } else {
        d.qFactor = 0.0f;
    }
}

// ─────────────────────────────────────────────────────────────
//  Sensor Reading
// ─────────────────────────────────────────────────────────────

void readSensor() {
    PowerData d;
    d.timestamp = millis();

    d.voltage   = pzem.voltage();
    d.current   = pzem.current();
    d.power     = pzem.power();
    d.energy    = pzem.energy();
    d.frequency = pzem.frequency();
    d.pf        = pzem.pf();

    // PZEM returns NaN on communication error
    if (isnan(d.voltage) || isnan(d.current) || isnan(d.power)) {
        Serial.println("[PZEM] Read error — check wiring and baud rate");
        d.valid = false;
        latest.valid = false;
        return;
    }

    d.valid = true;
    computeDerived(d);
    latest = d;

    // Pretty-print to Serial Monitor
    Serial.println("─────────────────────────────────");
    Serial.printf("  Voltage     : %.2f V\n",    d.voltage);
    Serial.printf("  Current     : %.3f A\n",    d.current);
    Serial.printf("  Active P    : %.2f W\n",    d.power);
    Serial.printf("  Apparent S  : %.2f VA\n",   d.apparent);
    Serial.printf("  Reactive Q  : %.2f VAR\n",  d.reactive);
    Serial.printf("  Q Factor    : %.4f\n",       d.qFactor);
    Serial.printf("  Power Factor: %.3f\n",       d.pf);
    Serial.printf("  Frequency   : %.2f Hz\n",   d.frequency);
    Serial.printf("  Energy      : %.3f kWh\n",  d.energy);
    Serial.println("─────────────────────────────────");
}

// ─────────────────────────────────────────────────────────────
//  SD Card Logging
// ─────────────────────────────────────────────────────────────

void initSD() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("[SD] Mount failed — check wiring or card format (FAT32)");
        sdAvailable = false;
        return;
    }
    sdAvailable = true;
    Serial.println("[SD] Mounted OK");

    // Write CSV header if file is new
    if (!SD.exists(LOG_FILENAME)) {
        logFile = SD.open(LOG_FILENAME, FILE_WRITE);
        if (logFile) {
            logFile.println(
                "timestamp,voltage_V,current_A,active_power_W,"
                "apparent_power_VA,reactive_power_VAR,q_factor,"
                "power_factor,frequency_Hz,energy_kWh"
            );
            logFile.close();
            Serial.println("[SD] Created log file with header");
        }
    }
}

void logToSD() {
    if (!sdAvailable || !latest.valid) return;

    logFile = SD.open(LOG_FILENAME, FILE_APPEND);
    if (!logFile) {
        Serial.println("[SD] Failed to open log file for append");
        return;
    }

    String ts = getTimestamp();
    logFile.printf(
        "%s,%.2f,%.3f,%.2f,%.2f,%.2f,%.4f,%.3f,%.2f,%.3f\n",
        ts.c_str(),
        latest.voltage, latest.current, latest.power,
        latest.apparent, latest.reactive, latest.qFactor,
        latest.pf, latest.frequency, latest.energy
    );

    logWriteCount++;

    // Flush periodically to reduce wear but ensure data is saved
    if (logWriteCount % SD_FLUSH_EVERY_N == 0) {
        logFile.flush();
    }
    logFile.close();

    Serial.printf("[SD] Logged row #%lu at %s\n", logWriteCount, ts.c_str());
}

// ─────────────────────────────────────────────────────────────
//  Web Dashboard (served from ESP32)
// ─────────────────────────────────────────────────────────────

// Returns JSON of latest readings — used by the dashboard JS
void handleJson() {
    if (!latest.valid) {
        server.send(503, "application/json", "{\"error\":\"No sensor data\"}");
        return;
    }
    String json = "{";
    json += "\"voltage\":"   + String(latest.voltage,   2) + ",";
    json += "\"current\":"   + String(latest.current,   3) + ",";
    json += "\"power\":"     + String(latest.power,     2) + ",";
    json += "\"apparent\":"  + String(latest.apparent,  2) + ",";
    json += "\"reactive\":"  + String(latest.reactive,  2) + ",";
    json += "\"qfactor\":"   + String(latest.qFactor,   4) + ",";
    json += "\"pf\":"        + String(latest.pf,        3) + ",";
    json += "\"frequency\":" + String(latest.frequency, 2) + ",";
    json += "\"energy\":"    + String(latest.energy,    3) + ",";
    json += "\"uptime\":"    + String(millis() / 1000)     + ",";
    json += "\"sd\":"        + String(sdAvailable ? "true" : "false") + ",";
    json += "\"logs\":"      + String(logWriteCount);
    json += "}";
    server.send(200, "application/json", json);
}

// Main HTML dashboard — auto-refreshes via JS fetch
void handleRoot() {
    String html = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
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
  .card.warn .value{color:#fb923c}
  .card.good .value{color:#34d399}
  .status{max-width:720px;margin:0 auto;display:flex;gap:10px;flex-wrap:wrap;justify-content:center;font-size:12px;color:#64748b}
  .dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px}
  .dot.ok{background:#34d399} .dot.err{background:#f87171}
  #ts{text-align:center;font-size:11px;color:#475569;margin-top:12px}
  .err-banner{display:none;max-width:720px;margin:0 auto 16px;background:#7f1d1d;color:#fca5a5;border-radius:8px;padding:10px;text-align:center;font-size:13px}
</style>
</head>
<body>
<h1>⚡ ESP32 Power Monitor</h1>
<div class="err-banner" id="err">Sensor read error — check PZEM wiring</div>
<div class="grid">
  <div class="card"><div class="label">Voltage</div><div class="value" id="v">—</div><div class="unit">V</div></div>
  <div class="card"><div class="label">Current</div><div class="value" id="i">—</div><div class="unit">A</div></div>
  <div class="card good"><div class="label">Active Power</div><div class="value" id="p">—</div><div class="unit">W</div></div>
  <div class="card"><div class="label">Apparent Power</div><div class="value" id="s">—</div><div class="unit">VA</div></div>
  <div class="card warn"><div class="label">Reactive Power</div><div class="value" id="q">—</div><div class="unit">VAR</div></div>
  <div class="card"><div class="label">Q Factor</div><div class="value" id="qf">—</div><div class="unit">tan φ</div></div>
  <div class="card"><div class="label">Power Factor</div><div class="value" id="pf">—</div><div class="unit">cos φ</div></div>
  <div class="card"><div class="label">Frequency</div><div class="value" id="hz">—</div><div class="unit">Hz</div></div>
  <div class="card"><div class="label">Energy</div><div class="value" id="en">—</div><div class="unit">kWh</div></div>
  <div class="card"><div class="label">Uptime</div><div class="value" id="up">—</div><div class="unit">seconds</div></div>
</div>
<div class="status">
  <span><span class="dot" id="sd-dot"></span>SD: <span id="sd-st">—</span></span>
  <span>Log rows: <span id="logs">—</span></span>
</div>
<div id="ts">Updating...</div>
<script>
async function update(){
  try{
    const r=await fetch('/data');
    if(!r.ok)throw new Error('HTTP '+r.status);
    const d=await r.json();
    if(d.error){document.getElementById('err').style.display='block';return;}
    document.getElementById('err').style.display='none';
    document.getElementById('v').textContent=d.voltage.toFixed(1);
    document.getElementById('i').textContent=d.current.toFixed(3);
    document.getElementById('p').textContent=d.power.toFixed(1);
    document.getElementById('s').textContent=d.apparent.toFixed(1);
    document.getElementById('q').textContent=d.reactive.toFixed(1);
    document.getElementById('qf').textContent=d.qfactor.toFixed(3);
    document.getElementById('pf').textContent=d.pf.toFixed(3);
    document.getElementById('hz').textContent=d.frequency.toFixed(2);
    document.getElementById('en').textContent=d.energy.toFixed(3);
    document.getElementById('up').textContent=d.uptime;
    const sdOk=d.sd;
    document.getElementById('sd-dot').className='dot '+(sdOk?'ok':'err');
    document.getElementById('sd-st').textContent=sdOk?'OK':'Not found';
    document.getElementById('logs').textContent=d.logs;
    document.getElementById('ts').textContent='Last update: '+new Date().toLocaleTimeString();
  }catch(e){
    document.getElementById('ts').textContent='Connection error: '+e.message;
  }
}
update();
setInterval(update,2000);
</script>
</body>
</html>)rawhtml";

    server.send(200, "text/html", html);
}

// Endpoint to reset the PZEM energy counter
void handleResetEnergy() {
    bool ok = pzem.resetEnergy();
    server.send(200, "text/plain", ok ? "Energy counter reset OK" : "Reset failed");
    Serial.println(ok ? "[PZEM] Energy counter reset" : "[PZEM] Energy reset FAILED");
}

// ─────────────────────────────────────────────────────────────
//  WiFi Setup
// ─────────────────────────────────────────────────────────────

void setupWiFi() {
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println("\n[WiFi] Connected!");
        Serial.printf("[WiFi] IP Address: http://%s\n", WiFi.localIP().toString().c_str());

        // Sync time via NTP
        configTime(GMT_OFFSET_S, DAYLIGHT_S, NTP_SERVER);
        Serial.println("[NTP] Time sync requested");
    } else {
        Serial.println("\n[WiFi] Failed — running without network (SD log still works)");
        wifiConnected = false;
    }
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n====== ESP32 Power Monitor ======");

    // Initialize PZEM UART
    Serial2.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    Serial.println("[PZEM] UART2 initialized at 9600 baud");

    // Initialize SD card
    initSD();

    // Connect to WiFi
    setupWiFi();

    // Register web server routes
    if (wifiConnected) {
        server.on("/",            handleRoot);
        server.on("/data",        handleJson);
        server.on("/reset",       handleResetEnergy);
        server.onNotFound([](){
            server.send(404, "text/plain", "Not found");
        });
        server.begin();
        Serial.println("[HTTP] Web server started");
    }

    Serial.println("[INIT] Setup complete — entering loop");
}

// ─────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────

void loop() {
    // Handle incoming HTTP requests
    if (wifiConnected) {
        server.handleClient();
    }

    uint32_t now = millis();

    // Read sensor at READ_INTERVAL_MS
    if (now - lastReadMs >= READ_INTERVAL_MS) {
        lastReadMs = now;
        readSensor();
    }

    // Log to SD at LOG_INTERVAL_MS
    if (now - lastLogMs >= LOG_INTERVAL_MS) {
        lastLogMs = now;
        logToSD();
    }
}
