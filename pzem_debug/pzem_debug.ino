/*
 * ============================================================
 *  PZEM-004T Debug Sketch
 *  Purpose : Figure out baud rate and PZEM communication
 *  Hardware: ESP32 + PZEM-004T
 *
 *  WIRING:
 *    ESP32 VIN   ──── PZEM VCC
 *    ESP32 GND   ──── PZEM GND
 *    ESP32 GPIO17 ─── PZEM RX
 *    ESP32 GPIO16 ─── PZEM TX
 *
 *  SERIAL COMMANDS (115200 baud):
 *    scan    — try all baud rates and print raw bytes
 *    read    — attempt PZEM read at current baud rate
 *    baud    — show current baud rate
 *    set4800 / set9600 / set19200 / set38400 / set115200
 *            — switch to that baud rate manually
 * ============================================================
 */

#include <Arduino.h>
#include <PZEM004Tv30.h>

#define PZEM_RX_PIN  16
#define PZEM_TX_PIN  17

// Current baud rate — change this if you find the right one
uint32_t currentBaud = 9600;

// PZEM object — recreated when baud changes
PZEM004Tv30* pzem = nullptr;

// Serial command buffer
char cmdBuf[32];
uint8_t cmdLen = 0;

// ─────────────────────────────────────────────────────────────
//  Raw UART test — send broadcast read, print raw bytes
// ─────────────────────────────────────────────────────────────

void rawTest(uint32_t baud) {
    Serial.printf("\n[RAW] Testing at %lu baud...\n", baud);

    Serial2.end();
    delay(100);
    Serial2.begin(baud, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    delay(100);

    // Flush
    while (Serial2.available()) Serial2.read();

    // Broadcast read command (address 0xF8)
    uint8_t cmd[] = {0xF8, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x64, 0x64};
    Serial2.write(cmd, sizeof(cmd));
    Serial.print("[RAW] Sent:     ");
    for (int i = 0; i < sizeof(cmd); i++) Serial.printf("0x%02X ", cmd[i]);
    Serial.println();

    delay(1000);

    int avail = Serial2.available();
    Serial.printf("[RAW] Received: %d bytes\n", avail);

    if (avail == 0) {
        Serial.println("[RAW] No response");
        return;
    }

    Serial.print("[RAW] Bytes:    ");
    int count = 0;
    while (Serial2.available()) {
        Serial.printf("0x%02X ", Serial2.read());
        count++;
        if (count % 8 == 0) Serial.print("\n              ");
    }
    Serial.println();

    // A valid PZEM response is 25 bytes starting with addr, 0x04, 0x14
    if (avail == 25) {
        Serial.println("[RAW] *** 25 bytes — looks like a valid PZEM frame! ***");
    } else if (avail > 0) {
        Serial.println("[RAW] Wrong byte count — baud rate likely still wrong");
    }
}

// ─────────────────────────────────────────────────────────────
//  Library read test at current baud
// ─────────────────────────────────────────────────────────────

void libraryRead() {
    Serial.printf("\n[LIB] Reading via PZEM library at %lu baud...\n", currentBaud);

    if (pzem) delete pzem;
    Serial2.end();
    delay(100);
    Serial2.begin(currentBaud, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    delay(100);
    pzem = new PZEM004Tv30(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
    delay(200);

    float v  = pzem->voltage();
    float i  = pzem->current();
    float p  = pzem->power();
    float e  = pzem->energy();
    float hz = pzem->frequency();
    float pf = pzem->pf();

    Serial.println("[LIB] Results:");
    Serial.printf("  Voltage   : %s\n", isnan(v)  ? "NaN (fail)" : String(v,  2).c_str());
    Serial.printf("  Current   : %s\n", isnan(i)  ? "NaN (fail)" : String(i,  3).c_str());
    Serial.printf("  Power     : %s\n", isnan(p)  ? "NaN (fail)" : String(p,  2).c_str());
    Serial.printf("  Energy    : %s\n", isnan(e)  ? "NaN (fail)" : String(e,  3).c_str());
    Serial.printf("  Frequency : %s\n", isnan(hz) ? "NaN (fail)" : String(hz, 2).c_str());
    Serial.printf("  PF        : %s\n", isnan(pf) ? "NaN (fail)" : String(pf, 3).c_str());

    if (!isnan(v) && !isnan(i)) {
        Serial.println("\n[LIB] *** SUCCESS — PZEM is working! ***");
        Serial.printf("[LIB] Use baud rate %lu in your main sketch\n", currentBaud);
    } else {
        Serial.println("\n[LIB] Failed — try 'scan' or a different baud rate");
    }
}

// ─────────────────────────────────────────────────────────────
//  Scan all common baud rates
// ─────────────────────────────────────────────────────────────

void scanBauds() {
    uint32_t bauds[] = {4800, 9600, 19200, 38400, 57600, 115200};
    Serial.println("\n[SCAN] Trying all baud rates...");
    Serial.println("[SCAN] Looking for 25-byte response\n");

    for (int b = 0; b < 6; b++) {
        rawTest(bauds[b]);
        delay(500);
    }

    Serial.println("\n[SCAN] Done. If you saw '25 bytes — valid frame', use that baud.");
    Serial.println("[SCAN] Then type 'setXXXX' and 'read' to confirm.");
}

// ─────────────────────────────────────────────────────────────
//  Set baud rate
// ─────────────────────────────────────────────────────────────

void setBaud(uint32_t baud) {
    currentBaud = baud;
    Serial2.end();
    delay(100);
    Serial2.begin(currentBaud, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    Serial.printf("[BAUD] Switched to %lu\n", currentBaud);
}

// ─────────────────────────────────────────────────────────────
//  Serial command handler
// ─────────────────────────────────────────────────────────────

void printHelp() {
    Serial.println(F("\n=== PZEM Debug Commands ==="));
    Serial.println(F("  scan      try all baud rates"));
    Serial.println(F("  read      library read at current baud"));
    Serial.println(F("  raw       raw UART test at current baud"));
    Serial.println(F("  baud      show current baud rate"));
    Serial.println(F("  set4800   switch to 4800 baud"));
    Serial.println(F("  set9600   switch to 9600 baud"));
    Serial.println(F("  set19200  switch to 19200 baud"));
    Serial.println(F("  set38400  switch to 38400 baud"));
    Serial.println(F("  set115200 switch to 115200 baud"));
    Serial.println(F("  help      this list"));
    Serial.println();
}

void processCommand(const char* cmd) {
    Serial.printf("\n> %s\n", cmd);

    if      (strcmp(cmd, "scan")      == 0) scanBauds();
    else if (strcmp(cmd, "read")      == 0) libraryRead();
    else if (strcmp(cmd, "raw")       == 0) rawTest(currentBaud);
    else if (strcmp(cmd, "baud")      == 0) Serial.printf("[BAUD] Current: %lu\n", currentBaud);
    else if (strcmp(cmd, "set4800")   == 0) setBaud(4800);
    else if (strcmp(cmd, "set9600")   == 0) setBaud(9600);
    else if (strcmp(cmd, "set19200")  == 0) setBaud(19200);
    else if (strcmp(cmd, "set38400")  == 0) setBaud(38400);
    else if (strcmp(cmd, "set115200") == 0) setBaud(115200);
    else if (strcmp(cmd, "help")      == 0) printHelp();
    else Serial.printf("  Unknown: '%s' — type 'help'\n", cmd);
}

void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                processCommand(cmdBuf);
                cmdLen = 0;
            }
        } else if (cmdLen < 31) {
            cmdBuf[cmdLen++] = c;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  Setup & Loop
// ─────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println(F("\n====== PZEM Debug ======"));
    Serial.printf("  RX pin : GPIO%d\n", PZEM_RX_PIN);
    Serial.printf("  TX pin : GPIO%d\n", PZEM_TX_PIN);
    Serial.printf("  Baud   : %lu\n", currentBaud);
    Serial.println(F("========================\n"));

    Serial2.begin(currentBaud, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);
    pzem = new PZEM004Tv30(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);
    delay(200);

    printHelp();
    Serial.println("Type 'scan' to auto-detect baud rate, or 'read' to test now.");
}

void loop() {
    handleSerial();
}
