/*******************************************************************************
 * FRC-Style "Hub" Fuel Counter
 * ────────────────────────────────────────────────────────────────────────────
 * Board   : Arduino UNO R4 WiFi  (Renesas RA4M1)
 * Network : Access-Point mode – SSID "HUB_COUNTER" / password "12345678"
 * Sensors : 4× VL53L0X ToF (I2C) behind a TCA9548A I2C multiplexer
 * Web UI  : HTTP on port 80 + WebSocket on port 81
 *
 * ── Wiring Notes ─────────────────────────────────────────────────────────
 *  Arduino UNO R4 WiFi          TCA9548A Mux
 *    SDA  (A4 / D18)  ──────►  SDA
 *    SCL  (A5 / D19)  ──────►  SCL
 *    3.3 V             ──────►  VCC    (mux + all VL53L0X breakouts)
 *    GND               ──────►  GND
 *    (A0 optional)     ──────►  RST (pull HIGH via 10k; LOW to reset)
 *
 *  TCA9548A channel 0  ──────► VL53L0X  Lane 1  (SDA/SCL)
 *  TCA9548A channel 1  ──────► VL53L0X  Lane 2
 *  TCA9548A channel 2  ──────► VL53L0X  Lane 3
 *  TCA9548A channel 3  ──────► VL53L0X  Lane 4
 *
 *  Most VL53L0X breakout boards (Adafruit, Pololu, generic) have on-board
 *  regulators and level-shifters, so they are safe on 3.3 V or 5 V.
 *  The TCA9548A itself runs at 3.3 V; its I/O is 3.3 V-tolerant (matches
 *  the UNO R4 WiFi's 3.3 V I2C lines).
 *
 * ── ToF Library ──────────────────────────────────────────────────────────
 *  Uses the Pololu VL53L0X library (lib_deps = pololu/VL53L0X).
 *  If you swap to VL53L1X sensors, replace with pololu/VL53L1X and change
 *  the #include + class name — the read API is nearly identical.
 ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <UnoR4WiFi_WebServer.h>   // DIYables – includes WebSocket support
#include <VL53L0X.h>               // Pololu VL53L0X library
#include "fuel_counter.h"           // Extracted state-machine & counting logic

// ═══════════════════════════════════════════════════════════════════════════
//  CONFIGURATION — tweak these as needed
// ═══════════════════════════════════════════════════════════════════════════

// ── Access-Point credentials ────────────────────────────────────────────
static const char AP_SSID[]     = "HUB_COUNTER";
static const char AP_PASSWORD[] = "12345678";        // must be ≥8 chars

// ── Sensor / counting parameters ────────────────────────────────────────
static const uint8_t  NUM_LANES            = FC_NUM_LANES;
static const uint8_t  TCA9548A_ADDR        = 0x70;  // default mux address

// Detection: a ball is "present" when the measured distance drops below
// (baseline − DETECTION_DELTA_MM).  The hysteresis band prevents chatter
// on the transition edge.  Lockout prevents a slowly-rolling ball from
// being counted twice.
static const uint16_t DETECTION_DELTA_MM   = 80;    // mm below baseline = ball
static const uint16_t CLEAR_HYSTERESIS_MM  = 30;    // mm of hysteresis band
static const uint32_t LOCKOUT_MS           = 60;    // post-count dead time (ms)
static const uint8_t  CALIB_SAMPLES        = 20;    // samples for baseline avg
static const uint16_t SENSOR_TIMEOUT_MS    = 50;    // per-read timeout (ms)

// ═══════════════════════════════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════════════════════════════

// ── Sensor objects ──────────────────────────────────────────────────────
VL53L0X tof[NUM_LANES];                       // one object per lane (reused via mux)

// ── Per-lane state (uses Lane struct from fuel_counter.h) ───────────────
Lane lanes[NUM_LANES];
uint32_t totalCount = 0;

// ── Network / server ────────────────────────────────────────────────────
UnoR4WiFi_WebServer server(80);
UnoR4WiFi_WebSocket* ws = nullptr;

// ── Change-broadcast throttle ───────────────────────────────────────────
bool     countsChanged   = true;              // send on first client connect
uint32_t lastBroadcastMs = 0;
static const uint32_t BROADCAST_MIN_INTERVAL_MS = 50;  // max ~20 updates/s

// ═══════════════════════════════════════════════════════════════════════════
//  TCA9548A MULTIPLEXER HELPER
// ═══════════════════════════════════════════════════════════════════════════

void muxSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ═══════════════════════════════════════════════════════════════════════════
//  HTML PAGE (stored in PROGMEM to save RAM)
// ═══════════════════════════════════════════════════════════════════════════

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hub Fuel Counter</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#111;color:#eee;
     display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:20px}
h1{font-size:1.4rem;margin-bottom:8px;color:#aaa;text-transform:uppercase;letter-spacing:2px}
#total{font-size:6rem;font-weight:700;color:#0f0;text-shadow:0 0 30px #0f0;margin:10px 0}
.lanes{display:flex;gap:16px;flex-wrap:wrap;justify-content:center;margin:20px 0}
.lane{background:#1a1a1a;border:2px solid #333;border-radius:12px;padding:20px 30px;
      min-width:120px;text-align:center;transition:border-color .3s}
.lane.error{border-color:#f44}
.lane-label{font-size:.85rem;color:#888;margin-bottom:4px}
.lane-count{font-size:2.8rem;font-weight:700;color:#4fc3f7}
.lane-count.error{color:#f44}
#status{margin:18px 0;padding:6px 18px;border-radius:20px;font-size:.85rem;font-weight:600}
#status.ok{background:#1b5e20;color:#69f0ae}
#status.err{background:#b71c1c;color:#ff8a80}
button{background:#1565c0;color:#fff;border:none;padding:12px 36px;font-size:1rem;
       border-radius:8px;cursor:pointer;margin-top:10px;transition:background .2s}
button:hover{background:#1976d2}
button:active{background:#0d47a1}
.ts{color:#555;font-size:.75rem;margin-top:12px}
</style>
</head>
<body>
<h1>&#x26FD; Hub Fuel Counter</h1>
<div id="total">0</div>
<div class="lanes">
  <div class="lane" id="l1box"><div class="lane-label">Lane 1</div><div class="lane-count" id="l1">0</div></div>
  <div class="lane" id="l2box"><div class="lane-label">Lane 2</div><div class="lane-count" id="l2">0</div></div>
  <div class="lane" id="l3box"><div class="lane-label">Lane 3</div><div class="lane-count" id="l3">0</div></div>
  <div class="lane" id="l4box"><div class="lane-label">Lane 4</div><div class="lane-count" id="l4">0</div></div>
</div>
<div id="status" class="err">Disconnected</div>
<button onclick="doReset()">Reset Counts</button>
<div class="ts" id="ts"></div>
<script>
var ws,reconDelay=1000;
function connect(){
  var host=location.hostname;
  ws=new WebSocket('ws://'+host+':81');
  ws.onopen=function(){
    document.getElementById('status').className='ok';
    document.getElementById('status').textContent='Connected';
    reconDelay=1000;
    ws.send(JSON.stringify({cmd:'ping'}));
  };
  ws.onclose=function(){
    document.getElementById('status').className='err';
    document.getElementById('status').textContent='Disconnected';
    setTimeout(connect,reconDelay);
    reconDelay=Math.min(reconDelay*2,8000);
  };
  ws.onerror=function(){ws.close();};
  ws.onmessage=function(ev){
    try{
      var d=JSON.parse(ev.data);
      if(d.cmd==='pong') return;
      if('total' in d){
        document.getElementById('total').textContent=d.total;
        for(var i=1;i<=4;i++){
          var el=document.getElementById('l'+i);
          var box=document.getElementById('l'+i+'box');
          if(d['s'+i]===false){
            el.textContent='ERR';
            el.className='lane-count error';
            box.className='lane error';
          } else {
            el.textContent=d['l'+i];
            el.className='lane-count';
            box.className='lane';
          }
        }
        if(d.ts) document.getElementById('ts').textContent='Uptime: '+(d.ts/1000).toFixed(1)+'s';
      }
    }catch(e){}
  };
}
function doReset(){
  if(ws&&ws.readyState===1) ws.send(JSON.stringify({cmd:'reset'}));
}
connect();
</script>
</body>
</html>
)rawliteral";

// ═══════════════════════════════════════════════════════════════════════════
//  HTTP ROUTE HANDLER  –  GET /
// ═══════════════════════════════════════════════════════════════════════════

void handleRoot(WiFiClient& client,
                const String& method,
                const String& request,
                const QueryParams& params,
                const String& jsonData)
{
  // Send the PROGMEM page.  sendResponse expects a const char* in RAM,
  // so we stream it manually to avoid a huge RAM copy.
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/html; charset=UTF-8"));
  client.println(F("Connection: close"));
  client.println();

  // Stream from PROGMEM in small chunks
  const uint16_t chunkSize = 256;
  uint16_t len = strlen_P(INDEX_HTML);
  for (uint16_t i = 0; i < len; i += chunkSize) {
    char buf[chunkSize + 1];
    uint16_t n = min((uint16_t)chunkSize, (uint16_t)(len - i));
    memcpy_P(buf, INDEX_HTML + i, n);
    buf[n] = '\0';
    client.print(buf);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  JSON HELPERS
// ═══════════════════════════════════════════════════════════════════════════

// Build the JSON status string that is sent to all WS clients.
String buildCountsJson() {
  // {"l1":0,"l2":0,"l3":0,"l4":0,"total":0,"s1":true,..."ts":12345}
  String j = "{";
  for (uint8_t i = 0; i < NUM_LANES; i++) {
    j += "\"l" + String(i + 1) + "\":" + String(lanes[i].count) + ",";
    j += "\"s" + String(i + 1) + "\":" + (lanes[i].sensorOk ? "true" : "false") + ",";
  }
  j += "\"total\":" + String(totalCount) + ",";
  j += "\"ts\":"   + String(millis());
  j += "}";
  return j;
}

// ═══════════════════════════════════════════════════════════════════════════
//  WEBSOCKET EVENT HANDLERS
// ═══════════════════════════════════════════════════════════════════════════

void onWsOpen(net::WebSocket& /* wsClient */) {
  Serial.println(F("[WS] Client connected"));
  // Immediately send current state to the new client
  if (ws) {
    String payload = buildCountsJson();
    ws->broadcastTXT(payload);
  }
}

void onWsMessage(net::WebSocket& /* wsClient */,
                 const net::WebSocket::DataType /* dt */,
                 const char* message,
                 uint16_t length)
{
  // Minimal JSON parsing (avoid heavy libraries on constrained MCU)
  String msg(message);
  msg.trim();
  Serial.print(F("[WS] Rx: ")); Serial.println(msg);

  if (msg.indexOf("\"reset\"") >= 0) {
    // ── Reset all counts ────────────────────────────────────────────
    resetLanes(lanes, NUM_LANES, totalCount);
    countsChanged = true;
    Serial.println(F("[WS] Counts reset"));
  }
  else if (msg.indexOf("\"ping\"") >= 0) {
    // ── Ping / pong ─────────────────────────────────────────────────
    if (ws) {
      String pong = "{\"cmd\":\"pong\",\"ts\":" + String(millis()) + "}";
      ws->broadcastTXT(pong);
    }
  }
}

void onWsClose(net::WebSocket& /* wsClient */,
               const net::WebSocket::CloseCode /* code */,
               const char* /* reason */,
               uint16_t /* length */)
{
  Serial.println(F("[WS] Client disconnected"));
}

// ═══════════════════════════════════════════════════════════════════════════
//  SENSOR INITIALISATION & CALIBRATION
// ═══════════════════════════════════════════════════════════════════════════

void initSensors() {
  Wire.begin();
  Wire.setClock(400000);              // 400 kHz fast-mode I2C

  for (uint8_t ch = 0; ch < NUM_LANES; ch++) {
    muxSelect(ch);
    delay(10);

    tof[ch].setTimeout(SENSOR_TIMEOUT_MS);

    if (!tof[ch].init()) {
      Serial.print(F("[ToF] Lane "));
      Serial.print(ch + 1);
      Serial.println(F("  INIT FAILED  — check wiring!"));
      lanes[ch].sensorOk = false;
      continue;
    }

    // Use continuous mode for best throughput (~33 ms/reading at default budget)
    tof[ch].startContinuous(0);       // 0 = back-to-back, no inter-measurement gap
    lanes[ch].sensorOk = true;

    Serial.print(F("[ToF] Lane "));
    Serial.print(ch + 1);
    Serial.println(F("  OK"));
  }
}

void calibrateBaselines() {
  Serial.println(F("[Cal] Calibrating baselines — keep lanes CLEAR …"));
  delay(500);

  for (uint8_t ch = 0; ch < NUM_LANES; ch++) {
    if (!lanes[ch].sensorOk) continue;

    uint32_t sum = 0;
    uint8_t  good = 0;

    muxSelect(ch);
    delay(5);

    for (uint8_t s = 0; s < CALIB_SAMPLES; s++) {
      uint16_t d = tof[ch].readRangeContinuousMillimeters();
      if (!tof[ch].timeoutOccurred() && d < 8000) {
        sum += d;
        good++;
      }
      delay(35);
    }

    if (good > 0) {
      calculateThresholds(lanes[ch], sum / good, DETECTION_DELTA_MM, CLEAR_HYSTERESIS_MM);

      Serial.print(F("[Cal] Lane "));
      Serial.print(ch + 1);
      Serial.print(F("  baseline = "));
      Serial.print(lanes[ch].baseline_mm);
      Serial.print(F(" mm  threshold = "));
      Serial.print(lanes[ch].threshold_mm);
      Serial.println(F(" mm"));
    } else {
      lanes[ch].sensorOk = false;
      Serial.print(F("[Cal] Lane "));
      Serial.print(ch + 1);
      Serial.println(F("  calibration FAILED"));
    }
  }
  Serial.println(F("[Cal] Done."));
}

// ═══════════════════════════════════════════════════════════════════════════
//  LANE STATE-MACHINE UPDATE  (called every loop iteration)
// ═══════════════════════════════════════════════════════════════════════════

void updateLane(uint8_t ch) {
  if (!lanes[ch].sensorOk) return;

  muxSelect(ch);
  // Tiny delay for mux settling is not needed at 400 kHz in practice,
  // but add 10 µs if you see glitches:
  // delayMicroseconds(10);

  uint16_t dist = tof[ch].readRangeContinuousMillimeters();

  if (tof[ch].timeoutOccurred()) {
    // Sensor read error — skip this cycle
    return;
  }

  // Delegate to the pure-logic state machine from fuel_counter lib
  bool counted = processLaneReading(lanes[ch], dist, millis(), LOCKOUT_MS);
  if (counted) {
    totalCount++;
    countsChanged = true;
    Serial.print(F("[+] Lane "));
    Serial.print(ch + 1);
    Serial.print(F("  count="));
    Serial.print(lanes[ch].count);
    Serial.print(F("  total="));
    Serial.println(totalCount);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1500);   // give Serial monitor time to attach
  Serial.println(F("\n════════════════════════════════════════"));
  Serial.println(F("  Hub Fuel Counter — UNO R4 WiFi AP"));
  Serial.println(F("════════════════════════════════════════"));

  // ── 1. Start Wi-Fi Access Point ────────────────────────────────────────
  Serial.print(F("[WiFi] Creating AP: "));
  Serial.println(AP_SSID);

  int apResult = WiFi.beginAP(AP_SSID, AP_PASSWORD);
  if (apResult != WL_AP_LISTENING) {
    Serial.println(F("[WiFi] AP FAILED — halting."));
    while (true) { delay(1000); }
  }

  IPAddress ip = WiFi.localIP();
  Serial.print(F("[WiFi] AP IP: "));
  Serial.println(ip);
  Serial.print(F("[WiFi] Open http://"));
  Serial.print(ip);
  Serial.println(F("/ in your browser"));

  // ── 2. HTTP routes ─────────────────────────────────────────────────────
  server.addRoute("/", handleRoot);

  // Start web server (WiFi already connected via AP)
  server.begin();
  Serial.println(F("[HTTP] Server started on port 80"));

  // ── 3. WebSocket server ────────────────────────────────────────────────
  ws = server.enableWebSocket(81);
  if (ws) {
    ws->onOpen(onWsOpen);
    ws->onMessage(onWsMessage);
    ws->onClose(onWsClose);
    Serial.println(F("[WS]   Server started on port 81"));
  } else {
    Serial.println(F("[WS]   FAILED to start WebSocket server"));
  }

  Serial.print(F("[WS]   Client URL: ws://"));
  Serial.print(ip);
  Serial.println(F(":81"));

  // ── 4. Sensors ─────────────────────────────────────────────────────────
  initSensors();
  calibrateBaselines();

  Serial.println(F("\n[✓] System ready — waiting for fuel balls …\n"));
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN LOOP  –  non-blocking
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
  // ── Service HTTP + WebSocket ──────────────────────────────────────────
  server.handleClient();
  server.handleWebSocket();

  // ── Poll all 4 ToF lanes via the mux ──────────────────────────────────
  for (uint8_t ch = 0; ch < NUM_LANES; ch++) {
    updateLane(ch);
  }

  // ── Broadcast counts to WebSocket clients if something changed ────────
  if (countsChanged) {
    uint32_t now = millis();
    if (now - lastBroadcastMs >= BROADCAST_MIN_INTERVAL_MS) {
      if (ws && ws->connectedClients() > 0) {
        String json = buildCountsJson();
        ws->broadcastTXT(json);
      }
      countsChanged    = false;
      lastBroadcastMs  = now;
    }
  }

  // No delay() here — the loop runs as fast as possible for responsive
  // ball detection.  The ToF continuous-mode reads throttle naturally
  // (~30 ms per sensor × 4 = ~120 ms full cycle ≈ 8 Hz per lane).
}