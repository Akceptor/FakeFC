#include "web.h"
#include "rx.h"
#include <WiFi.h>
#include <WebServer.h>
#include <stdarg.h>

static WebServer server(80);

// ── Log buffer ───────────────────────────────────────────────────────────────
#define LOG_LINES 48
#define LOG_LINE_MAX 96
static char  log_lines[LOG_LINES][LOG_LINE_MAX];
static uint8_t log_head = 0;   // next write position
static uint8_t log_count = 0;  // number of valid lines

static void log_push(const char* msg) {
    uint32_t t = millis();
    snprintf(log_lines[log_head], LOG_LINE_MAX, "%lu ms: %s",
             (unsigned long)t, msg);
    log_head = (uint8_t)((log_head + 1) % LOG_LINES);
    if (log_count < LOG_LINES) log_count++;
}

void web_log(const char* msg) { log_push(msg); }

void web_logf(const char* fmt, ...) {
    char buf[LOG_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    log_push(buf);
}

// ── HTML page ─────────────────────────────────────────────────────────────────
// Polls /channels every 100 ms and draws a bar graph for each channel.
static const char PAGE[] PROGMEM = R"(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>FakeFC - RC Channels</title>
<style>
  body { font-family: monospace; background: #111; color: #eee; padding: 20px; }
  h2   { margin-bottom: 16px; }
  .ch  { display: flex; align-items: center; margin: 4px 0; }
  .lbl { width: 40px; text-align: right; margin-right: 8px; color: #aaa; }
  .bar { height: 16px; background: #4af; min-width: 2px; transition: width 0.08s; }
  .val { margin-left: 8px; width: 44px; }
  #log { margin-top: 20px; white-space: pre; font-size: 12px; color: #bbb; }
</style>
</head>
<body>
<h2>RC Channels</h2>
<div id="channels"></div>
<h2>Log</h2>
<div id="log">loading...</div>
<script>
const N = 16;
const MIN = 1000, MAX = 2000, RANGE = MAX - MIN;
const div = document.getElementById('channels');
const logDiv = document.getElementById('log');

// Build rows once
for (let i = 0; i < N; i++) {
  div.insertAdjacentHTML('beforeend',
    `<div class="ch">
       <span class="lbl">CH${i+1}</span>
       <div class="bar" id="b${i}" style="width:50%"></div>
       <span class="val" id="v${i}">1500</span>
     </div>`);
}

async function update() {
  try {
    const r = await fetch('/channels');
    const d = await r.json();
    d.forEach((v, i) => {
      const pct = Math.round((v - MIN) / RANGE * 100);
      document.getElementById('b' + i).style.width = pct + '%';
      document.getElementById('v' + i).textContent = v;
    });
  } catch(e) {}
  setTimeout(update, 100);
}
update();

async function updateLog() {
  try {
    const r = await fetch('/log');
    const d = await r.json();
    logDiv.textContent = d.join('\n');
  } catch(e) {}
  setTimeout(updateLog, 500);
}
updateLog();
</script>
</body>
</html>
)";

// ── Handlers ──────────────────────────────────────────────────────────────────

static void handle_root() {
    server.send_P(200, "text/html", PAGE);
}

static void handle_channels() {
    const uint16_t* ch = rx_get_channels();
    String json = "[";
    for (uint8_t i = 0; i < RX_CHANNEL_COUNT; i++) {
        if (i) json += ',';
        json += ch[i];
    }
    json += ']';
    server.send(200, "application/json", json);
}

static void handle_log() {
    String json = "[";
    uint8_t start = (log_head + LOG_LINES - log_count) % LOG_LINES;
    for (uint8_t i = 0; i < log_count; i++) {
        uint8_t idx = (start + i) % LOG_LINES;
        if (i) json += ',';
        json += '"';
        // Escape quotes and backslashes minimally
        for (const char* p = log_lines[idx]; *p; ++p) {
            if (*p == '"' || *p == '\\') json += '\\';
            json += *p;
        }
        json += '"';
    }
    json += ']';
    server.send(200, "application/json", json);
}

// ── Public API ────────────────────────────────────────────────────────────────

void web_init() {
    WiFi.softAP("FakeFC");   // open AP, no password
    server.on("/",        handle_root);
    server.on("/channels", handle_channels);
    server.on("/log",      handle_log);
    server.begin();
    web_log("web: started AP FakeFC at 192.168.4.1");
}

void web_update() {
    server.handleClient();
}
