#include "web.h"
#include "crsf.h"
#include <WiFi.h>
#include <WebServer.h>

static WebServer server(80);

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
</style>
</head>
<body>
<h2>RC Channels</h2>
<div id="channels"></div>
<script>
const N = 16;
const MIN = 1000, MAX = 2000, RANGE = MAX - MIN;
const div = document.getElementById('channels');

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
</script>
</body>
</html>
)";

// ── Handlers ──────────────────────────────────────────────────────────────────

static void handle_root() {
    server.send_P(200, "text/html", PAGE);
}

static void handle_channels() {
    const uint16_t* ch = crsf_get_channels();
    String json = "[";
    for (uint8_t i = 0; i < CRSF_CHANNEL_COUNT; i++) {
        if (i) json += ',';
        json += ch[i];
    }
    json += ']';
    server.send(200, "application/json", json);
}

// ── Public API ────────────────────────────────────────────────────────────────

void web_init() {
    WiFi.softAP("FakeFC");   // open AP, no password
    server.on("/",        handle_root);
    server.on("/channels", handle_channels);
    server.begin();
}

void web_update() {
    server.handleClient();
}
