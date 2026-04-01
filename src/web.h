#pragma once

// WiFi access point + HTTP server.
// SSID: "FakeFC", no password.
// Open http://192.168.4.1 in a browser to see live RC channel values.

void web_init();
void web_update();   // call every loop iteration
