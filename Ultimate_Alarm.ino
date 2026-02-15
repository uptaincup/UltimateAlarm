// !!!!!!!!!!!!!!!!!
// doezs not take affect do to a bug either in arduino ide v2 or elegant ota lib. so must be set directly by patching librarys /home/cs4/Arduino/libraries/ElegantOTA/src/ElegantOTA.cpp
// before any includes              
#define ELEGANTOTA_USE_ASYNC_WEBSERVER 1


#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include <ESPAsyncWebServer.h> // by ESP32Async

#include <ElegantOTA.h>
// #include <RemoteDebug.h> // does not support c6 as of now. only c3. they are different arhitecture like riscv and arm
#include <WebSerial.h>

/* ---------------- I/O ---------------- */
constexpr int BUZZER_PIN = 16;
constexpr int BUILT_IN_LED = 15;
constexpr int HAPTIC_PIN = 21;
/* ---------------- WiFi ---------------- */
const char* ssid     = "Our^_^Home";
const char* password = "j/-%O-xo!IRr1Vh@vs";

String ESP32_HOST_ID = "alarm";

/* ---------------- Globals ---------------- */
AsyncWebServer server(80);
// RemoteDebug Debug;

unsigned long ota_progress_millis = 0;

/* ---------------- OTA callbacks ---------------- */
void onOTAStart() {
    Serial.println("OTA update started!");
}

void onOTAProgress(size_t current, size_t final) {
    if (millis() - ota_progress_millis > 1000) {
        ota_progress_millis = millis();
        Serial.printf("OTA Progress: %u / %u bytes\n", current, final);
    }
}

void onOTAEnd(bool success) {
    Serial.println(success ? "OTA success" : "OTA failed");
}

/* ---------------- Setup ---------------- */
void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(BUILT_IN_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(HAPTIC_PIN, OUTPUT);

    Serial.println("Booting...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    /* -------- Web server -------- */
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "ElegantOTA Async OK");
    });

    ElegantOTA.begin(&server);
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    WebSerial.begin(&server);

    server.begin();
    Serial.println("HTTP server started");

    /* -------- mDNS -------- */
    if (!MDNS.begin(ESP32_HOST_ID.c_str())) {
        Serial.println("MDNS failed");
    } else {
        Serial.println("MDNS started");
    }

    /* -------- RemoteDebug -------- */
    // Debug.begin(ESP32_HOST_ID);
    // Debug.setResetCmdEnabled(true);
    // Debug.showProfiler(true);
    // Debug.showColors(true);

    // Serial.println("RemoteDebug ready");
}

/* ---------------- Loop ---------------- */
void loop() {
    ElegantOTA.loop();
    // Debug.handle();
    WebSerial.loop();

    static uint32_t t = 0;
    if (millis() - t > 1000) {
        t = millis();
        digitalWrite(BUILT_IN_LED, !digitalRead(BUILT_IN_LED));
        digitalWrite(HAPTIC_PIN, !digitalRead(HAPTIC_PIN));
        // debugV("alive");
    }

    delay(1);
}
