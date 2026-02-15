#include <Arduino.h>


/* ---------------- I/O ---------------- */
constexpr int BUZZER_PIN = 16;
constexpr int BUILT_IN_LED = 15;


/* ---------------- Setup ---------------- */
void setup() {
    delay(1000); // 1 sec of silence (after constant beep during flash)

    pinMode(BUILT_IN_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    digitalWrite(BUILT_IN_LED, HIGH); // turn off built-in led. its inverted

    Serial.begin(115200);

    // Beep. Board is alive
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);

    Serial.println("Booted...");
}

void loop() {
}
