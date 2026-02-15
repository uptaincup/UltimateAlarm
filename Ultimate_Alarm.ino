#include <Arduino.h>
#include "SparkFunLSM6DS3.h"

LSM6DS3 myIMU( I2C_MODE, 0x6B );


/* ---------------- I/O ---------------- */
constexpr int GPIO0 = 0;
constexpr int GPIO1 = 1;
constexpr int GPIO2 = 2;
constexpr int HAPTIC_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr int SDA_PIN = 23;
constexpr int BUILT_IN_LED = 15;
constexpr int BUZZER_PIN = 16;
constexpr int GPIO17 = 17;
constexpr int GPIO19 = 19;
constexpr int GPIO20 = 20;
constexpr int GPIO18 = 18;
constexpr int GPIO7 = 7;
constexpr int GPIO5 = 5;
constexpr int GPIO6 = 6;
constexpr int GPIO4 = 4;



/* ---------------- Setup ---------------- */
void setup() {

    // Don't let gpios float:
    pinMode(GPIO0, OUTPUT);
    pinMode(GPIO1, OUTPUT);
    pinMode(GPIO2, OUTPUT);
    pinMode(HAPTIC_PIN, OUTPUT);
//     pinMode(SCL_PIN, OUTPUT);
//     pinMode(SDA_PIN, OUTPUT);
    pinMode(BUILT_IN_LED, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(GPIO17, OUTPUT);
    pinMode(GPIO19, OUTPUT);
    pinMode(GPIO20, OUTPUT);
    pinMode(GPIO18, OUTPUT);
    pinMode(GPIO7, OUTPUT);
    pinMode(GPIO5, OUTPUT);
    pinMode(GPIO6, OUTPUT);
    pinMode(GPIO4, OUTPUT);

    digitalWrite(GPIO0, LOW);
    digitalWrite(GPIO1, LOW);
    digitalWrite(GPIO2, LOW);
    digitalWrite(HAPTIC_PIN, LOW);
//     digitalWrite(SCL_PIN, LOW);
//     digitalWrite(SDA_PIN, LOW);
    digitalWrite(BUILT_IN_LED, HIGH); // turn off built-in led. its inverted
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(GPIO17, LOW);
    digitalWrite(GPIO19, LOW);
    digitalWrite(GPIO20, LOW);
    digitalWrite(GPIO18, LOW);
    digitalWrite(GPIO7, LOW);
    digitalWrite(GPIO5, LOW);
    digitalWrite(GPIO6, LOW);
    digitalWrite(GPIO4, LOW);

    Wire.begin(SDA_PIN, SCL_PIN);


    // Shut down IMU
    myIMU.begin();
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00); // Accelerometer power-down
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00); // Gyroscope power-down



    Serial.begin(115200);


    delay(1000); // 1 sec of silence (after constant beep during flash)


    // Beep. Board is alive
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);


    Serial.println("Booted...");
}

void loop() {
}
