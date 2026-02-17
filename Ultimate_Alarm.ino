#include <Arduino.h>
#include "SparkFunLSM6DS3.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

// static constexpr uint64_t WAKE_SECONDS = 60*60*12; // 12h
static constexpr uint64_t WAKE_SECONDS = 20; // s


LSM6DS3 myIMU( I2C_MODE, 0x6B );


/* ---------------- I/O ---------------- */
constexpr int GPIO0 = 0;
constexpr int GPIO1 = 1;
constexpr int GPIO2 = 2;
constexpr int GPIO21 = 21;
constexpr int SCL_PIN = 22;
constexpr int SDA_PIN = 23;
constexpr int BUILT_IN_LED = 15;
constexpr int GPIO16 = 16;
constexpr int GPIO17 = 17;
constexpr int GPIO19 = 19;
constexpr int GPIO20 = 20;
constexpr int GPIO18 = 18;
constexpr int BUZZER_PIN = 7;
constexpr int HAPTIC_PIN = 5;
constexpr int GPIO6 = 6;
constexpr int GPIO4 = 4;


static inline gpio_num_t G(int pin) { return (gpio_num_t)pin; }

void beepAlive() { // Beep. Board is alive
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);

} 

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
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(HAPTIC_PIN, OUTPUT);
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
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(HAPTIC_PIN, LOW);
    digitalWrite(GPIO6, LOW);
    digitalWrite(GPIO4, LOW);

    Wire.begin(SDA_PIN, SCL_PIN);


    // Shut down IMU
    myIMU.begin();
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00); // Accelerometer power-down
    myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00); // Gyroscope power-down



    Serial.begin(115200);

    delay(1000); // 1 sec of silence (after constant beep during flash, and firs consious alive beep)

}

void afterSleepChores(){
    // unhold pinns so they are usable again
    rtc_gpio_hold_dis(G(HAPTIC_PIN));
    rtc_gpio_hold_dis(G(BUZZER_PIN));
}

void beforeSleepChores(){
    // Deep Sleep
    // Enable hold per-pin (required)
    // Pin Must support LP!
    esp_err_t e1 = rtc_gpio_hold_en(G(HAPTIC_PIN));
    esp_err_t e2 = rtc_gpio_hold_en(G(BUZZER_PIN));

    // Force holds during sleep (affects pins that were successfully hold-enabled)
    rtc_gpio_force_hold_en_all();
}



void loop() {

    afterSleepChores();

    beepAlive();
    Serial.println("Booted...");

    // keep wake to have window to connect and re-flash if needed.    
    Serial.println("Flash window. tens of seconds.");    
    delay(20000);
    
    Serial.println("Going to sleep for a long time...");    
    esp_sleep_enable_timer_wakeup(WAKE_SECONDS * 1000000ULL);

    beforeSleepChores();
    // Sleep    
    // esp_light_sleep_start();
    esp_deep_sleep_start();

}

