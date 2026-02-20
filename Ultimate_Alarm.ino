#include "SparkFunLSM6DS3.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include <Arduino.h>
#include <Wire.h>

// static constexpr uint64_t WAKE_SECONDS = 60*60*12; // 12h
static constexpr uint64_t WAKE_SECONDS = 20; // s

LSM6DS3 myIMU(I2C_MODE, 0x6B);
volatile uint8_t int1Status = 0;

/* ---------------- I/O ---------------- */
constexpr int GPIO0 = 0;
constexpr int IMU_INT_PIN = 1;
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

void IRAM_ATTR int1ISR() {
  // Serial.println("Interrupt serviced.");
  int1Status++;
}

static void printWakeInfo() {
  Serial.printf("WAKE_CAUSE=%d\n", (int)esp_sleep_get_wakeup_cause());
  Serial.printf("EXT1_STATUS=0x%llx\n",
                (unsigned long long)esp_sleep_get_ext1_wakeup_status());
}

static void setupTapInterrupt() {

  // Call .beginCore() to configure the IMU
  if (myIMU.beginCore() != 0) {
    Serial.print("Error at beginCore().\n");
  } else {
    Serial.print("\nbeginCore() passed.\n");
  }

  // Error accumulation variable
  uint8_t errorAccumulator = 0;

  uint8_t dataToWrite = 0; // Temporary variable

  // IMU reset
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, 0x01);
  delay(50);
  do {
    myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL3_C);
  } while (dataToWrite & 0x01);
  do {
    myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL3_C);
  } while (dataToWrite & 0x01);

  // IMPORTANT CHANGE:
  // Make INT active-LOW explicitly (H_LACTIVE=1). Keep push-pull (PP_OD=0).
  uint8_t c3 = 0;
  errorAccumulator += myIMU.readRegister(&c3, LSM6DS3_ACC_GYRO_CTRL3_C);
  c3 |= (uint8_t)0x20;  // H_LACTIVE = 1 (active low)
  c3 &= (uint8_t)~0x10; // PP_OD    = 0 (push-pull)
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, c3);

  uint8_t c3r = 0;
  myIMU.readRegister(&c3r, LSM6DS3_ACC_GYRO_CTRL3_C);
  Serial.printf("CTRL3_C=0x%02X\n", c3r);

  // Setup the accelerometer
  dataToWrite = 0; // Start Fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;

  // //Now, write the patched together data
  errorAccumulator +=
      myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Set the ODR bit
  errorAccumulator +=
      myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
  errorAccumulator +=
      myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);

  // TARGET (enable XYZ tap + enable latch bit)
  // If LIR is bit0 in TAP_CFG1 (common for LSM6DS3), this makes it latched.
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x0F);

  // Set tap threshold
  // Write 0Ch into TAP_THS_6D
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x15); // was 0x03

  // Set Duration, Quiet and Shock time windows
  // Write 7Fh into INT_DUR2
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);

  // Single & Double tap enabled (SINGLE_DOUBLE_TAP = 1)
  // Write 80h into WAKE_UP_THS
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS,
                      0x80); // ToDo:cs4: this what missed in tutoral...

  // Single tap interrupt driven to INT1 pin -- enable latch
  // Write 08h into MD1_CFG
  errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x48);

  if (errorAccumulator) {
    Serial.println("Problem configuring the device.");
  } else {
    Serial.println("Device O.K.");
  }

  // Configure the interrupt pin
  //  ESP pin: idle HIGH expected, interrupt on FALLING (active-low assertion)
  pinMode(IMU_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), int1ISR, FALLING);

  // Clear pending sources
  uint8_t dummy = 0;
  myIMU.readRegister(&dummy, LSM6DS3_ACC_GYRO_TAP_SRC);
#ifdef LSM6DS3_ACC_GYRO_ALL_INT_SRC
  myIMU.readRegister(&dummy, LSM6DS3_ACC_GYRO_ALL_INT_SRC);
#endif

  Serial.printf("IMU_INT_PIN=%d level=%d\n", IMU_INT_PIN,
                digitalRead(IMU_INT_PIN));

  // IMPORTANT CHANGE:
  // Deep sleep wake when pin goes LOW (active-low interrupt)
  esp_sleep_enable_ext1_wakeup(1ULL << IMU_INT_PIN, ESP_EXT1_WAKEUP_ALL_LOW);
}

/* ---------------- Setup ---------------- */
void setup() {

  // Don't let gpios float:
  pinMode(GPIO0, OUTPUT);
  // pinMode(IMU_INT_PIN, OUTPUT);
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
  // digitalWrite(IMU_INT_PIN, LOW);
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

  Serial.begin(115200);

  delay(1000); // 1 sec of silence (after constant beep during flash, and firs
               // consious alive beep)

  printWakeInfo();

  myIMU.begin();
  // Shut down IMU
  // myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x00); // Accelerometer
  // power-down myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, 0x00); //
  // Gyroscope power-down

  setupTapInterrupt();
}

void afterSleepChores() {
  // unhold pinns so they are usable again
  rtc_gpio_hold_dis(G(HAPTIC_PIN));
  rtc_gpio_hold_dis(G(BUZZER_PIN));
}

void beforeSleepChores() {
  // Deep Sleep
  // Enable hold per-pin (required)
  // Pin Must support LP!
  esp_err_t e1 = rtc_gpio_hold_en(G(HAPTIC_PIN));
  esp_err_t e2 = rtc_gpio_hold_en(G(BUZZER_PIN));

  // Force holds during sleep (affects pins that were successfully hold-enabled)
  // rtc_gpio_force_hold_en_all();
}

void loop() {
  if (int1Status > 0) // If ISR has been serviced at least once
  {
    // Wait for a window (in case a second tap is coming)
    delay(300);

    // Check if there are more than one interrupt pulse
    if (int1Status == 1) {
      Serial.print("Single-tap event\n");
    }
    if (int1Status > 1) {
      Serial.print("Double-tap event\n");
    }

    // Clear the ISR counter
    int1Status = 0;
  }

  afterSleepChores();

  beepAlive();
  Serial.println("Booted...");

  // keep wake to have window to connect and re-flash if needed.
  Serial.println("Flash window. tens of seconds.");
  delay(20000);

  Serial.println("Going to sleep for a long time...");

  // Enable Wake by timer
  // esp_sleep_enable_timer_wakeup(WAKE_SECONDS * 1000000ULL);
  esp_sleep_enable_ext1_wakeup(1ULL << IMU_INT_PIN, ESP_EXT1_WAKEUP_ALL_LOW);

  // RTC-domain pullup for stability in deep sleep (matches idle HIGH)
  rtc_gpio_pulldown_dis((gpio_num_t)IMU_INT_PIN);
  rtc_gpio_pullup_en((gpio_num_t)IMU_INT_PIN);

  // Clear any latched tap from earlier so INT line is not stuck active
  uint8_t dummy = 0;
  myIMU.readRegister(&dummy, LSM6DS3_ACC_GYRO_TAP_SRC);

#ifdef LSM6DS3_ACC_GYRO_ALL_INT_SRC
  myIMU.readRegister(&dummy, LSM6DS3_ACC_GYRO_ALL_INT_SRC);
#endif
  Serial.printf("INT1 pre-sleep level=%d\n", digitalRead(IMU_INT_PIN));

  beforeSleepChores();
  // Sleep
  // esp_light_sleep_start();
  esp_deep_sleep_start();
}
