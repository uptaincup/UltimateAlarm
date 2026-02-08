// // Include necessary libraries
#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc; // Create an RTC object
constexpr int SCL_PIN = 3;
constexpr int SDA_PIN = 4;

void setup () {
  Serial.begin(57600); // Start serial communication
  Wire.begin(SDA_PIN, SCL_PIN);; // Initialize I2C communication
  delay(2000);
  
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while ( 1); // Stop the program here if the RTC is not found
  }

  // Check if the RTC lost power and set the time if needed
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // Set the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // You can also set a specific date and time like this:
    // rtc.adjust(DateTime(2025, 2, 7, 2, 19, 0)); // Year, Month, Day, Hour, Min, Sec
  }
}

void loop () {
  DateTime now = rtc.now(); // Get the current date and time

  // Print the date and time to the Serial Monitor
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(now.dayOfTheWeek()); // Print the day of the week
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();

  // Print the t emperature (DS3231 feature)
  Serial.print("Temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");

  delay(3000); // Wait for 3 seconds before the next reading
}
 
