#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Initialize the U8g2 library for the SH1106 OLED display with I2C communication
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Initialize the ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Pins for the heart rate sensor and buzzer
const int heartRatePin = A0;
const int buzzerPin = 3;

// Variables for heart rate monitoring
int dangerHeartRateH = 100; // Threshold for a dangerous heart rate
int dangerHeartRateL = 60; // Threshold for a low heart rate
int heartRateThreshold = 2000; // Threshold to detect a heart rate pulse
int noiseThreshold = 10; // Threshold to filter out noise
unsigned long lastBeatTime = 0; // Time of the last heart rate pulse
int beatCount = 0; // Number of heart rate pulses detected
float heartRate = 0; // Calculated heart rate

// Variables for step counting
float prevAccelX = 0; 
float prevAccelY = 0; 
float prevAccelZ = 0;
int stepCount = 0; // Number of steps detected
float stepThreshold = 12.0; // Sensitivity of the step detection

// Function to count steps based on accelerometer data
void countSteps(float accelX, float accelY, float accelZ) {
  // Calculate the magnitude of the acceleration vector
  float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  //stepCount = magnitude;

  // Check if the magnitude exceeds the step threshold
  if (magnitude > stepThreshold) {
    stepCount++;
  }

  // Update previous acceleration values
  prevAccelX = accelX;
  prevAccelY = accelY;
  prevAccelZ = accelZ;
}


void setup() {
  // Initialize the Serial Monitor
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);

  // Initialize the U8g2 library
  u8g2.begin();

  // Initialize the ADXL345
  if (!accel.begin()) {
    Serial.println("No ADXL345 detected");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
}

void loop() {
  // Read accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);

  // Read heart rate sensor data
  int heartRate = analogRead(heartRatePin);
  heartRate = map(heartRate, 0, 1023, 0, 200);

  int sensorValue = analogRead(heartRatePin);

  // Apply a simple noise filter
  if (sensorValue > noiseThreshold) {
    Serial.println(sensorValue);
    unsigned long currentTime = millis();
    if (sensorValue > heartRateThreshold) {
      if (currentTime - lastBeatTime > 250) { // Debounce the signal
        beatCount++;
        lastBeatTime = currentTime;
      }
    }

    // Calculate heart rate every 10 beats
    if (beatCount == 10) {
      unsigned long timeInterval = currentTime - lastBeatTime;
      heartRate = (60000.0 / timeInterval) * 10;
      beatCount = 0;
    }
  }

  // Count steps based on accelerometer data
  countSteps(event.acceleration.x, event.acceleration.y, event.acceleration.z);


  // Clear the display buffer
  u8g2.clearBuffer();

  // Set text size and color
  u8g2.setFont(u8g2_font_ncenB08_tr); // Choose a suitable font

  // Display heart rate in console, used for testing
  u8g2.setCursor(0, 10);
  u8g2.print("Heart Rate: ");
  u8g2.print(heartRate);
  u8g2.print(" bpm");

  // Display accelerometer data in console, used for testing
  u8g2.setCursor(0, 30);
  u8g2.print("Steps: "); u8g2.print(stepCount);
  //u8g2.print("X: "); u8g2.print(event.acceleration.x); u8g2.print(" m/s^2");
  //u8g2.setCursor(0, 40);
  //u8g2.print("Steps: "); u8g2.print(stepCounts);
  //u8g2.print("Y: "); u8g2.print(event.acceleration.y); u8g2.print(" m/s^2");
  //u8g2.setCursor(0, 50);
  //u8g2.print("Z: "); u8g2.print(event.acceleration.z); u8g2.print(" m/s^2");

  // Update the display with the new data and clear the buffer
  u8g2.sendBuffer();

  // Print the same data to the Serial Monitor for debugging
  //Serial.print("Heart Rate: ");
  // Serial.print(heartRate);
  // Serial.println(" bpm");
  //Serial.print("X: "); Serial.print(event.acceleration.x); Serial.println(" m/s^2");
  //Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.println(" m/s^2");
  //Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.println(" m/s^2");

  // Check if the heart rate exceeds the threshold and activate the buzzer if necessary
if (heartRate > dangerHeartRateH || heartRate < dangerHeartRateL) {
  digitalWrite(buzzerPin, HIGH);
} else {
  digitalWrite(buzzerPin, LOW);
}

  // Add a delay to reduce the frequency of display updates
  delay(1000);
}

