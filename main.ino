#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Initialize the U8g2 library for the SH1106 OLED display with I2C communication
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Initialize the ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

const int heartRatePin = A0;
const int buzzerPin = 3;
int heartRateThreshold = 1000; // Set your threshold here
int noiseThreshold = 10; // Threshold to filter out noise

// Global variables for step counting
float prevAccelX = 0;
float prevAccelY = 0;
float prevAccelZ = 0;
int stepCount = 0;
float stepThreshold = 1.0; // Adjust this threshold based on your needs

void countSteps(float accelX, float accelY, float accelZ) {
  // Calculate the magnitude of the acceleration vector
  float magnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // Check if the magnitude exceeds the step threshold
  if (magnitude > stepThreshold && prevAccelX < stepThreshold && prevAccelY < stepThreshold && prevAccelZ < stepThreshold) {
    stepCount++;
  }

  // Update previous acceleration values
  prevAccelX = accelX;
  prevAccelY = accelY;
  prevAccelZ = accelZ;
}


void setup() {
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
  sensors_event_t event;
  accel.getEvent(&event);

  int heartRate = analogRead(heartRatePin);
  heartRate = map(heartRate, 0, 1023, 0, 200); // Adjust mapping as needed

  // Apply noise threshold
  if (heartRate < noiseThreshold) {
    heartRate = 0;
  }

  //countSteps(event.acceleration.x, event.acceleration.y, event.acceleration.z);


  // Clear the display buffer
  u8g2.clearBuffer();

  // Set text size and color
  u8g2.setFont(u8g2_font_ncenB08_tr); // Choose a suitable font

  // Display heart rate
  u8g2.setCursor(0, 10);
  u8g2.print("Heart Rate: ");
  u8g2.print(heartRate);
  u8g2.print(" bpm");

  // Display accelerometer data
  u8g2.setCursor(0, 30);
  u8g2.print("Steps: "); u8g2.print(stepCount);
  //u8g2.print("X: "); u8g2.print(event.acceleration.x); u8g2.print(" m/s^2");
  //u8g2.setCursor(0, 40);
  //u8g2.print("Y: "); u8g2.print(event.acceleration.y); u8g2.print(" m/s^2");
  //u8g2.setCursor(0, 50);
  //u8g2.print("Z: "); u8g2.print(event.acceleration.z); u8g2.print(" m/s^2");

  // Update the display with the new data
  u8g2.sendBuffer();

  // Print the same data to the Serial Monitor
  Serial.print("Heart Rate: ");
  Serial.print(heartRate);
  Serial.println(" bpm");
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.println(" m/s^2");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.println(" m/s^2");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.println(" m/s^2");

  // Check if the heart rate exceeds the threshold and activate the buzzer if necessary
  if (heartRate > heartRateThreshold) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  // Add a delay to reduce the frequency of display updates
  delay(1000);
}

