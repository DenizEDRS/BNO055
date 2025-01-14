#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

// Pin definitions for your board
#define TFT_CS        7    // GPIO7 - Chip select
#define TFT_DC        39   // GPIO39 - Data/Command
#define TFT_RESET     40   // GPIO40 - Reset
#define TFT_BACKLIGHT 45   // GPIO45 - Backlight control

// Create the display object
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RESET);

// Initialize the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Samplerate delay
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);

  // Initialize the display
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH); // Turn on backlight
  tft.init(135, 240); // Initialize for 240x135 resolution
  tft.setRotation(3); // Adjust orientation
  tft.fillScreen(ST77XX_BLACK);

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.println("BNO055 Error!");
    while (1);
  }

  bno.setExtCrystalUse(true); // Use external crystal for better accuracy

  // Wait for full calibration
  uint8_t system, gyro, accel, mag;
  while (true) {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("Calibration Status - Sys: ");
    Serial.print(system);
    Serial.print(", Gyro: ");
    Serial.print(gyro);
    Serial.print(", Accel: ");
    Serial.print(accel);
    Serial.print(", Mag: ");
    Serial.println(mag);

    // Display calibration status on the TFT
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(0,70);
    tft.printf("Calib Sys:%d G:%d A:%d M:%d", system, gyro, accel, mag);

    imu::Vector<3> accelData = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    Serial.print("Accel X: "); Serial.println(accelData.x());
    Serial.print("Accel Y: "); Serial.println(accelData.y());
    Serial.print("Accel Z: "); Serial.println(accelData.z());

    if (system == 3 && gyro == 3 && accel == 3 && mag == 3) {
      Serial.println("Calibration complete!");
      break;
    }

    delay(500); // Check calibration status every 500ms
  }

  // Optional: Retrieve and print calibration offsets for verification
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);

  Serial.println("Calibration offsets:");
  
  Serial.print("Accel Offset X: "); Serial.println(offsets.accel_offset_x);
  Serial.print("Accel Offset Y: "); Serial.println(offsets.accel_offset_y);
  Serial.print("Accel Offset Z: "); Serial.println(offsets.accel_offset_z);

  Serial.print("Gyro Offset X: "); Serial.println(offsets.gyro_offset_x);
  Serial.print("Gyro Offset Y: "); Serial.println(offsets.gyro_offset_y);
  Serial.print("Gyro Offset Z: "); Serial.println(offsets.gyro_offset_z);

  Serial.print("Mag Offset X: "); Serial.println(offsets.mag_offset_x);
  Serial.print("Mag Offset Y: "); Serial.println(offsets.mag_offset_y);
  Serial.print("Mag Offset Z: "); Serial.println(offsets.mag_offset_z);
}

void loop() {
  // Get sensor data
  imu::Vector<3> accelData = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyroData = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> magData = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Clear the screen before drawing
  tft.fillScreen(ST77XX_BLACK);

  // Draw Accelerometer Data (A)
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("a");
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.printf("x: %.2f", accelData.x());
  tft.setCursor(10, 60);
  tft.printf("y: %.2f", accelData.y());
  tft.setCursor(10, 80);
  tft.printf("z: %.2f", accelData.z());

  // Draw Gyroscope Data (B)
  tft.setTextSize(2);
  tft.setCursor(90, 10);
  tft.println("w");
  tft.setTextSize(1);
  tft.setCursor(90, 40);
  tft.printf("x: %.2f", gyroData.x());
  tft.setCursor(90, 60);
  tft.printf("y: %.2f", gyroData.y());
  tft.setCursor(90, 80);
  tft.printf("z: %.2f", gyroData.z());

  // Draw Magnetometer Data (C)
  tft.setTextSize(2);
  tft.setCursor(170, 10);
  tft.println("B");
  tft.setTextSize(1);
  tft.setCursor(170, 40);
  tft.printf("x: %.2f", magData.x());
  tft.setCursor(170, 60);
  tft.printf("y: %.2f", magData.y());
  tft.setCursor(170, 80);
  tft.printf("z: %.2f", magData.z());

  // Draw Units at the Bottom
  tft.setTextSize(1);
  tft.setCursor(10, 110);
  tft.println("Accel: m/s^2");

  tft.setCursor(90, 110);
  tft.println("Gyro: rad/s");

  tft.setCursor(170, 110);
  tft.println("Mag: uT");


  // Print Accelerometer Data to Serial Console
  Serial.println("Accelerometer Data:");
  Serial.print("  X: "); Serial.println(accelData.x(), 2);
  Serial.print("  Y: "); Serial.println(accelData.y(), 2);
  Serial.print("  Z: "); Serial.println(accelData.z(), 2);



  // Print Gyroscope Data to Serial Console
  Serial.println("Gyroscope Data:");
  Serial.print("  X: "); Serial.println(gyroData.x(), 2);
  Serial.print("  Y: "); Serial.println(gyroData.y(), 2);
  Serial.print("  Z: "); Serial.println(gyroData.z(), 2);


    //Serial outputs
    // Print Magnetometer Data to Serial Console
  Serial.println("Magnetometer Data:");
  Serial.print("  X: "); Serial.println(magData.x(), 2);
  Serial.print("  Y: "); Serial.println(magData.y(), 2);
  Serial.print("  Z: "); Serial.println(magData.z(), 2);

  // Add a blank line to separate readings
  Serial.println();




  // Update every delay period
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
