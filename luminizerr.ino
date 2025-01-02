#include <Wire.h>
#include <BH1750.h>
#include <NewPing.h>
#include <math.h>
#include "Adafruit_TCS34725.h"
#include <ADS1115_WE.h>

#define I2C_ADDRESS 0x48
// BH1750 setup
BH1750 lightMeter;
const float x = 10.0; // Constant for each lux reading

// Initialize the TCS34725 RGB sensor
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

// Initialize the ADS1115 sensor
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

// HC-SR04 setup
#define TRIG_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 400 // Maximum measurable distance in cm

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// Constants
const float b = 2; // Half the width of the light source (in meters)
const float h = 3; // Height of the light source (in meters)
const float sensor_area = 6; // Surface area of the sensor (in square meters)

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize BH1750
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("BH1750 initialized successfully");
  } else {
    Serial.println("Error initializing BH1750");
    while (true);
  }

  // Initialize the RGB sensor
  if (tcs.begin()) {
    Serial.println("TCS34725 sensor found");
  } else {
    Serial.println("No TCS34725 sensor found ... check your connections");
    while (true);
  }

  // Initialize HC-SR04
  Serial.println("HC-SR04 initialized");

  // Initialize the ADS1115 sensor
  if (!adc.init()) {
    Serial.println("ADS1115 not connected!");
    while (true);
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setMeasureMode(ADS1115_CONTINUOUS);

  Serial.println("Setup complete.");
}

void loop() {
  // Variables for averaging lux
  const int numMeasurements = 12;
  float totalIntensity = 0;

  // Take 12 measurements for lux
  for (int i = 0; i < numMeasurements; i++) {
    float lux = lightMeter.readLightLevel();
    if (lux < 0) {
      Serial.println("Error reading light level.");
    } else {
      lux += x; // Add the constant value
      totalIntensity += lux;
    }
    delay(100);
  }

  // Variables for RGB sensor
  uint16_t r, g, b, c, colorTemp, luxRGB;
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  luxRGB = tcs.calculateLux(r, g, b);

  // Variables for ADS1115 sensor
  float voltageIR = readChannel(ADS1115_COMP_0_GND);
  float voltageTemp = readChannel(ADS1115_COMP_1_GND);
  float voltageUV = readChannel(ADS1115_COMP_2_GND);

  // Get distance from HC-SR04
  float distance_cm = sonar.ping_cm();
  if (distance_cm == 0) {
    Serial.println("No object detected");
    return;
  }

  // Convert distance to meters
  float l = distance_cm / 100.0;

  // Ensure inputs are valid
  if (l <= 0 || b <= 0 || h <= 0) {
    Serial.println("Error: Invalid dimensions (l, b, h must be positive and non-zero)");
    return;
  }

  // Calculate solid angle (Ω) using the provided formula
  float denominator = sqrt((l * l + 4 * h * h) * (b * b + 4 * h * h));
  float solid_angle = 4 * asin((l * b) / denominator);

  // Calculate luminous intensity (I)
  float averageIntensity = totalIntensity / numMeasurements;
  float light_intensity = (averageIntensity * sensor_area) / solid_angle;

  // Print results
  Serial.print("Average Light Intensity: ");
  Serial.print(averageIntensity);
  Serial.println(" lux");

  Serial.print("Distance (l): ");
  Serial.print(l, 3);
  Serial.println(" m");

  Serial.print("Solid Angle (Ω): ");
  Serial.print(solid_angle, 6);
  Serial.println(" sr");

  Serial.print("Light Intensity (I): ");
  Serial.print(light_intensity);
  Serial.println(" cd");

  // Print RGB sensor readings
  Serial.print("Color Temp: ");
  Serial.print(colorTemp);
  Serial.print(" K, Lux: ");
  Serial.print(luxRGB);
  Serial.print(", R: ");
  Serial.print(r);
  Serial.print(", G: ");
  Serial.print(g);
  Serial.print(", B: ");
  Serial.print(b);
  Serial.print(", C: ");
  Serial.println(c);

  // Print ADS1115 sensor readings
  Serial.print("IR: ");
  Serial.print(voltageIR);
  Serial.print(" mV, Temp: ");
  Serial.print(voltageTemp);
  Serial.print(" mV, UV: ");
  Serial.print(voltageUV);
  Serial.println(" mV");

  delay(1000); // Update every second
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_mV(); // alternative: getResult_mV for Millivolt
  return voltage;
}
