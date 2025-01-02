#include <BH1750.h>
#include <Wire.h>
#include <NewPing.h>
#include <math.h>
#include <PZEM004Tv30.h>
#include <SoftwareSerial.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>

Adafruit_ADS1115 ads48;  // First ADS1115 at address 0x48 to sensor board
Adafruit_ADS1115 ads4A;  // Second ADS1115 at address 0x4A to main board

LiquidCrystal_I2C lcd1(0x27, 16, 2);  // Default address for LCD 1
LiquidCrystal_I2C lcd2(0x26, 16, 2);  // Address for LCD 2 (A0 set to HIGH)

int stepPin = 5;       // Motor 1 Step Pin arm a1
int directionPin = 4;  // Motor 1 Direction Pin arm
int stpPin = 3;        // Motor 2 Step Pin light a2
int dirctionPin = 2;   // Motor 2 Direction Pinlight
const int relayPin = 8;

#define trig 9
#define echo 10
#define PZEM_RX_PIN 12
#define PZEM_TX_PIN 13

const float b = 2;            // Half the width of the light source (in meters)
const float h = 3;            // Height of the light source (in meters)
const float sensor_area = 6;  // Surface area of the sensor (in square meters)
const float fluxarea = 6;

SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem(pzemSWSerial);

BH1750 lightmeter;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);

float responsivity = 0.5;  // V/W·m² from datasheet for ir
float Maxtemperature = 65.0;
const float uvScaleFactor = 307;     // Typically, 1 V equals 307 mW/m² of UV intensity
const float uvToIndexFactor = 0.05;  // Conversion factor from mW/m² to UV index
const int stepDelay = 500;

//light angle senor
float readAngle() {
  // Read raw ADC value from channel 0 of the ADS1115
  int16_t sensrValue = ads4A.readADC_SingleEnded(0);
  // Convert raw ADC value to voltage (assuming default gain of ±6.144V)
  float voltage1r = sensrValue * 0.076295;  //
  // Map voltage (0-5V) to angle (0-360°)
  float angl = map(voltage1r * 1000, 0, 5000, 0, 360);  // Convert V to mV for map
  return angl;
}

//arm angle sensor
float readAngle2() {
  // Read raw ADC value from channel 1 of the ADS1115
  int16_t sensorValue = ads4A.readADC_SingleEnded(1);
  // Convert raw ADC value to voltage (assuming default gain of ±6.144V)
  float voltage2r = sensorValue * 0.076295;  // 0.1875mV per bit
  // Map voltage (0-5V) to angle (0-360°)
  float angle = map(voltage2r * 1000, 0, 5000, 0, 360);  // Convert V to mV for map
  return angle;
}

void stepMotor(bool clockwise) {
  digitalWrite(dirctionPin, clockwise ? HIGH : LOW);
  digitalWrite(stpPin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stpPin, LOW);
  delayMicroseconds(stepDelay);
}

void stpMotor(bool clockwise) {
  digitalWrite(directionPin, clockwise ? HIGH : LOW);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelay);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lightmeter.begin();
  tcs.begin();

  pinMode(stepPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(stpPin, OUTPUT);
  pinMode(dirctionPin, OUTPUT);
  pinMode(relayPin, OUTPUT);

  lcd1.begin(16, 2);
  lcd1.backlight();  // Turn on backlight for LCD 1
  lcd1.setCursor(0, 0);
  lcd1.print("Please wait...");
  lcd2.begin(16, 2);
  lcd2.backlight();  // Turn on backlight for LCD 2
  lcd2.setCursor(0, 0);
  lcd2.print("Please wait...");
  delay(1000);

  float targetAngl = 100.0;
  float currentAngl = readAngle();
  float targetAngle = 30.0;
  float currentAngle = readAngle2();

  float luxvalue[30];
  float diffluxvalue[30];
  float difflxvalue[30];
  float totalluxstep[30];
  float uvofflight[30];
  float irofflight[30];
  float uvvalue[30];
  float irvalue[30];
  float totaluvstep[30];
  float totalirstep[30];
  float avgluvstep[30];
  float avglirstep[30];
  float totalintensitystep[30];
  float averageluxstep[30];
  float averageintensitystep[30];
  float luxvalueofflight[30];
  float power[10];
  float averageluxlight = 0.0;
  float totalluxonlight = 0.0;
  float totalintensity = 0.0;
  float bulbintensity = 0.0;
  float totaluvoff = 0.0;
  float avguvoff = 0.0;
  float totaliroff = 0.0;
  float avgiroff = 0.0;
  float avguvon = 0.0;
  float avgiron = 0.0;
  float totaluvintensity = 0.0;
  float totalirintensity = 0.0;
  float uvintensity = 0.0;
  float irintensity = 0.0;
  float totalpower = 0.0;
  float avgpower = 0.0;
  float flux = 0.0;
  float efficiency = 0.0;

  long distance_mm[20];
  float distance[20];
  long totaldistance = 0.0;
  float light_intensity[30];
  float denominator[30];
  float solid_angle[30];
  float avgdistance[30];
  delay(1000);
  digitalWrite(relayPin, LOW);

  delay(50);

  //light move
  
  // while (true);

  delay(500);

  for (int i = 0; i < 30; i++) {
    luxvalueofflight[i] = 0.0;
    uvofflight[i] = 0.0;
    irofflight[i] = 0.0;

    float luxofflight = lightmeter.readLightLevel();
    luxvalueofflight[i] = luxofflight;

    delay(300);
    // Read LM35 temperature sensor (Channel 2 of 0x48)
    int16_t uvRawoff = ads48.readADC_SingleEnded(2);
    float uvVoltage = uvRawoff * 0.076295 / 1000.0;  //v
    // Convert the voltage to UV intensity (mW/m²)
    float uvIntensityoff = uvVoltage * uvScaleFactor;
    // Convert UV intensity to UV index
    float uvIndexoff = uvIntensityoff * uvToIndexFactor;
    uvofflight[i] = uvIntensityoff * 0.1;  //uW/cm²
    totaluvoff += uvofflight[i];

    delay(300);
    // Read LM35 temperature sensor (Channel 0 of 0x48)
    int16_t irRawoff = ads48.readADC_SingleEnded(0);
    float irVoltageoff = irRawoff * 0.076295 / 1000.0;   //v
    float irIntensityoff = irVoltageoff * responsivity;  //W/m²
    irofflight[i] = irIntensityoff * 100.0;              //uW/cm²
    totaliroff += irofflight[i];
    delay(300);

    digitalWrite(directionPin, HIGH);
    for (float stepCount = 1; stepCount <= 88.89; stepCount++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
    delay(500);
  }

  avguvoff = totaluvoff / 30.0;
  avgiroff = totaliroff / 30.0;


  for (int i = 0; i < 30; i++) {
    totalluxstep[i] = 0.0;
    averageluxstep[i] = 0.0;
  }
  delay(1000);

  while (int(currentAngle) != int(targetAngle)) {
    currentAngle = readAngle2();

    if (currentAngle < targetAngle) {
      stpMotor(true);  // Move clockwise
    } else if (currentAngle > targetAngle) {
      stpMotor(false);  // Move counterclockwise
    }
  }
  // while (true);
  delay(1000);
  digitalWrite(relayPin, HIGH);
  delay(500);

  for (int i = 0; i < 10; i++) {
    while (int(currentAngle) != int(targetAngle)) {
      currentAngle = readAngle2();

      if (currentAngle < targetAngle) {
        stpMotor(true);  //  arm Move clockwise
      } else if (currentAngle > targetAngle) {
        stpMotor(false);  // Move counterclockwise
      }
    }

    // while (true);
    delay(500);

    for (int i = 0; i < 30; i++) {
      luxvalue[i] = 0.0;
      light_intensity[i] = 0.0;
      uvvalue[i] = 0.0;
      irvalue[i] = 0.0;

      float lux = lightmeter.readLightLevel();
      luxvalue[i] = lux;
      totalluxstep[i] += luxvalue[i];
      difflxvalue[i] = luxvalue[i] - luxvalueofflight[i];
      distance_mm[i] = 0.0;
      distance[i] = 0.0;
      totaldistance = 0.0;
      light_intensity[i] = 0.0;
      denominator[i] = 0.0;
      solid_angle[i] = 0.0;
      avgdistance[i] = 0.0;

      for (int i = 0; i < 20; i++) {
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);

        // Measure the time taken by the echo
        long t = pulseIn(echo, HIGH, 30000);
        // Calculate distance in millimeters
        distance_mm[i] = (t * 343) / 2000;  // Speed of sound is 343 m/s,** divide by 2000 to get mm
        totaldistance += distance_mm[i];
        delay(60);
      }
      avgdistance[i] = (totaldistance / 20000.0) - 0.01;
      denominator[i] = sqrt((avgdistance[i] * avgdistance[i] + 4 * h * h) * (b * b + 4 * h * h));
      solid_angle[i] = 4 * asin((avgdistance[i] * b) / denominator[i]);
      light_intensity[i] = (difflxvalue[i] * sensor_area) / solid_angle[i];
      totaldistance = 0.0;
      totalintensitystep[i] += light_intensity[i];

      delay(300);
      // Read LM35 temperature sensor (Channel 2 of 0x48)
      int16_t uvRawon = ads48.readADC_SingleEnded(2);
      float uvVoltageon = uvRawon * 0.076295 / 1000.0;  //v
                                                        // Convert the voltage to UV intensity (mW/m²)
      float uvIntensityon = uvVoltageon * uvScaleFactor;
      // Convert UV intensity to UV index
      float uvIndex = uvIntensityon * uvToIndexFactor;
      uvvalue[i] = uvIntensityon * 0.1;
      totaluvstep[i] += uvvalue[i];

      delay(300);
      // Read LM35 temperature sensor (Channel 0 of 0x48)
      int16_t irRawon = ads48.readADC_SingleEnded(0);
      float irVoltageon = irRawon * 0.076295 / 1000.0;   //v
      float irIntensityon = irVoltageon * responsivity;  //w/m²
      irvalue[i] = irIntensityon * 100.0;
      totalirstep[i] += irvalue[i];
      delay(300);

      // Read LM35 temperature sensor (Channel 1 of 0x48)
      int16_t lm35Raw = ads48.readADC_SingleEnded(1);
      float lm35Voltage = lm35Raw * 0.076295 / 1000;  // Convert raw ADC value to voltage
      float temperatureC = lm35Voltage * 100;         // Convert voltage to temperature (°C)

      if (temperatureC > 65) {
        digitalWrite(relayPin, LOW);
        return;
      }

      delay(300);

      digitalWrite(directionPin, HIGH);
      for (float stepCount = 1; stepCount <= 88.89; stepCount++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(500);
      }
      delay(500);
    }

    digitalWrite(directionPin, LOW);
    for (float stepCount = 1; stepCount <= 2666.67; stepCount++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
    delay(500);

    digitalWrite(dirctionPin, HIGH);
    for (float stpCount = 1; stpCount <= 160.0; stpCount++) {
      digitalWrite(stpPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stpPin, LOW);
      delayMicroseconds(500);
    }

    power[i] = pzem.power();
    totalpower += power[i];

    delay(500);
  }
  digitalWrite(dirctionPin, LOW);
  for (float stpCount = 1; stpCount <= 1600.0; stpCount++) {
    digitalWrite(stpPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stpPin, LOW);
    delayMicroseconds(500);
  }
  delay(500);
  digitalWrite(directionPin, HIGH);
  for (float stepCount = 1; stepCount <= 1333.33; stepCount++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }

  for (int i = 0; i < 30; i++) {
    averageluxstep[i] = totalluxstep[i] / 10.0;
    diffluxvalue[i] = averageluxstep[i] - luxvalueofflight[i];
    totalluxonlight += diffluxvalue[i];
    averageintensitystep[i] = totalintensitystep[i] / 10.0;
    totalintensity += averageintensitystep[i];
    avgluvstep[i] = totaluvstep[i] / 10.0;
    totaluvintensity += avgluvstep[i];
    avglirstep[i] = totalirstep[i] / 10.0;
    totalirintensity += avglirstep[i];
  }

  uint16_t r, g, b, c, colorTemp;

  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  float lux2 = tcs.calculateLux(r, g, b);


  avguvon = totaluvintensity / 30.0;
  avgiron = totalirintensity / 30.0;

  averageluxlight = totalluxonlight / 30.0;  //lx
  bulbintensity = totalintensity / 30.0;     //cd
  uvintensity = avguvon - avguvoff;          //uW/cm²
  irintensity = avgiron - avgiroff;          //uW/cm²
  avgpower = totalpower / 10.0;              //w
  flux = averageluxlight * fluxarea;         //lm
  efficiency = (flux * 100.0) / avgpower;

  for (int i = 0; i < 30; i++) {
    Serial.print("Light Intensity: ");
    Serial.print(averageintensitystep[i]);
    Serial.println(" cd");
    delay(1000);
  }

  digitalWrite(relayPin, LOW);
  delay(1000);

  while (true) {
    lcd2.clear();
    lcd2.setCursor(0, 0);
    lcd2.print("Light Intensity ");
    lcd2.setCursor(5, 1);
    lcd2.print(bulbintensity);
    lcd2.print(" cd");
    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("Illuminance ");
    lcd1.setCursor(5, 1);
    lcd1.print(averageluxlight);
    lcd1.print(" Lux");
    delay(3000);

    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("UV Intensity ");
    lcd1.setCursor(5, 1);
    lcd1.print(uvintensity);
    lcd1.print(" ");
    lcd1.write(byte(223));  // µ character (micro)
    lcd1.print("W/cm");
    lcd1.write(2);  // ² character (superscript 2)
    delay(3000);

    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("IR Intensity ");
    lcd1.setCursor(5, 1);
    lcd1.print(irintensity);
    lcd1.print(" ");
    lcd1.write(byte(223));  // µ character (micro)
    lcd1.print("W/cm");
    lcd1.write(2);  // ² character (superscript 2)
    delay(3000);

    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("Luminous Flux ");
    lcd1.setCursor(5, 1);
    lcd1.print(flux);
    lcd1.print(" lm");
    delay(1500);
    lcd2.clear();
    lcd2.setCursor(0, 0);
    lcd2.print("Efficiency ");
    lcd2.setCursor(5, 1);
    lcd2.print(efficiency);
    lcd2.print("%");  // Print the % symbol
    delay(1500);

    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("Power of lamp ");
    lcd1.setCursor(5, 1);
    lcd1.print(avgpower);
    lcd1.print(" W");
    delay(3000);

    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("R: ");
    lcd1.setCursor(2, 0);
    lcd1.print(r, DEC);
    lcd1.print(" ");
    lcd1.setCursor(7, 0);
    lcd1.print("G: ");
    lcd1.setCursor(9, 0);
    lcd1.print(g, DEC);
    lcd1.print(" ");
    lcd1.setCursor(0, 1);
    lcd1.print("B: ");
    lcd1.setCursor(2, 1);
    lcd1.print(b, DEC);
    lcd1.print(" ");
    delay(3000);

    lcd1.clear();
    lcd1.setCursor(0, 0);
    lcd1.print("Color Temp: ");
    lcd1.setCursor(5, 1);
    lcd1.print(colorTemp, DEC);
    lcd1.print(" K ");
    delay(3000);
  }
}

void loop() {
}