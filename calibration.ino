/* ----------- Required libaries --------------*/
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_BMP280.h>
#include <SparkFun_SCD30_Arduino_Library.h>

/* ---------------- Lora ----------------- */
// LoRa pins
// Don't change these
const int csPin = 24;
const int resetPin = 26;
const int irqPin = 25;

// Frquency for LoRa in Hz
const long freq = 4337E5; // 433.7 MHz

/* --------------- Sensors ---------------- */
// Sensor power pins
const int sensor_board_power = 22;

// Mics2714 variables
const int mics_input = 21;
const int mics_power = 20;
double mics_voltage = 0.0;
float no2_ppm;

// SGP30 object
Adafruit_SGP30 sgp;

// SGP30 Variables
float tvoc, eco2;
uint16_t TVOC_base = 0x0;
uint16_t eCO2_base = 0x0;

// SCD30 object
SCD30 scd;

// SCD30 variables
float co2;
float temp_scd30 = 25.0;
float humidity_scd30 = 35.0;

// Sensor object for BMP280
Adafruit_BMP280 bmp;

// Variables for BMP280
float temp_bmp = 0;
float pressure = 0;

/* ---------------- Miscellaneous variables ----------------- */
// Variables fot timing
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 5000; // 5 second

// How many steps are in analogRead, it differs from original 1024,
// because later on resolution gets changed from 8 to 12 bits
int analog_steps;

bool calibrated_scd = false;
bool calibrated_sgp = false;

/* ---------------------- Functions  -------------------------*/
// Function that turns on and begins communication with all senors
void turn_on_all_sensors()
{
  // Turn on sensor board
  digitalWrite(sensor_board_power, HIGH);
  Serial.println("Sensor board power is ON. Waiting 2 second for sensors to turn on!");
  delay(2000);

  // Begin communication with SCD30 sensor
  while (!scd.begin(Wire1))
  {
    Serial.println("SCD30 not found. This warning should only show up once!");
    delay(1000);
  }
  Serial.println("SCD30 started");
  LoRa.beginPacket();
  LoRa.println("SCD30 started");
  LoRa.endPacket();
  
  // Begin communication with SGP30 sensor
  while (!sgp.begin(&Wire2))
  {
    Serial.println("SGP not found");
    delay(100);
  }
  LoRa.beginPacket();
  LoRa.println("SGP started");
  LoRa.endPacket();
  Serial.println("SGP started");

  // Turn on Mics-2714
  digitalWrite(mics_power, LOW);
  Serial.println("Mics-2714 turned on");
  LoRa.beginPacket();
  LoRa.println("MICS started");
  LoRa.endPacket();
}

// Converts relative humidity to absolute humidity
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}

/* ---------------------- Setup -----------------------*/
void setup()
{
  // Set sensor power pins to output
  pinMode(sensor_board_power, OUTPUT);
  pinMode(mics_power, OUTPUT);

  // Set input pin from mics to input mode
  pinMode(mics_input, INPUT);
  // At the start set mics to off
  digitalWrite(mics_power, HIGH);

  // Begin serial and i2c comunication.
  Serial.begin(9600);
  Serial.println("Starting CanSat");
  Wire.begin();
  Wire1.begin();
  Wire2.begin();
  Serial.println("I2C started");

  // Change analog resolution to 12 bits. That is from 0 to 4095
  analogReadResolution(12);
  analog_steps = 4096;

  // Sets up required pins for LoRa radio module
  LoRa.setPins(csPin, resetPin, irqPin);

  // Tries to start up LoRa module. If it doesn't start, goes into infinite loop
  if (!LoRa.begin(freq))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  
  // Settings for LoRa
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  Serial.println("LoRa started");
  
  // Starts BMP280 sensor
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  // Settings for BMP280 sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
                  
  // Turn on all sensors for testing purposes
  tu rn_on_all_sensors();
}

/*------------------- Loop -------------------------*/
void loop()
{    
  // Get time since turned on in miliseconds
  currentMillis = millis();
  
  // Run only if interval of time has passed
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    // Get data from BMP280 sensor
    temp_bmp = bmp.readTemperature();
    pressure = bmp.readPressure();
    
    // If SCD30 has data ready, get the data
    if (scd.dataAvailable())
    {
      scd.setAmbientPressure(pressure/100);
      co2 = scd.getCO2();
      temp_scd30 = scd.getTemperature();
      humidity_scd30 = scd.getHumidity();
    }
    // Sets SGP30 humidity from SCD30 humidity measurment, to get the most accurate readings
    sgp.setHumidity(getAbsoluteHumidity(temp_bmp, humidity_scd30));
    // Gets readings from SGP30
    sgp.IAQmeasure();
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
    
    // Gets voltage from mics input
    mics_voltage = (analogRead(mics_input) * 3.3) / analog_steps;
    // Calculates no2 concentration in ppm, using graph from datasheet
    no2_ppm = ((3.3 - mics_voltage) / mics_voltage) / 6.667;

    if (calibrated_scd == false)
    {
      if ((millis()/1000) > 1800)
      {
        calibrated_scd = true;
        scd.setForcedRecalibrationFactor(410);
        LoRa.beginPacket();
        LoRa.print("Calibration is done");
        LoRa.endPacket();
      }
    }

    if (calibrated_sgp == false)
    {
      if ((millis()/1000) > 3600)
      {
        calibrated_sgp = true;
        sgp.getIAQBaseline(&eCO2_base, &TVOC_base);
        EEPROM.put(0, eCO2_base);
        EEPROM.put(100, TVOC_base);
        LoRa.beginPacket();
        LoRa.print("New SGP30 baseline  ");
        LoRa.print("eCO2 Base: ");
        LoRa.print(eCO2_base, HEX);
        LoRa.print("eCO2 Base: ");
        LoRa.print(TVOC_base, HEX);
        LoRa.endPacket();
        sgp.setIAQBaseline(eCO2_base, TVOC_base);
       }
    }
    //noInterrupts();
    // Sends data to LoRa radio module
    LoRa.beginPacket();
    LoRa.print((millis()/1000.0)/60.0);
    LoRa.print(",");
    LoRa.print(scd.getTemperatureOffset());
    LoRa.print(",");
    LoRa.print(temp_bmp);
    LoRa.print(",");
    LoRa.print(temp_scd30);
    LoRa.print(",");
    LoRa.print(humidity_scd30);
    LoRa.print(",");
    LoRa.print(pressure);
    LoRa.print(",");
    LoRa.print(eco2);
    LoRa.print(",");
    LoRa.print(co2);
    LoRa.print(",");
    LoRa.print(tvoc);
    LoRa.print(",");
    LoRa.print(no2_ppm);
    LoRa.endPacket();
   }
}
