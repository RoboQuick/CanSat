/* ----------- Required libaries --------------*/
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_BMP280.h>
#include <SparkFun_SCD30_Arduino_Library.h>

/* --------------- Sensors ---------------- */
// Sensor power pins
const int sensor_board_power = 2;

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
  while (!scd.begin(Wire2))
  {
    Serial.println("SCD30 not found. This warning should only show up once!");
    delay(1000);
  }
  Serial.println("SCD30 started");
  
  // Begin communication with SGP30 sensor
  while (!sgp.begin(&Wire2))
  {
    Serial.println("SGP not found");
    delay(100);
  }
  Serial.println("SGP started");
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

  // Begin serial and i2c comunication.
  Serial.begin(9600);
  Serial.println("Starting CanSat");
  Wire.begin();
  Wire2.begin();
  Serial.println("I2C started");

  // Starts BMP280 sensor
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  // Settings for BMP280 sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
                  
  // Turn on all sensors for testing purposes
  turn_on_all_sensors();
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
      co2 = scd.getCO2();
      temp_scd30 = scd.getTemperature() - 3.5;
      humidity_scd30 = scd.getHumidity();
    }
    // Sets SGP30 humidity from SCD30 humidity measurment, to get the most accurate readings
    sgp.setHumidity(getAbsoluteHumidity(temp_scd30, humidity_scd30));
    // Gets readings from SGP30
    sgp.IAQmeasure();
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
    
    if (calibrated_scd == false)
    {
      if ((millis()/1000) > 1800)
      {
        calibrated_scd = true;
        scd.setForcedRecalibrationFactor(420);
        Serial.println("SCD calibrated");
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
        sgp.setIAQBaseline(eCO2_base, TVOC_base);
        Serial.println("SGP calibrated");
        Serial.print("eCO2 base: ");
        Serial.println(eCO2_base);
        Serial.print("TVOC base: ");
        Serial.println(TVOC_base);
       }
    }
    Serial.print("Seconds passed: ");
    Serial.println(millis() / 1000);
    }
}
