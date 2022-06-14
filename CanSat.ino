/* ----------- Required libaries --------------*/
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SD.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_PM25AQI.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <MicroNMEA.h>
#include <TeensyThreads.h>
#include <mpu9250.h>

/* ---------------- Lora ----------------- */
// LoRa pins
// Don't change these
const int csPin = 24;
const int resetPin = 26;
const int irqPin = 25;

// Frquency for LoRa in Hz
const long freq = 4339E5; // 433.9 MHz

/* ----------------- GPS ------------------*/
// Buffer to store data from gps
volatile char nmeaBuffer[100];
// GPS object
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// Variable to see if gps thread is running
int gps_thread;

// Variables to store gps data
float gps_latitude = 0.0;
float gps_longitude = 0.0;
long _gps_altitude = 0.0;
float gps_altitude = 0.0;
float gps_speed = 0.0;
int gps_hour, gps_minute, gps_second  = 0.0;

/* --------------- SD Card ---------------- */
// File object
File csvFile;

// Name of created csv file
String file_name;

// CSV file header
// EACH NAME MUST BE SEPERATED BY A COMMA(,) AND WITHOUT A SPACE BETWEEN
String header = "co2,eco2,tvoc,gps_latitude,gps_longitude,gps_altitude,baro_altitude,gps_speed,temp_bmp,temp_thermo,temp_scd30,temp_tmp,humidity_scd30,pressure,no2_ppm,pm10_std,pm25_std,pm100_std,pm_10_env,pm25_env,pm100_env,p03,p05,p10,p25,p50,p100,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,temp_imu";

/* --------------- Sensors ---------------- */
// Sensor board power enable/disable pin
const int sensor_board_power = 2;

// MPU9250
bfs::Mpu9250 imu;
float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
float temp_imu;

// TMP36
const int tmp_power = 30;
const int tmp_ground = 32;
const int tmp_signal = 31;
float temp_tmp;

// Mics2714 variables
const int mics_input = 23;
float no2_ppm, mics_voltage = 0.0;

// SGP30 object
Adafruit_SGP30 sgp;

// SGP30 Variables
int tvoc, eco2;

// PMS Objects
Adafruit_PM25AQI pms = Adafruit_PM25AQI();
PM25_AQI_Data pms_data;

// PMS variables
float pm10_std, pm25_std, pm100_std, pm10_env, pm25_env, pm100_env, p03, p05, p10, p25, p50, p100;

// SCD30 object
SCD30 scd;

// SCD30 variables
int co2;
float temp_scd30 = 25.0;
float humidity_scd30 = 35.0;

// Constants for NTC sensor
const int resistance_def = 33000; // Resistance for the sensor
const int constant_B = 3932;      // Sensors constant for temperature
const float t_25_kelvin = 298.15; // 25 degree temperature in kelvins
const float VCC = 3.3;            // Teensy voltage

// Variables for NTC sensor
float ntc_voltage, voltage_diff, ntc_resistance, ln_value, temp_thermo;

// Sensor object for BMP280
Adafruit_BMP280 bmp;

// Variables for BMP280
float pressure, baro_altitude, temp_bmp, highest_baro_altitude = 0;
unsigned long highest_baro_alt_time = 0;
const float ambient_pressure = 1010.3;

/* --------------------------Time ----------------- */
// Variables to store current time
int time_hour, time_minute, time_second = 0;

/* ---------------- Miscellaneous variables ----------------- */
// Variables for timing
unsigned long last_radio_transmit_millis = 0;
unsigned long last_sensor_reading_millis = 0;

const unsigned long sensor_reading_interval = 200; // 0.2 seconds
const unsigned long radio_interval = 1000; // 1 second

// How many steps are in analogRead, it differs from original 1024,
// because later on resolution gets changed from 8 to 12 bits
float analog_steps;

// Beeper power pin
const int beeper_power = 29;

/* ---------------------- Functions  -------------------------*/
// Converts relative humidity to absolute humidity
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
  // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
  const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
  const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
  return absoluteHumidityScaled;
}
// Funtion for TeensyThread to collect serial data from GPS continuosly
void read_gps_serial() {
  while (1) {
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      nmea.process(c);
    }
  }
}

// Function that updates gps data
void update_gps_data()
{
  // Save data to variables
  // First checks if data is not set to invalid
  float temp;
  temp = nmea.getLatitude() / 1000000.0;
  if (temp != 0.0 && temp != 999.0) {
    gps_latitude = temp;
  }
  temp = nmea.getLongitude() / 1000000.0;
  if (temp != 0.0 && temp != 999.0) {
    gps_longitude = temp;
  }
  nmea.getAltitude(_gps_altitude);
  temp = _gps_altitude / 1000.0;
  if (temp != 0.0) {
    gps_altitude = temp;
  }
  temp = (nmea.getSpeed() / 1000.0) * 1.852 / 3.6;
  if (temp >= 0) {
    gps_speed = temp;
  }
  temp = nmea.getHour();
  if (temp != 99) {
    gps_hour = temp;
  }
  temp = nmea.getMinute();
  if (temp != 99) {
    gps_minute = temp;
  }
  temp = nmea.getSecond();
  if (temp != 99) {
    gps_second = temp;
  }
}

// Start up LoRa and sets all settings
void begin_lora()
{
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
  // THESE SETINGS MUST MACTH BASE STATION SETINGS
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);

  // Except this one
  LoRa.setCodingRate4(8);

  // THIS LINE IS PROBABLY USELESS BECAUSE WE'RE NOT RECEIVING DATA
  LoRa.enableCrc();
}

// Function that turns on and begins communication with all senors
void turn_on_all_sensors()
{
  // Turn on sensor board
  digitalWrite(sensor_board_power, HIGH);
  delay(1000);

  // Starts BMP280 sensor
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

  // Settings for BMP280 sensor
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,   /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  // Start MPU9250 sensor
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  // Initialize and configure IMU
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while (1) {}
  }
  // Set the sample rate divider
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while (1) {}
  }

  // Begin communication with SCD30 sensor
  while (!scd.begin(Wire2))
  {
    delay(1000);
    Serial.println("SCD not working");
  }
  Serial.println("SCD working");
  // Begin communication with PMSA003I sensor
  while (!pms.begin_I2C(&Wire2))
  {
    delay(1000);
    Serial.println("PMS not working");
  }
  Serial.println("PMS working");

  // Begin communication with SGP30 sensor
  while (!sgp.begin(&Wire2))
  {
    delay(1000);
    Serial.println("SGP not working");
  }
  Serial.println("SGP working");
  /*
    int e_base = 0;
    int t_base = 0;
    EEPROM.get(0, e_base);
    EEPROM.get(100, t_base);
    sgp.setIAQBaseline(e_base, t_base);
  */

  // Begin communication with the GPS module
  Serial1.begin(9600);
  gps_thread = threads.addThread(read_gps_serial);
  Serial.println("GPS working");
}

// Turns off all sensors and gps
void turn_off_all_sensors()
{
  digitalWrite(sensor_board_power, LOW);
  threads.kill(gps_thread);
}

// Function that writes given message to SD card csv file
void write_to_sd(String message)
{
  // Opens a file
  File csvFile = SD.open(file_name.c_str(), FILE_WRITE);

  // if the file is available, write to it:
  if (csvFile) {
    csvFile.println(message);
    csvFile.close();
  }
}

// Create a new csv file and write the header with names of variables
void create_sd_file()
{
  // Get last index of created csv file
  int current_index = 0;
  EEPROM.get(200, current_index);

  // THIS CODE IS JUST FOR TESTING!!!
  // PLEASE DON'T FORGET TO DELETE THIS!!!
  if (current_index == 255) {
    EEPROM.put(200, 0);
  }

  // Increase index
  current_index += 1;

  // Creates the file name
  file_name = "CanSatLog" + (String)current_index;

  // If file with that index already exists, increment index until the index isn't taken
  while (SD.exists(file_name.c_str())) {
    current_index += 1;
    file_name = "CanSatLog" + (String)current_index;
  }

  // Write header to the new csv file
  write_to_sd(header);

  // Store the new index, that just got taken
  EEPROM.put(200, current_index);
}

// Begin SD card and create new file
void begin_sd()
{
  // See if the card is present and initialize it
  while (!SD.begin(BUILTIN_SDCARD)) {
    delay(1000);
  }
  // Create a new csv file where to save data
  create_sd_file();
}

// Read data from SCD30
void read_scd30()
{
  if (scd.dataAvailable()) {
    // scd.setAmbientPressure(pressure/1000);
    co2 = scd.getCO2();
    temp_scd30 = scd.getTemperature() - 3.5;
    humidity_scd30 = scd.getHumidity();
  }
}

// Read data from PMSA003I
void read_pms() {
  pms.read(&pms_data);
  pm10_std = pms_data.pm10_standard;
  pm25_std = pms_data.pm25_standard;
  pm100_std = pms_data.pm100_standard;
  pm10_env = pms_data.pm10_env;
  pm25_env = pms_data.pm25_env;
  pm100_env = pms_data.pm100_env;
  p03 = pms_data.particles_03um;
  p05 = pms_data.particles_05um;
  p10 = pms_data.particles_10um;
  p25 = pms_data.particles_25um;
  p50 = pms_data.particles_50um;
  p100 = pms_data.particles_100um;
}

// Read data from SGP30
void read_sgp() {
  // Sets SGP30 humidity from SCD30 humidity measurment, to get the most accurate readings
  sgp.setHumidity(getAbsoluteHumidity(temp_thermo, humidity_scd30));
  // Gets readings from SGP30
  if (sgp.IAQmeasure()) {
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
  }
}

// Read data from GY-91
void read_gy91()
{
  // BMP280 data
  temp_bmp = bmp.readTemperature();
  pressure = bmp.readPressure();
  baro_altitude = bmp.readAltitude(ambient_pressure);
  // If higher barometer altitude has been recorded, save it and get time when it happened
  if (baro_altitude > highest_baro_altitude)
  {
    highest_baro_altitude = baro_altitude;
    highest_baro_alt_time = millis();
  }

  // MPU9250 data
  if (imu.Read()) {
    acc_x = imu.accel_x_mps2();
    acc_y = imu.accel_y_mps2();
    acc_z = imu.accel_z_mps2();
    gyro_x = imu.gyro_x_radps();
    gyro_y = imu.gyro_y_radps();
    gyro_z = imu.gyro_z_radps();
    mag_x = imu.mag_x_ut();
    mag_y = imu.mag_y_ut();
    mag_z = imu.mag_z_ut();
    temp_imu = imu.die_temp_c();
  }
}

// Read temperature from NTC thermoresistor
void read_ntc()
{
  ntc_voltage = (VCC / analog_steps) * analogRead(A10); // Analog reading from NTC
  voltage_diff = VCC - ntc_voltage;
  ntc_resistance = ntc_voltage / (voltage_diff / 4700); // Calculates the resistance of the sensor
  ln_value = log(ntc_resistance / resistance_def);                     // Calculates natural log value
  temp_thermo = (1 / ((ln_value / constant_B) + (1 / t_25_kelvin)));   // Temperature from sensor in kelvin
  temp_thermo = temp_thermo - 273.15 - 2.5; // Converts to celsius with correction offset
}

void read_mics()
{
  // Gets voltage from mics input
  mics_voltage = (analogRead(mics_input) * 3.3) / analog_steps;
  // Calculates no2 concentration in ppm, using graph from datasheet
  no2_ppm = (((3.3 - mics_voltage) / mics_voltage) / 6.667) - 1.0;
  if (no2_ppm < 0.0) {
    no2_ppm = 0.0;
  }
}

// Writes all data to csv file
void write_new_csv_line()
{
  // Save data to csv file
  String csv_row = "";
  int int_variables[] = {co2, eco2, tvoc};
  float location_variables[] = {gps_latitude, gps_longitude};
  float float_variables[] = {gps_altitude, baro_altitude, gps_speed, temp_bmp, temp_thermo, temp_scd30, temp_tmp, humidity_scd30, pressure, no2_ppm, pm10_std, pm25_std, pm100_std, pm10_env, pm25_env, pm100_env, p03, p05, p10, p25, p50, p100, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temp_imu};
  // Append all int type variables to string
  for (int i = 0; i < (sizeof(int_variables) / sizeof(int_variables[0])); i++) {
    csv_row.concat(int_variables[i]);
    csv_row.concat(",");
  }
  // Serial.print("After ints: "); Serial.println(csv_row);

  // Append all location variables to string
  // These 2 variables are added seperate, because they need more decimal places
  for (int i = 0; i < (sizeof(location_variables) / sizeof(location_variables[0])); i++) {
    csv_row.concat(String(location_variables[i], 6));
    csv_row.concat(",");
  }
  // Serial.print("After location: "); Serial.println(csv_row);
  
  // Append the rest of the float type variables to string
  for (int i = 0; i < (sizeof(float_variables) / sizeof(float_variables[0])); i++) {
    csv_row.concat(String(float_variables[i], 2));
    if ((sizeof(float_variables) / sizeof(float_variables[0])) - 1 != i) {
      csv_row.concat(",");
    }
  }
  // Serial.print("After floats: "); Serial.println(csv_row);
  
  // Write a new line to csv file
  write_to_sd(csv_row);
}

// THIS FUNCTION HAS TO BE CHANGED TO BE MORE RELIABLE
// Function checks if Cansat has landed and puts cansat in power saving and recovery mode
void check_if_landed()
{
  // Check if altitude has been higher than 400m, that means that launch was successful
  // then check that Cansat is falling down, and then we can be sure that after 10 minutes
  // Cansat must have landed and stayed still
  if (highest_baro_altitude > 400 && baro_altitude < 300 && (millis() - highest_baro_alt_time) > 600000)
  {
    while (1) {

    }
  }
}

/* ---------------------- Setup -----------------------*/
void setup()
{
  // Set all pins to their required mode
  pinMode(sensor_board_power, OUTPUT);
  pinMode(beeper_power, OUTPUT);
  pinMode(mics_input, INPUT);
  pinMode(tmp_ground, OUTPUT);
  digitalWrite(tmp_ground, LOW);
  pinMode(tmp_power, OUTPUT);
  digitalWrite(tmp_power, HIGH);

  // Begin serial and i2c comunication.
  Serial.begin(115200);
  Wire.begin();
  Wire2.begin();

  // Change analog resolution to 12 bits. That is from 0 to 4095
  analogReadResolution(12);
  analog_steps = 4096.0;

  // Start LoRa
  begin_lora();

  // Set up SD for writing data to a file
  begin_sd();

  // Turn on the rest of the electronics
  turn_on_all_sensors();

  Serial.println("Setup done");
}

/*------------------- Loop -------------------------*/
void loop()
{
  // Checks if Cansat has landed
  check_if_landed();

  // If it is time to read and sensor board is on, get latest data available
  if (((millis() - last_sensor_reading_millis) > sensor_reading_interval) && (digitalRead(sensor_board_power) == HIGH)) {
    // GPS data
    update_gps_data();

    // NTC thermoresistor data
    read_ntc();
    
    // SCD 30 data
    read_scd30();

    // PMSA003I data
    read_pms();

    // SGP30 data
    read_sgp();

    // GY-91 data (BMP280 and MPU9250)
    read_gy91();

    // Mics-2714 data
    read_mics();

    // Read temperature from TMP36
    temp_tmp = (((3.3 / analog_steps) * analogRead(tmp_signal)) - 0.5) * 100;

    // Write all data to the csv file
    write_new_csv_line();

    // Set time when last readings happend
    last_sensor_reading_millis = millis();
  }

  // Run only if interval of time has passed
  if (millis() - last_radio_transmit_millis >= radio_interval)
  {
    // Sends data to LoRa radio module
    /*
      LoRa.beginPacket();
      LoRa.print(gps_hour);
      LoRa.print(",");
      LoRa.print(gps_minute);
      LoRa.print(",");
      LoRa.print(gps_second);
      LoRa.print(",");
      LoRa.print(gps_latitude, 6);
      LoRa.print(",");
      LoRa.print(gps_longitude, 6);
      LoRa.print(",");
      LoRa.print(gps_altitude);
      LoRa.print(",");
      LoRa.print(gps_speed);
      LoRa.print(",");
      LoRa.print(temp_scd30);
      LoRa.print(",");
      LoRa.print(pressure);
      LoRa.print(",");
      LoRa.print(humidity_scd30);
      LoRa.endPacket();
    */
    // Display all data in Serial monitor
    Serial.println("--- GPS data ---");
    Serial.print("Longitude = ");
    Serial.print(gps_latitude, 6);
    Serial.print(" | Latitude = ");
    Serial.print(gps_longitude, 6);
    Serial.print(" | Altitude = ");
    Serial.print(gps_altitude);
    Serial.print(" meters | Speed = ");
    Serial.print(gps_speed);
    Serial.print(" km/h | Time = ");
    Serial.print(gps_hour);
    Serial.print(":");
    Serial.print(gps_minute);
    Serial.print(":");
    Serial.println(gps_second);
    Serial.println("--- SCD30 data ---");
    Serial.print("CO2 = ");
    Serial.print(co2);
    Serial.print(" ppm | Temperature = ");
    Serial.print(temp_scd30);
    Serial.print(" °C | Humidity = ");
    Serial.print(humidity_scd30);
    Serial.println(" %");
    Serial.println("--- SGP30 data ---");
    Serial.print("eC02 = ");
    Serial.print(eco2);
    Serial.print(" ppm | TVOC = ");
    Serial.print(tvoc);
    Serial.println(" ppb");
    Serial.println("--- PMSA003I data ---");
    Serial.print("PM1.0 STD = ");
    Serial.print(pm10_std);
    Serial.print(" | PM2.5 STD = ");
    Serial.print(pm25_std);
    Serial.print(" | PM10.0 STD = ");
    Serial.print(pm100_std);
    Serial.print(" | PM1.0 ENV = ");
    Serial.print(pm10_env);
    Serial.print(" | PM2.5 ENV = ");
    Serial.print(pm25_env);
    Serial.print(" | PM10.0 ENV = ");
    Serial.print(pm100_env);
    Serial.print(" | P0.3 = ");
    Serial.print(p03);
    Serial.print(" | P0.5 = ");
    Serial.print(p05);
    Serial.print(" | P1.0 = ");
    Serial.print(p10);
    Serial.print(" | P2.5 = ");
    Serial.print(p25);
    Serial.print(" | P5.0 = ");
    Serial.print(p50);
    Serial.print(" | P10.0 = ");
    Serial.println(p100);
    Serial.println("--- GY-91 data ---");
    Serial.print("Temperature  = ");
    Serial.print(temp_bmp);
    Serial.print(" °C | Pressure = ");
    Serial.print(pressure);
    Serial.print(" Pa | Altitude = ");
    Serial.print(baro_altitude);
    Serial.print(" m | AccX  = ");
    Serial.print(acc_x);
    Serial.print(" m/s^2 | AccY = ");
    Serial.print(acc_y);
    Serial.print(" m/s^2 | AccZ = ");
    Serial.print(acc_z);
    Serial.print(" m/s^2 | GyroX  = ");
    Serial.print(gyro_x);
    Serial.print(" rad/s | GyroY = ");
    Serial.print(gyro_y);
    Serial.print(" rad/s | GyroZ = ");
    Serial.print(gyro_z);
    Serial.print(" rad/s | MagX  = ");
    Serial.print(mag_x);
    Serial.print(" µT | MagY= ");
    Serial.print(mag_y);
    Serial.print(" µT | MagZ = ");
    Serial.print(mag_z);
    Serial.print(" µT | TempMPU = ");
    Serial.print(temp_imu);
    Serial.println("°C");
    Serial.println("--- Mics-2714 data ---");
    Serial.print("NO2 = ");
    Serial.print(no2_ppm);
    Serial.println(" ppm");
    Serial.println("--- Temperature sensor data ---");
    Serial.print("Thermmoresistor temperature = ");
    Serial.print(temp_thermo);
    Serial.print(" °C | TMP36 temperature = ");
    Serial.print(temp_tmp);
    Serial.println(" °C");
    Serial.println("--------------------------------------------");

    last_radio_transmit_millis = millis();
  }
}
