/* Copyright (c) Dyson Technology Ltd 2019. All rights reserved. 
 *  Software for the James Dyson Foundation Air Quality Monitoring Kit
 *  A.T. Luisi, 11/2019
 *  Diagnostic Software and Manufacturing Test
 */

// Arduino Level Includes
#include <Wire.h>

// AVR includes for direct hardware control
// specifically for the Watchdog Timer
#include <avr/wdt.h>

// EEPROM include for persistently storing
// SGP30 baseline value
#include <EEPROM.h>

// Include the part libraries
#include "Seeed_HM330X.h"
#include "Adafruit_SGP30.h"
#include "Adafruit_NeoPixel.h"

// Defines
#define DEBUG

#define LED_STRIP_COUNT 10u
#define LED_STRIP_1_PIN 2u // could go in 5
#define LED_STRIP_2_PIN 3u // could go in 6

#define H330X_DATA_BUFFER_CHECK_SIZE 28u
#define H330X_DATA_BUFFER_SIZE 30u

#define TIME_ELAPSED_HOURS_1_2  4320L

// Use to confirm in EEPROM if the SGP30 ID even exists
#define SGP_DEVICE_ID_WRITTEN_CONFIRM 0xDEu

// Address to verify if the SGP30 ID has been written to EEPROM before
#define SGP_DEVICE_REGISTERED_ADDR            0x00u
// The address to store the baselines
#define SGP_CO2_BASELINE_EEPROM_ADDR_START    0x01u
#define SGP_CO2_BASELINE_EEPROM_ADDR_END      0x02u

#define SGP_TVOC_BASELINE_EEPROM_ADDR_START   0x03u
#define SGP_TVOC_BASELINE_EEPROM_ADDR_END     0x04u

// Addresses of the bytes for the 48-bit device address
#define SGP_DEVICE_EEPROM_START_ADDR          0x05u
#define SGP_DEVICE_EEPROM_END_ADDR            0x0Au

// Global Variable declarations
HM330X pmSensor;
Adafruit_SGP30 vocHumiditySensor;
Adafruit_NeoPixel ledStrip_1(LED_STRIP_COUNT, LED_STRIP_1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledStrip_2(LED_STRIP_COUNT, LED_STRIP_2_PIN, NEO_GRB + NEO_KHZ800);

typedef struct PMSensorReadings 
{
   u16 pm1_0_cf1;
   u16 pm2_5_cf1;
   u16 pm10_cf1;
   u16 pm1_0_ae;
   u16 pm2_5_ae;
   u16 p10_a3;
   
} PMSensorReadings;

// Set the basic colors for the "AQI Thermometer"
uint32_t ledStrip_blue = ledStrip_1.Color(0u,0u,255u);

uint16_t hueValue = 21846u; // Arbitrarily chosen to start at green

bool isH330Connected = false;
bool isSGP30Connected = false;
bool hasSGP30IDBeenWritten = false;

// Timekeeping for SGP30 startup
uint32_t startTime;
// Elapsed time kept in the loop scope to discard
// when out of scope

// Function prototypes:
void colorWipe(byte red, byte green, byte blue, int SpeedDelay);
err_t inspectChecksum(u8 *data);
err_t parse_result(u8 *data, PMSensorReadings *pmReads);
uint16_t mapPM2_5(uint16_t value);
uint16_t mapPM10(uint16_t value);
uint16_t mapTVOC(uint16_t value);
uint16_t getTopHueColor(uint8_t mapIndex);

void determineSetupStateBeforeLoop();

void initH330();
void initSGP30();

void earlyOperationSGP30(uint16_t &co2Base, uint16_t &tvocBase);
void storeBaselines(uint16_t &co2Base, uint16_t &tvocBase);
void getBaselines(uint16_t &co2Base, uint16_t &tvocBase);

void checkSGP30IDWritten();
bool getSGP30DeviceID(uint16_t *devID); // Bool to verify IDs match
void storeSGP30DeviceID(uint16_t *devID);

void clearEEPROMRegion();

void waitForWatchDogKick();

void determineOperationSGP30();

void blinkErrorCode(uint8_t blinkNo, String err);

// Taken and adapted for use from: https://www.tweaking4all.com/hardware/arduino/adruino-led-strip-effects/ 04/12/2019
void colorWipe(byte red, byte green, byte blue, int SpeedDelay) {
  for(uint16_t i=0; i<LED_STRIP_COUNT; i++) {
      ledStrip_1.setPixelColor(i, red, green, blue);
      ledStrip_2.setPixelColor(i, red, green, blue);
      ledStrip_1.show();
      ledStrip_2.show();
      delay(SpeedDelay);
  }
}

void initH330()
{
  // call this periodically to either enter or exit the SGP30
  // conditioning routine
   // Set up the PM sensor:
  #ifdef DEBUG
  Serial.println("Initializing PM Sensor...");
  #endif 
  if(pmSensor.init())
  {
    #ifdef DEBUG
    Serial.println("PM Sensor Init Fail");
    #endif
    // Enter the conditioning stage
    // after the main loop
    isH330Connected = false;
  }
  else
  {
    isH330Connected = true;
    #ifdef DEBUG
    Serial.println("PM Sensor Init Success");
    #endif 
  }
}

void initSGP30()
{
  // Set up the tVOC sensor
  #ifdef DEBUG
  Serial.println("Initializing tVOC Sensor...");
  #endif
  if (!vocHumiditySensor.begin())
  {
    #ifdef DEBUG
    Serial.println("tVOC Sensor Init Fail");
    #endif
    // Set a flag to go to the conditioning routine 
    // after the setup has been completed. Use the NACK
    // from the PM sensor _not_ being connected and a flag
    // set here to determine correct entry.
    isSGP30Connected = false;
  }
  else
  {
    isSGP30Connected = true;
    Serial.println("tVOC Sensor Init Success");
  }
}


void setup() {
  // put your setup code here, to run once:
  // Output to the serial terminal
  Serial.begin(115200);

  // Init the watchdog timer for a 4s timeout
  //wdt_enable(WDTO_4S);
  //wdt_reset();
  
  #ifdef DEBUG
  Serial.println("Initializing LED Strip...");
  #endif 
  // Set up the LED strips
  ledStrip_1.begin();
  // First turn off the strip
  ledStrip_1.show();
  // Set the brightness to something that isn't blinding, 50% seems reasonable
  ledStrip_1.setBrightness(128);
  // Turn on the entire strip to blue
  ledStrip_1.fill(ledStrip_blue);
  ledStrip_1.show();
  // enough time to inspect ~2secs
  delay(2000);
  // Turn off the strip for later
  ledStrip_1.clear();
  ledStrip_1.show();

  ledStrip_2.begin();
  // First turn off the strip
  ledStrip_2.show();
  // Set the brightness to something that isn't blinding, 50% seems reasonable
  ledStrip_2.setBrightness(128);
  // Turn on the entire strip to blue
  ledStrip_2.fill(ledStrip_blue);
  ledStrip_2.show();
  // enough time to inspect ~2secs
  delay(2000);
  // Turn off the strip for later
  ledStrip_2.clear();
  ledStrip_2.show();
  
  #ifdef DEBUG
  Serial.println("LED Strip Init Success");
  #endif 

  // LEDs initialized animation
  colorWipe(0u,0u,255u,50u);
  colorWipe(0u,0u,0u,50u);

  // Init the sensors to determine
  // their connection status 
  initH330();
  initSGP30();

  // Determine the state to setup depending
  // on the status of the EEPROM check and
  // the sensor connection status
  checkSGP30IDWritten();
  determineSetupStateBeforeLoop();
  
}

// Clears the EEPROM region in use and sets to 0
void clearEEPROMRegion()
{
  // Erase EEPROM and reset
  for(uint8_t i = SGP_DEVICE_REGISTERED_ADDR; i < SGP_DEVICE_EEPROM_END_ADDR; i++)
  {
    EEPROM.write(i, 0);
  }
}

// Choose the pre-Loop setup routine:
void determineSetupStateBeforeLoop()
{
  
  uint16_t sgp30Serial[3];
  uint16_t co2Baseline;
  uint16_t tvocBaseline;

  #ifdef DEBUG
  Serial.println("Determining setup state");
  #endif
  
  if((isH330Connected) && (isSGP30Connected) && (hasSGP30IDBeenWritten))
  {
    
    if(getSGP30DeviceID(sgp30Serial))
    { 
      // Start normal operation
      #ifdef DEBUG
      Serial.println("Normal operation mode.");
      #endif
      // Load the baselines:
      
      getBaselines(co2Baseline, tvocBaseline);

      // Set the IAQ baselines
      vocHumiditySensor.setIAQBaseline(co2Baseline, tvocBaseline);
      
      // Force return to caller to prevent
      // filter through to the rest of the code
      return;
      
    }
    else
    {
      // Serial numbers don't match,
      // so enter the Early Op phase
      // to get the baselines:
      #ifdef DEBUG
      Serial.println("Non-matching serial numbers. EOM.");
      #endif

      // Run early operation:
      earlyOperationSGP30(co2Baseline, tvocBaseline);
  
      // Store the baselines:
      storeBaselines(co2Baseline, tvocBaseline);

      // Update the serial number
      storeSGP30DeviceID(vocHumiditySensor.serialnumber);
      
      // Reset:
      waitForWatchDogKick();      
    }
  }

  if((!isH330Connected) && (!isSGP30Connected) && (!hasSGP30IDBeenWritten))
  {
    // Reset
    #ifdef DEBUG
    Serial.println("No sensors connected, no value stored. Reset.");
    #endif 

    waitForWatchDogKick();
  }
  
  if((!isH330Connected) && (!isSGP30Connected) && (hasSGP30IDBeenWritten))
  {
    // Reset
    #ifdef DEBUG
    Serial.println("No sensors connected, value stored. Reset.");
    #endif 

    waitForWatchDogKick();
  }

  if((isH330Connected) && (!isSGP30Connected) && (!hasSGP30IDBeenWritten))
  {
    // Reset
    #ifdef DEBUG
    Serial.println("PM sensor connected, no TVOC sensor, no value stored. Reset.");
    #endif 

    waitForWatchDogKick();
  }

  if((isH330Connected) && (!isSGP30Connected) && (hasSGP30IDBeenWritten))
  {
    // Clear EEPROM region and reset
    #ifdef DEBUG
    Serial.println("PM sensor connected, no TVOC sensor, value stored. Clear and reset.");
    #endif 

    clearEEPROMRegion();
    waitForWatchDogKick();
  }

  if((!isH330Connected) && (isSGP30Connected) && (!hasSGP30IDBeenWritten))
  {
    // Early operation, store baselines, reset
    #ifdef DEBUG
    Serial.println("No PM sensor, TVOC sensor connected, no value stored/previously written. EOM then reset.");
    #endif 

    // Pass in the baseline containers
    // Want to keep the overall control of the data
    // in this scope to aid understanding of the code

    // Conduct the early operation phase:
    earlyOperationSGP30(co2Baseline, tvocBaseline);

    // Store the baselines:
    storeBaselines(co2Baseline, tvocBaseline);

    // Reset:
    waitForWatchDogKick();
  }

  if((!isH330Connected) && (isSGP30Connected) && (hasSGP30IDBeenWritten))
  {
   #ifdef DEBUG 
   Serial.println("No PM sensor, TVOC sensor connected, value stored/previously written.");
   #endif

   // Extract the VOC sensor ID and check
   // if the stored value matches with the
   // connected device
   if(getSGP30DeviceID(sgp30Serial))
   {
    // Flash the LEDs to indicate the PM sensor
    // should be connected

    // Reset:
    #ifdef DEBUG
    Serial.println("Serial no.s match. Reset.");
    #endif 
    waitForWatchDogKick();
   }
   else
   {
    // Serial numbers don't match,
    // so enter the Early Op phase
    // to get the baselines:
    #ifdef DEBUG
    Serial.println("Serial no.s do not match. EOM then Reset.");
    #endif

    // Conduct the early operation phase:
    earlyOperationSGP30(co2Baseline, tvocBaseline);

    // Store the baselines:
    storeBaselines(co2Baseline, tvocBaseline);

    // Update the serial number
    storeSGP30DeviceID(vocHumiditySensor.serialnumber);
    
    // Reset:
    waitForWatchDogKick();
   }
  }

  if((isH330Connected) && (isSGP30Connected) && (!hasSGP30IDBeenWritten))
  {
    // Early operation, store baselines, reset
    #ifdef DEBUG
    Serial.println("All sensors connected but no value. EOM then reset.");
    #endif 

    // Conduct the early operation phase:
    earlyOperationSGP30(co2Baseline, tvocBaseline);

    // Store the baselines:
    storeBaselines(co2Baseline, tvocBaseline);

    // Store serial no.
    storeSGP30DeviceID(vocHumiditySensor.serialnumber);

    // Reset:
    waitForWatchDogKick();
  }
   
}

// Extract the data from the EEPROM
bool getSGP30DeviceID(uint16_t *devID)
{
  // Check EEPROM for baseline and serial number of
  // the SGP30
  uint16_t sgp30DevSerialNo[3];
  uint8_t extracted;
  uint64_t tempDevSerial = 0u;
  uint16_t shiftValue = 0u;
  uint8_t index = 5u; // to store big endian

  uint16_t toInsert;

  // Extract the contents from the EEPROM

  // Stored BIG ENDIAN- the MSBs are at the start and the LSBs are at the end
  // The sensor by default outputs big endian
  for(uint16_t i = SGP_DEVICE_EEPROM_START_ADDR; i < SGP_DEVICE_EEPROM_END_ADDR + 1; i++)
  {
    extracted = EEPROM.read(i);
    shiftValue = index * 8u;
    // Shifts in MSB first for big endian
    tempDevSerial |= (((uint64_t)(extracted)) << shiftValue);
    
    index--;
  }

  // Extract bits, then cast to uint16
  // MSB component gets shifted 8 left after cast
  sgp30DevSerialNo[2]  = ((uint16_t)((tempDevSerial) & 0x00FFu));
  sgp30DevSerialNo[2] |= (((uint16_t)((tempDevSerial >> 8u) & 0x00FFu)) << 8u);

  sgp30DevSerialNo[1]  = ((uint16_t)((tempDevSerial >> 16u) & 0x00FFu));
  sgp30DevSerialNo[1] |= (((uint16_t)((tempDevSerial >> 24u) & 0x00FFu)) << 8u);

  sgp30DevSerialNo[0]  = ((uint16_t)((tempDevSerial >> 32u) & 0x00FFu));
  sgp30DevSerialNo[0] |= (((uint16_t)((tempDevSerial >> 40u) & 0x00FFu)) << 8u);

  // Pass back the pointer, even if the return is found to be false
  devID = sgp30DevSerialNo;

  // Check the LSBs of the serial number with the extracted EEPROM data
   
  if(sgp30DevSerialNo[2] != vocHumiditySensor.serialnumber[2])
  {
    // error
    #ifdef DEBUG
    Serial.println("[2]: ");
    Serial.println(sgp30DevSerialNo[2]);
    #endif 

    return false;
  }

  if(sgp30DevSerialNo[1] != vocHumiditySensor.serialnumber[1])
  {
    // error
    #ifdef DEBUG
    Serial.println("[1]: ");
    Serial.println(sgp30DevSerialNo[1]);
    #endif 
    return false;
  }

  if(sgp30DevSerialNo[0] != vocHumiditySensor.serialnumber[0])
  {
    // error
    #ifdef DEBUG
    Serial.println("[0]: ");
    Serial.println(sgp30DevSerialNo[0]);
    #endif 
    return false;
  }

  // Drop out here- the SGP30 device IDs match
  return true;
  
}

void storeSGP30DeviceID(uint16_t *devID)
{
  uint8_t toStore;
  uint8_t eepromIndex = SGP_DEVICE_EEPROM_START_ADDR;

  #ifdef DEBUG
  Serial.println("Storing VOC ID");
  #endif 

  // Extract the individual bytes from
  // the ID array to store in EEPROM
  for(uint8_t i = 0; i < 3; i++)
  {
    // Upper byte
    toStore = ((uint8_t)((devID[i] >> 8u) & 0x00FFu));
    #ifdef DEBUG
    Serial.println(toStore);
    #endif 
    EEPROM.write(eepromIndex, toStore);
    eepromIndex++;
    // Lower byte
    toStore = ((uint8_t)(devID[i] & 0x00FFu));
    #ifdef DEBUG
    Serial.println(toStore);
    #endif
    EEPROM.write(eepromIndex, toStore);
    eepromIndex++;
  }

  // Write to the flag location
  EEPROM.write(SGP_DEVICE_REGISTERED_ADDR, SGP_DEVICE_ID_WRITTEN_CONFIRM);  
  
}

void checkSGP30IDWritten()
{
  byte extracted = EEPROM.read(SGP_DEVICE_REGISTERED_ADDR);

  if(extracted != SGP_DEVICE_ID_WRITTEN_CONFIRM)
  {
    hasSGP30IDBeenWritten = false;
  }
  else
  {
    hasSGP30IDBeenWritten = true;
  }
  
}

// Error state routine
void waitForWatchDogKick()
{
  #ifdef DEBUG
  Serial.println("Resetting.");
  #endif 
  wdt_enable(WDTO_4S);
  
  while(1);
}

// Perform the necessary steps to extract
// the baselines after the required time period
void earlyOperationSGP30(uint16_t &co2Base, uint16_t &tvocBase)
{
  // Run for 12 hours 
  uint32_t startTime = millis();
  uint8_t hourCount = 0u;
  bool elapsed1Hour = false;

  // Clear the LED strip
  ledStrip_1.clear();
  ledStrip_1.show();

  // Perform the IAQ init
  vocHumiditySensor.IAQinit();

  #ifdef DEBUG
  Serial.println("Early Operation Mode.");
  Serial.println("Running SGP30 Baseline Initialization");
  #endif 

  while(hourCount < 10u)
  {
    while(!elapsed1Hour)
    {
      
      // Check how much time has elapsed
      uint32_t timeElapsed = (millis() - startTime);
      if((timeElapsed >= (TIME_ELAPSED_HOURS_1_2 * 1000)))
      {
        // exit the inner loop
        elapsed1Hour = true;
      }
    }
    elapsed1Hour = false;
    startTime = millis();
    // Update the LED strip for every 1.2 hours elapsed
    ledStrip_1.setPixelColor(hourCount, ledStrip_blue);
    ledStrip_1.show();
    hourCount++;
  }

  // 12 hours elapsed, now can get baseline values
  uint16_t *co2;
  uint16_t *tvoc;
  vocHumiditySensor.getIAQBaseline(co2, tvoc);

  // Dereference and then pass back
  co2Base = *co2;
  tvocBase = *tvoc;

  #ifdef DEBUG
  Serial.print("CO2 baseline:");
  Serial.println(co2Base);
  Serial.print("TVOC baseline: ");
  Serial.println(tvocBase);
  #endif 
}

// Store the baseline values to the EEPROM
void storeBaselines(uint16_t &co2Base, uint16_t &tvocBase)
{
  uint8_t toStore = 0u;

  // Store the MSBs and then LSBs (big endian) 
  
  // CO2
  toStore = (uint8_t)((co2Base >> 8u) & 0x00FFu);
  EEPROM.write(SGP_CO2_BASELINE_EEPROM_ADDR_START, toStore);
  toStore = (uint8_t)(co2Base & 0x00FFu);
  EEPROM.write(SGP_CO2_BASELINE_EEPROM_ADDR_END, toStore);

  // TVOC
  toStore = (uint8_t)((tvocBase >> 8u) & 0x00FFu);
  EEPROM.write(SGP_TVOC_BASELINE_EEPROM_ADDR_START, toStore);
  toStore = (uint8_t)(tvocBase & 0x00FFu);
  EEPROM.write(SGP_TVOC_BASELINE_EEPROM_ADDR_END, toStore);
 
}

// Get the IAQ baselines from the EEPROM
void getBaselines(uint16_t &co2Base, uint16_t &tvocBase)
{
  uint8_t extractedMsbs = 0u;
  uint8_t extractedLsbs = 0u;
  uint16_t shift = 0u;
  
  extractedMsbs = EEPROM.read(SGP_CO2_BASELINE_EEPROM_ADDR_START);
  shift = (((uint16_t)(extractedMsbs)) << 8u);
  extractedLsbs = EEPROM.read(SGP_CO2_BASELINE_EEPROM_ADDR_END);
  shift |= extractedLsbs;

  co2Base = shift;

  extractedMsbs = 0u;
  extractedLsbs = 0u;
  shift = 0u;

  extractedMsbs = EEPROM.read(SGP_TVOC_BASELINE_EEPROM_ADDR_START);
  shift = (((uint16_t)(extractedMsbs)) << 8u);
  extractedLsbs = EEPROM.read(SGP_TVOC_BASELINE_EEPROM_ADDR_END);
  shift |= extractedLsbs;

  tvocBase = shift;
  
}

// Blink the LEDs if there are errors 
// associated with the sensors
void blinkErrorCode(uint8_t blinkNo, String err)
{
  Serial.println(err);
  // Blink the LED Strip 4 times for a PM Sensor problem
  for(int i = 0; i < blinkNo; i++)
  {
    ledStrip_1.fill(ledStrip_blue);
    ledStrip_1.show();
    delay(1000);
    ledStrip_1.clear();
    ledStrip_1.show();
  }
}

// Inspect the PM result checksum
err_t inspectChecksum(u8 *data)
{
  u8 sum = 0;

  // Perform the checksum
  for(int i = 0; i < H330X_DATA_BUFFER_CHECK_SIZE; i++)
  {
    sum += data[i];  
  }
  if(sum != data[H330X_DATA_BUFFER_CHECK_SIZE])
  {
    // Checksum is incorrect so data is unreliable
    // Throw an error
    return ERROR_PARAM;
  }
  return NO_ERROR;
}

// Parse the PM results 
err_t parse_result(u8 *data, PMSensorReadings *pmReads)
{
  u16 unpackingArray[8];

  // Check if the buffer contains any data
  if(!data)
  {
    return ERROR_PARAM;
  }

  // Extract the bytes
  for(int i = 1; i < 8; i++)
  {
    unpackingArray[i-1] = (u16)((data[i*2] << 8) | (data[(i*2)+1]));
  }

  // Possibly a more elegant way to do this
  pmReads->pm1_0_cf1 = unpackingArray[0];
  pmReads->pm2_5_cf1 = unpackingArray[1];
  pmReads->pm10_cf1 = unpackingArray[2];
  
  pmReads->pm1_0_ae = unpackingArray[3];
  pmReads->pm2_5_ae = unpackingArray[4];
  pmReads->pm2_5_ae = unpackingArray[5];

  return NO_ERROR;
}

// PM2.5
uint16_t mapPM2_5(uint16_t value)
{

  uint16_t defraLow = 0u;
  uint16_t defraHigh = 70u;
  uint16_t defraIndexLow = 0u;
  uint16_t defraIndexHigh = 9u;
  
  if(value >= 71u)
  {
    return 10u;
  }

  return map(value, defraLow, defraHigh, defraIndexLow, defraIndexHigh);
  // constrain?
}

// PM10
uint16_t mapPM10(uint16_t value)
{
  
  uint16_t defraLow = 0u;
  uint16_t defraHigh = 100u;
  uint16_t defraIndexLow = 0u;
  uint16_t defraIndexHigh = 9u;
  
  if(value >= 101u)
  {
    return 10u;
  }

  return map(value, defraLow, defraHigh, defraIndexLow, defraIndexHigh);
  // constrain?
}

// TVOC
uint16_t mapTVOC(uint16_t value)
{
  
  uint16_t vocLow = 0u;
  uint16_t vocHigh = 5001u;
  uint16_t vocLowIndex = 0u;
  uint16_t vocHighIndex = 9u;

  if(value >= 6000u)
  {
    return 10u;
  }

  return map(value, vocLow, vocHigh, vocLowIndex, vocHighIndex);
  // constrain?
}

// Get the HSV value to fill the LEDs with
uint16_t getTopHueColor(uint8_t mapIndex)
{
  
  uint16_t aqiMax = 9u;
  
  return (uint16_t)(((hueValue / aqiMax)*((uint16_t)mapIndex - aqiMax)) + hueValue);
  
}


// Determine the dominant PM value
uint8_t mapPMAQIValues(PMSensorReadings *pm)
{
  // Get the individual readings
  uint16_t pm2_5 = (uint16_t)pm->pm2_5_cf1; // in ug/m^3
  uint16_t pm10 = (uint16_t)pm->pm10_cf1;
  
  uint16_t pm2_5Normalized = mapPM2_5(pm2_5);
  uint16_t pm10Normalized = mapPM10(pm10);
  

  // Filter out the dominant pollutant value
  uint16_t largest = (pm2_5Normalized > pm10Normalized) ? (pm2_5Normalized) : (pm10Normalized);

  return (uint8_t)largest;
  
}

uint8_t mapTVOCAQIValues(uint16_t tvocIn)
{
   uint16_t tvocNormalized = mapTVOC(tvocIn);

   return (uint8_t)(tvocNormalized);
   
}

void loop() {
  // PM Sensor Variables
  u8 pmDataBuffer[H330X_DATA_BUFFER_SIZE];
  PMSensorReadings pmReadings;

  // SGP30 Variables
  uint16_t tvocValue;
  uint16_t eCO2Value; // Needed?

  uint8_t aqi_pm; // raw pm value
  uint8_t aqi_tvoc; // raw co2 value ??
  
  uint32_t hueTopColor;
  uint16_t colorInterp;

  uint8_t sat = 70u; // Aribitrarily chosen
  uint8_t val = 70u; // As above

  // Check if the PM sensor is connected

  // Read from the PM sensor
  if(pmSensor.read_sensor_value(pmDataBuffer, (H330X_DATA_BUFFER_SIZE - 1)))
  {
    blinkErrorCode(4u, "PM sensor read failed");
  }

  // Check the output value
  if(parse_result(pmDataBuffer, &pmReadings))
  {
    blinkErrorCode(4u, "PM sensor checksum failed");
  }
  
  // Get the SPS30 reading
  if(!vocHumiditySensor.IAQmeasure())
  {
    blinkErrorCode(8u, "TVOC IAQ read failed");
  }

  tvocValue = vocHumiditySensor.TVOC;
  eCO2Value = vocHumiditySensor.eCO2;

  // Map the PM AQI values to the first LED strip
  aqi_pm = mapPMAQIValues(&pmReadings);

  // Map the TVOC AQI values to the second LED strip
  aqi_tvoc = mapTVOCAQIValues(tvocValue);


  // For the PM reading first:
  // Get the 'top' RGB color for the last LED
  hueTopColor = getTopHueColor(aqi_pm);
  #ifdef DEBUG
  Serial.print("Hue Top Color (PM): ");
  Serial.println(hueTopColor);
  #endif 

  // Calculate the steps between the 'top' color and the base hue
  colorInterp = (hueValue / (LED_STRIP_COUNT - 1));
  #ifdef DEBUG
  Serial.print("Color Interp: ");
  Serial.println(colorInterp);
  #endif

  // Yellow could do with some work...?

  // Index from 1 to put at least one LED on and prevent user concern
  // Note that this has the effect of artificially increasing the AQI value by 1

  for(int i = 0; i < (aqi_pm+1); i++)
  {
    // Get the current color values
    uint16_t currentLEDHue = hueValue - (colorInterp * i);
    uint32_t rgbColor = ledStrip_1.gamma32(ledStrip_1.ColorHSV(currentLEDHue));
    #ifdef DEBUG
    Serial.print("Current LED hue: ");
    Serial.println(currentLEDHue);
    
    Serial.print("RGB Color: ");
    Serial.println(rgbColor);
    #endif 

    // Write to the LEDs sequentially:
    ledStrip_1.setPixelColor(i, rgbColor);
    ledStrip_1.show();

  }

  if((aqi_pm+1) < (ledStrip_1.numPixels() - 1))
  {
    // Blank the LEDs not required
    ledStrip_1.fill(0, (aqi_pm+1), (ledStrip_1.numPixels() - (aqi_pm+1)));
  }

  // Repeat for the TVOC reading:
  // Get the 'top' RGB color for the last LED
  hueTopColor = getTopHueColor(aqi_tvoc);
  #ifdef DEBUG
  //Serial.print("Hue Top Color (TVOC): ");
  //Serial.println(hueTopColor);
  #endif 

  // Calculate the steps between the 'top' color and the base hue
  colorInterp = (hueValue / (LED_STRIP_COUNT - 1));
  #ifdef DEBUG
  //Serial.print("Color Interp: ");
  //Serial.println(colorInterp);
  #endif

  // Yellow could do with some work...?
  for(int i = 0; i < (aqi_tvoc+1); i++)
  {
    // Get the current color values
    uint16_t currentLEDHue = hueValue - (colorInterp * i);
    uint32_t rgbColor = ledStrip_2.gamma32(ledStrip_2.ColorHSV(currentLEDHue));
    #ifdef DEBUG
    //Serial.print("Current LED hue: ");
   //Serial.println(currentLEDHue);
    
    //Serial.print("RGB Color: ");
    //Serial.println(rgbColor);
    #endif 

    // Write to the LEDs sequentially:
    ledStrip_2.setPixelColor(i, rgbColor);
    
    ledStrip_2.show();
    //delay(10);
  }
  if((aqi_tvoc+1) < (ledStrip_2.numPixels() - 1))
  {
    // Blank the LEDs not required
    ledStrip_2.fill(0, (aqi_tvoc), (ledStrip_2.numPixels() - (aqi_tvoc)));
  }  
  Serial.println("PM 2.5:");
  Serial.println(pm2_5);
  delay(500)
  Serial.println("PM 10:");
  Serial.println(pm10);
    

}
