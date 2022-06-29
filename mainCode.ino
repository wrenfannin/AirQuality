/*Copyright (C) 2022 Wren Fannin and Arav Raja luwr5102@gmail.com 
Everyone is permitted to copy and distribute copies of this file under GNU-GPL3 */

#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "Seeed_HM330X.h"

Adafruit_SGP30 sgp;
HM330X sensor;
u8 buf[30];

const char *str[]={"sensor num: ","PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                    "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                    "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                    "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                    "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                    "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                    };

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}
err_t print_result(const char* str,u16 value)
{
    if(NULL==str)
        return ERROR_PARAM;
    Serial.print(str);
    Serial.println(value);
    return NO_ERROR;
}

/*parse buf with 29 u8-data*/
err_t parse_result(u8 *data)
{
    u16 value=0;
    err_t NO_ERROR;
    if(NULL==data)
        return ERROR_PARAM;
    for(int i=1;i<8;i++)
    {
         value = (u16)data[i*2]<<8|data[i*2+1];
         print_result(str[i-1],value);

    }
}

err_t parse_result_value(u8 *data)
{
    if(NULL==data)
        return ERROR_PARAM;
    //for(int i=0;i<28;i++)
    //{
        //Serial.print(data[i],HEX);
        //Serial.print("  ");
        //if((0==(i)%5)||(0==i))
        //{
          
            //Serial.println(" ");
        //}
    //}
    u8 sum=0;
    for(int i=0;i<28;i++)
    {
        sum+=data[i];
    }
    if(sum!=data[28])
    {
        Serial.println("wrong checkSum!!!!");
    }
    Serial.println(" ");
    Serial.println(" ");
    return NO_ERROR;
}
void setup(){
   Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for serial console to open!

  Serial.println("SGP30 test");

  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
  if(sensor.init())
    {
        Serial.println("HM330X init failed!!!");
        while(1);
    }
 
}
 int counter = 0;

 void loop(){
   if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\r\n");
  Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");

  if (! sgp.IAQmeasureRaw()) {
    Serial.println("Raw Measurement failed");
    return;
  }
  Serial.print("Raw H2 "); Serial.print(sgp.rawH2); Serial.print("\r\n");
  Serial.print("Raw Ethanol "); Serial.print(sgp.rawEthanol); Serial.println("");
 
  delay(1000);

  counter++;
  if (counter == 30) {
    counter = 0;

    uint16_t TVOC_base, eCO2_base;
    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
      Serial.println("Failed to get baseline readings");
      return;
    }
    //Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
    //Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
  }
  if(sensor.read_sensor_value(buf,29))
    {
        Serial.println("HM330X read result failed!!!");
    }
    parse_result_value(buf);
    parse_result(buf);
    //Serial.println(" ");
    //Serial.println(" ");
    //Serial.println(" ");
    delay(5000);
 }
