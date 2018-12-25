/*
 * 
 *                       FRSKY SPORT 2 MAVLINK CONVERTER
 * 
 * 
 * 
 * Aaron G.
 * Dec 2018
 */

#include "FrSkySportSensor.h"
#include "FrSkySportSensorXjt.h"
#include "FrSkySportSensorFcs.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorRpm.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportDecoder.h"
#include "SoftwareSerial.h"

#define UPDATE_DELAY      1000 //ms


FrSkySportSensorXjt xjt;
FrSkySportSensorFcs fcs;
FrSkySportSensorGps gps;
FrSkySportSensorRpm rpm;
FrSkySportSensorVario vario;
FrSkySportDecoder decoder;



uint32_t currentTime, displayTime;
uint16_t decodeResult;

void setup(){
  decoder.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_3,&xjt,&fcs,&gps,&rpm,&vario);
  Serial.begin(57600);
}

void loop(){
  decodeResult = decoder.decode();
  if(decodeResult != SENSOR_NO_DATA_ID){
    Serial.print("   :)   Decoded data with AppID 0x"); Serial.println(decodeResult, HEX);
  }
  
  currentTime = millis();
  if(currentTime > displayTime){
    displayTime = currentTime + UPDATE_DELAY;

    Serial.println("");

    Serial.print("BASIC: RSSI = "); Serial.print(xjt.getRssi());
    Serial.print(", ADC1 = "); Serial.print(xjt.getAdc1());
    Serial.print("V, ADC2 = "); Serial.print(xjt.getAdc2());
    Serial.print("V, RxBatt = "); Serial.print(xjt.getRxBatt());
    Serial.print("V, SWR = "); Serial.println(xjt.getSwr());

    Serial.print("FCS: current = "); Serial.print(fcs.getCurrent());
    Serial.print("A, voltage = "); Serial.print(fcs.getVoltage()); Serial.println("V");

    Serial.print("VARIO: altitude = "); Serial.print(vario.getAltitude());
    Serial.print("m, VSI = "); Serial.print(vario.getVsi()); Serial.println("m/s");

    Serial.print("GPS: lat = "); Serial.print(gps.getLat(), 6); Serial.print(", lon = "); Serial.print(gps.getLon(), 6);
    Serial.print(", altitude = "); Serial.print(gps.getAltitude());
    Serial.print("m, speed = "); Serial.print(gps.getSpeed());
    Serial.print("m/s, COG = "); Serial.print(gps.getCog());
    char dateTimeStr[18]; 
    sprintf(dateTimeStr, "%02u-%02u-%04u %02u:%02u:%02u", gps.getDay(), gps.getMonth(), gps.getYear() + 2000, gps.getHour(), gps.getMinute(), gps.getSecond());
    Serial.print(", date/time = "); Serial.println(dateTimeStr);

    Serial.print("TAS: ACCX = "); Serial.print(xjt.getAccX());
    Serial.print("G, ACCY = "); Serial.print(xjt.getAccY());
    Serial.print("G, ACCZ = "); Serial.print(xjt.getAccZ()); Serial.println("G");

    Serial.print("RPM: RPM = "); Serial.print(rpm.getRpm()*2);
    Serial.print(", T1 = "); Serial.print(rpm.getT1());
    Serial.print(" deg. C, T2 = "); Serial.print(rpm.getT1()); Serial.println(" deg. C");


    Serial.println("");
  }
}
