/*
 * 
 *                        FRSKY SPORT 2 MAVLINK CONVERTER
 *                       
 * Arduino based FrSky SmartPort to MAVLink converter to display INAV telemetry 
 * data in Ground Station App via Bluetooth.
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

#define BLUETOOTH         Serial
#define UPDATE_DELAY      200 //ms
#define TIME_OUT          2000 //ms


FrSkySportSensorXjt xjtSensor;
FrSkySportSensorFcs fcsSensor;
FrSkySportSensorGps gpsSensor;
FrSkySportSensorRpm rpmSensor;
FrSkySportSensorVario varioSensor;
FrSkySportDecoder smartPortDecoder;



uint32_t currentTime,bluetoothTime,lastReceived;
uint16_t decodeResult;

void setup(){
  smartPortDecoder.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_3,&xjtSensor,&fcsSensor,&gpsSensor,&rpmSensor,&varioSensor);
  BLUETOOTH.begin(57600);
}

void loop(){
  decodeResult = smartPortDecoder.decode();
  if(decodeResult != SENSOR_NO_DATA_ID) lastReceived = millis();
  
  currentTime = millis();
  if((currentTime > bluetoothTime) && ((currentTime - lastReceived) < TIME_OUT)){
    bluetoothTime = currentTime + UPDATE_DELAY;

    BLUETOOTH.print(xjtSensor.getRssi());
    BLUETOOTH.print(",");
    BLUETOOTH.print(xjtSensor.getAdc1());
    BLUETOOTH.print(",");
    BLUETOOTH.print(xjtSensor.getAdc2());
    BLUETOOTH.print(",");
    BLUETOOTH.print(xjtSensor.getRxBatt(),2);
    BLUETOOTH.print(",");
    BLUETOOTH.print(xjtSensor.getSwr());
    BLUETOOTH.print(",");
    BLUETOOTH.print(fcsSensor.getCurrent());
    BLUETOOTH.print(",");
    BLUETOOTH.print(fcsSensor.getVoltage(),2);
    BLUETOOTH.print(",");
    BLUETOOTH.print(varioSensor.getAltitude(),1);
    BLUETOOTH.print(",");
    BLUETOOTH.print(varioSensor.getVsi(),1);
    BLUETOOTH.print(",");
    BLUETOOTH.print(gpsSensor.getLat(),6);
    BLUETOOTH.print(",");
    BLUETOOTH.print(gpsSensor.getLon(),6);
    BLUETOOTH.print(",");
    BLUETOOTH.print(gpsSensor.getAltitude(),1);
    BLUETOOTH.print(",");
    BLUETOOTH.print(gpsSensor.getSpeed(),1);
    BLUETOOTH.print(",");
    BLUETOOTH.print(gpsSensor.getCog());
    BLUETOOTH.print(",");
    /*
    gpsSensor.getDay()
    gpsSensor.getMonth()
    gpsSensor.getYear()
    gpsSensor.getHour()
    gpsSensor.getMinute()
    gpsSensor.getSecond()
    */
    BLUETOOTH.print(xjtSensor.getAccX());
    BLUETOOTH.print(",");
    BLUETOOTH.print(xjtSensor.getAccY());
    BLUETOOTH.print(",");
    BLUETOOTH.print(xjtSensor.getAccZ());
    BLUETOOTH.print(",");
    BLUETOOTH.print(rpmSensor.getRpm());
    BLUETOOTH.print(",");
    BLUETOOTH.print(rpmSensor.getT1());
    BLUETOOTH.print(",");
    BLUETOOTH.print(rpmSensor.getT1());

    BLUETOOTH.println("");

  }


  
}
