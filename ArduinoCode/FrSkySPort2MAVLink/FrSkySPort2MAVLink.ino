/*
  FrSky S-Port Telemetry Decoder library example for the XJT decoder class (old hub telemetry and basic RSSI/ADC1/ADC2/RxBatt/SWR data)
  (c) Pawelsky 20160919
  Not for commercial use
  
  Note that you need Teensy 3.x/LC or 328P based (e.g. Pro Mini, Nano, Uno) board and FrSkySportDecoder library for this example to work
*/

// Uncomment the #define below to enable internal polling of data.
// Use only when there is no device in the S.Port chain (e.g. S.Port capable FrSky receiver) that normally polls the data.
//#define POLLING_ENABLED

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
#ifdef POLLING_ENABLED
  FrSkySportDecoder decoder(true);  // Create decoder object with polling
#else
  FrSkySportDecoder decoder;        // Create decoder object without polling
#endif


uint32_t currentTime, displayTime;
uint16_t decodeResult;

void setup(){
  decoder.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_3, &xjt, &fcs, &gps, &rpm, &vario);
  Serial.begin(57600);
}

void loop(){
  // Read and decode the telemetry data, note that the data will only be decoded for sensors
  // that have been passed to the begin method. Print the AppID of the decoded data.
  decodeResult = decoder.decode();
  if(decodeResult != SENSOR_NO_DATA_ID){
    Serial.print("   :)   Decoded data with AppID 0x"); Serial.println(decodeResult, HEX);
  }
  
  // Display data once a second to not interfeere with data decoding
  currentTime = millis();
  if(currentTime > displayTime){
    displayTime = currentTime + UPDATE_DELAY;

    Serial.println("");

    // Get basic XJT data (RSSI/ADC1/ADC2/RxBatt/SWR data)
    Serial.print("BASIC: RSSI = "); Serial.print(xjt.getRssi()); // RSSI
    Serial.print(", ADC1 = "); Serial.print(xjt.getAdc1());      // ADC1 voltage in volts
    Serial.print("V, ADC2 = "); Serial.print(xjt.getAdc2());     // ADC2 voltage in volts
    Serial.print("V, RxBatt = "); Serial.print(xjt.getRxBatt()); // RxBatt voltage in volts
    Serial.print("V, SWR = "); Serial.println(xjt.getSwr());     // SWR

    // Get current/voltage sensor (FAS) data
    Serial.print("FCS: current = "); Serial.print(fcs.getCurrent());                    // Current consumption in amps
    Serial.print("A, voltage = "); Serial.print(fcs.getVoltage()); Serial.println("V"); // Battery voltage in volts

    // Get variometer sensor (FVAS) data
    Serial.print("VARIO: altitude = "); Serial.print(vario.getAltitude()); // Altitude in m (can be nevative)
    Serial.print("m, VSI = "); Serial.print(vario.getVsi()); Serial.println("m/s");          // Verticas speed in m/s (can be nevative)

    // Get GPS data
    Serial.print("GPS: lat = "); Serial.print(gps.getLat(), 6); Serial.print(", lon = "); Serial.print(gps.getLon(), 6); // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
    Serial.print(", altitude = "); Serial.print(gps.getAltitude()); // Altitude in m (can be negative)
    Serial.print("m, speed = "); Serial.print(gps.getSpeed()); // Speed in m/s
    Serial.print("m/s, COG = "); Serial.print(gps.getCog());   // Course over ground in degrees (0-359, 0 = north)
    char dateTimeStr[18]; 
    sprintf(dateTimeStr, "%02u-%02u-%04u %02u:%02u:%02u", gps.getDay(), gps.getMonth(), gps.getYear() + 2000, gps.getHour(), gps.getMinute(), gps.getSecond());
    Serial.print(", date/time = "); Serial.println(dateTimeStr); // Date (year - need to add 2000, month, day) and time (hour, minute, second)

    // Get accelerometer sensor (TAS) data
    Serial.print("TAS: ACCX = "); Serial.print(xjt.getAccX());                    // X axis acceleraton in Gs (can be negative)
    Serial.print("G, ACCY = "); Serial.print(xjt.getAccY());                      // Y axis acceleraton in Gs (can be negative)
    Serial.print("G, ACCZ = "); Serial.print(xjt.getAccZ()); Serial.println("G"); // Z axis acceleraton in Gs (can be negative)

    // Get RPM sensor (RPMS) data
    Serial.print("RPM: RPM = "); Serial.print(rpm.getRpm()*2);
    Serial.print(", T1 = "); Serial.print(rpm.getT1());
    Serial.print(" deg. C, T2 = "); Serial.print(rpm.getT1()); Serial.println(" deg. C");


    Serial.println("");
  }
}
