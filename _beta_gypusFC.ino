#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Adafruit_BMP3XX.h"
#define gpsSerial Serial1

#define GPSTAG1 "$GNRMC"
#define GPSTAG2 "$GNGGA"
#define GPSTAG3 "$GNGLL"

#define RX 2
#define TX 4

#define RXD2 16
#define TXD2 17

Adafruit_MPU6050 mpu;

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

String stringStart = "[";
String stringEnd = "]";
String stringComma = ",";
String _lat;
String _lon;
String _data = "";

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  gpsSerial.begin(115200, SERIAL_8N1, RX, TX);
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C(0x76)) {   // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  delay(10);

}

void loop() {
  static String _lat = "00.00";
  static String _lon = "00.00";
  static String identifier;
  static String recivedGps;
  /* Take a new reading */
  mpu.read();

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  recivedGps = readGPS();           // read NMEA Sentences
  identifier = recivedGps.substring(0, 6);
  Serial.println(recivedGps);

  if (identifier == GPSTAG1){ // print only $GNRMC, $GNGGA and $GNGLL nmea sentences
   // Serial.println(recivedGps);
    if(recivedGps.length()>=60){
      _lat = recivedGps.substring(19,29);
      _lon = recivedGps.substring(32,43);
    }
  }
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  String _value = stringStart + _lat  + stringComma + _lon + stringComma + String(a.acceleration.x) + stringComma + String(a.acceleration.y) + stringComma + String(a.acceleration.z) + stringComma + String(g.gyro.x) + stringComma + String(g.gyro.y) + stringComma + String(g.gyro.z) + stringComma + String(bmp.temperature) + stringComma + String(bmp.pressure / 100.0) + stringComma + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + stringEnd;
  Serial2.println(_value);
  delay(500);
}

String readGPS() {        // read raw data from serial
  static String _rawData;
  while (gpsSerial.available()) {
    //clearGPS();         //clear serial buffer and ignore garbage data
    _rawData = gpsSerial.readStringUntil('\r');     // read new data until a carriage return occurs "almost all nmea sentences had CR at end"
  }
  return correctNMEA(_rawData);                  //corrent data sometime multiple nmea identifier may occur in same line for example $GNGGA$GNRMC,XX,XX,, ignore first and extract latest
}
void clearGPS() {  //function to clear serial buffer
  while (gpsSerial.available()) {
    gpsSerial.readStringUntil('\n');
  }
  while (gpsSerial.available()) {
    gpsSerial.readStringUntil('\n');
  }
//  while (gpsSerial.available()) {
//    char c = (char)gpsSerial.read();
//    if (c == '\r') {
//      break;
//    }
//  }
}
String correctNMEA(String nmea) { // function to correct nmea sentences
  int len = nmea.length();
  static String validData;
  for ( int i = 0; i < len; i++) {
    if (nmea[i] == '$')
      validData = "";
    validData += nmea[i];     // new data will start form $, if multiple $ occurs only append from latest $
  }
  return validData;
}

