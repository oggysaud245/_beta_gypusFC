#include <Wire.h>
#include <Adafruit_Sensor.h>
#include<HardwareSerial.h>
#include<SoftwareSerial.h>
//compass sensor
#include <MechaQMC5883.h>
MechaQMC5883 qmc; 

//esp-now declaration
#include <esp_now.h>
#include <WiFi.h>

String lower_data;

typedef struct struct_message {
    char rnd_1[200];
} struct_message;

struct_message receive_Data;




HardwareSerial xbee(2);
String xbeedata;

//all variable declaration for xbee write and data log
int alti, acc_x, acc_y, acc_z, gy_x, gy_y, gy_z,vib_data,temp, pres,heading, seperation_dep,sat_dep=0, para_dep=0,append=0;
float longi, lati;

#define ledpin 2
#define piezo_pin 4

//GPS declaration
#include<TinyGPSPlus.h>
//#include<WiFi.h>

float latitude, longitude;
String latitude_string, longitude_string;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);



//limit switch declaration
#define limitswitch1 33
#define limitswitch2 34
#define limitswitch3 35
//bool l1state,l2state,l3state;

//MPU declaration
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;
int offsetacc=3;


//Compass_declaration
#include<QMC5883LCompass.h>
QMC5883LCompass compass; 

//BMP_declaration
#include "Adafruit_BMP3XX.h"
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;


//SD_card_declaration
#include "FS.h"
#include <SD.h>
#include <SPI.h>
#define SD_CS 5
String dataMessage;


void setup() 
{
WiFi.mode(WIFI_STA); //--> Set device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // for limit switch
  pinMode(limitswitch1,INPUT);
  pinMode(limitswitch2,INPUT);
  pinMode(limitswitch3,INPUT);
  //LED_blink_setup
  pinMode(ledpin,OUTPUT);
  Serial.begin(9600); 
  xbee.begin(115200, SERIAL_8N1, 16, 17);
   
  Initialize_BMP390();
//   Initialize SD card
//  Wire.begin();
//  initialized_SD(); 
  mpu_initialize();
  qmc.init();
  mpu_initialize();
  
  //GPS setup
  SerialGPS.begin(9600, SERIAL_8N1, 14, 15);
}



void mpu_initialize()
{
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
}



void Initialize_BMP390()
{
   Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ); 
}

//SD card initialization
void initialized_SD()
{
   SD.begin(SD_CS);  
  if(!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }
  File file = SD.open("/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.txt", "ESP32 and SD Card \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
}


void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return; 
  }
  if(file.print(message)) {
    Serial.println("Message appended");
    append=1;
  } else {
    Serial.println("Append failed");
    append=0;
  }
  file.close();
}
//End SD card initialization



void OnDataRecv(const uint8_t * mac,const uint8_t* incomingData, int len) {
  memcpy(&receive_Data, incomingData, sizeof(receive_Data));
//  Serial.println();
//  Serial.println("<<<<< Receive Data:");
//  Serial.print("Bytes received: ");
//  Serial.println(len);
//  Serial.println("Receive Data: ");
//  Serial.println(receive_Data.rnd_1);
  lower_data=receive_Data.rnd_1;
//  Serial.println(lower_data);
//  Serial.println("<<<<<");
}



void compass_data()
{
  // Read compass values
  int x,y,z;
  qmc.read(&x,&y,&z); 
  heading=atan2(x, y)/0.0174532925;
 //Convert result into 0 to 360
  if(heading < 0)
  { 
  heading+=360;
  } 
 
    heading = 360-heading;
  }


void logSDCard() 
{     
  dataMessage=lower_data;
  Serial.print("Save data: ");
//  Serial.println(dataMessage);
  appendFile(SD, "/data.txt", dataMessage.c_str());
}

void vibration_sensor()
{
  vib_data= analogRead(piezo_pin);
  
}

void bmpdata()
{
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
//  temperature in celcius
  temp=bmp.temperature;
//pressure in hpa
  pres=bmp.pressure / 100.0;
// altitude in meter
  alti=bmp.readAltitude(SEALEVELPRESSURE_HPA);
}


void mpu_data()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  //acceleration in meter per second
  acc_x=a.acceleration.x;
  acc_y=a.acceleration.y;
  acc_z=(a.acceleration.z)-offsetacc ;

// gyro value in (rad/s)
  gy_x= g.gyro.x;
  gy_y= g.gyro.y;
  gy_z= g.gyro.z;
}

void led_blink()
{
  digitalWrite(ledpin, HIGH); 
  delay(100);    
  digitalWrite(ledpin,LOW); 
  delay(100); 
}

void limit_switch()
{
  bool l1state=digitalRead(limitswitch1);
  bool l2state=digitalRead(limitswitch2);
  bool l3state=digitalRead(limitswitch3);
  //seperation_dep=l1state;
  if(l1state == HIGH){seperation_dep=0;}
  else{ seperation_dep=1;}
 // Serial.println(l1state);

  //sat_dep=l2state;
  if(l2state == HIGH){sat_dep=0;}
  else{ sat_dep=1;}

  //para_dep=l3state;
  if(l3state == HIGH){para_dep=0;}
  else{ para_dep=1;}

}


void gps_data()
{
  while(SerialGPS.available()>0)
  {
    if(gps.encode(SerialGPS.read()))
    {
      if(gps.location.isValid())
      {
//         Serial.println(gps.location.lat(), 6);
//         Serial.println(F(","));
//         Serial.println(gps.location.lng(), 6);;
         lati= gps.location.lat();
         longi=gps.location.lng();
      }
      else
      {
        Serial.print(F("Invalid"));
      }
  }
  if (millis()>5000 && gps.charsProcessed()<10)
{
  Serial.println(F("No GPS detected: check wiring.")); 
  while(true);
}
     
}
}



void xbee_print()
{
  
 xbeedata=" ";
 xbeedata+="U,";
 xbeedata+=String(alti)+",";
 xbeedata+=String(acc_x)+",";
 xbeedata+=String(acc_y)+",";
 xbeedata+=String(acc_z)+",";
 xbeedata+=String(gy_x)+",";
 xbeedata+=String(gy_y)+",";
 xbeedata+=String(gy_z)+",";
 xbeedata+=String(lati,6)+",";
 xbeedata+=String(longi,6)+",";
 xbeedata+=String(vib_data)+",";
 xbeedata+=String(temp)+",";
 xbeedata+=String(pres)+",";
 xbeedata+=String(heading)+",";
 xbeedata+=String(seperation_dep)+",";
 xbeedata+=String(sat_dep)+",";
 xbeedata+=String(para_dep);

  xbee.println(xbeedata);
  xbee.println(lower_data); 
 // Serial.print(xbeedata);
  //Serial.println(lower_data);
  
  
}


void loop()
{
     
 gps_data();
 vibration_sensor();
 bmpdata();
 mpu_data();
 compass_data();
 limit_switch();
// logSDCard();
 xbee_print();
 delay(100);
 
}
