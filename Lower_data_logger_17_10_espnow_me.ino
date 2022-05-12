#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SPI.h>
#define SEALEVELPRESSURE_HPA (1013.25)
//compass sensor
#include <MechaQMC5883.h> 
MechaQMC5883 qmc;

Adafruit_BMP3XX bmp;
uint16_t ground_alt = 0;
uint32_t current_alt = 0;
uint32_t current_time = 0;

uint16_t lower_max_alt = 0;
uint16_t max_alt = 0;
uint16_t separationDistance = 200;

byte limit = 33;
byte mosfet = 35;
bool isDeployed = false;
bool toFire = false; 

//all variable declaration for xbee write and data log
int alti, acc_x, acc_y, acc_z, gy_x, gy_y, gy_z,vib_data,temp, pres,heading, sat_dep, para_dep,append;
float longi=0.000, lati=0.0000;

#define ledpin 2
#define piezo_pin 4

// Esp_now_declaration
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

String lower_data;
String success;

typedef struct struct_message {
    char rnd_1[200];
} struct_message;

struct_message send_Data;


//
////limit switch declaration
const int paradeploy=33;
//int paradeploy_state = 0;
bool depstate1;

 
//MPU declaration
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;
int offsetacc= 3;


//Compass_declaration
#include<QMC5883LCompass.h>
QMC5883LCompass compass; 

//BMP_declaration
//#include "Adafruit_BMP3XX.h"
//#define SEALEVELPRESSURE_HPA (1013.25)
//Adafruit_BMP3XX bmp;


//SD_card_declaration
#include "FS.h"
#include <SD.h>
#include <SPI.h>
#define SD_CS 5
String dataMessage;

void setup() 
{
Serial.begin(9600);
  pinMode(limit, INPUT);
  pinMode(mosfet, OUTPUT);
  digitalWrite(mosfet, LOW);
  digitalWrite(limit, HIGH);
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

pinMode(paradeploy,INPUT);
WiFi.mode(WIFI_STA); //--> Set device as a Wi-Fi Station.

  //----------------------------------------Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  
  //LED_blink_setup
  pinMode(ledpin,OUTPUT);
 
   
  Initialize_BMP390();
   // Initialize SD card
  Wire.begin();
  initialized_SD(); 
  mpu_initialize();
  qmc.init();

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  delay(1000);
 
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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
  Serial.println(">>>>>");
}


void compass_data()
{
  // Read compass values
int x,y,z;
qmc.read(&x,&y,&z); 
  
  heading=atan2(x, y)/0.0174532925;
 //Convert result into 0 to 360
  if(heading < 0) 
  heading+=360;
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
  //bool paradeploy_state;
 // depstate1 = digitalRead(paradeploy); 
   bool paradeploy_state = digitalRead(paradeploy); 
  //paradeploy_state = depstate1;
//int paradeploy_state= paradeploy_state.getState();
  if(paradeploy_state == HIGH){para_dep=0;}
  else{ para_dep=1;}
  
}


 

String xbee_print()
{
  
 lower_data=" ";
 lower_data+="L,";
 lower_data+=String(alti)+",";
 lower_data+=String(acc_x)+",";
 lower_data+=String(acc_y)+",";
 lower_data+=String(acc_z)+",";
 lower_data+=String(gy_x)+",";
 lower_data+=String(gy_y)+",";
 lower_data+=String(gy_z)+",";
 lower_data+=String(lati,6)+",";
 lower_data+=String(longi,6)+",";
 lower_data+=String(vib_data)+",";
 lower_data+=String(temp)+",";
 lower_data+=String(pres)+",";
 lower_data+=String(heading)+",";
 lower_data+=String(sat_dep)+",";
 lower_data+=String(para_dep)+",";
 lower_data+=String(append) ;

 return lower_data;
 Serial.println(lower_data);
  
}


void loop()
{ 
   while (millis() > 1000 && millis() < 3000) {
    Serial.println("Waiting...");
  }
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  static bool takeAverage = true;
  if (takeAverage) {
    for (int i = 0; i < 20; i++) {
      ground_alt += bmp.readAltitude(SEALEVELPRESSURE_HPA);
      //Serial.println(ground_alt);
      delay(100);
      if (i == 19) {
        takeAverage = false;
        ground_alt -= 1583;
        ground_alt /= i;
        continue;
      }
    }
  }
  static long int pressure = bmp.pressure / 100.0; // hpa
  current_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA); //meters
   if (limit == HIGH) {
    isDeployed = true;
  }
  if (isDeployed == true) {
    _process();
  }
  
limit_switch();
 xbee_print();
 vibration_sensor();
 bmpdata();
 mpu_data();
 compass_data();
 logSDCard();

 strcpy(send_Data.rnd_1,lower_data.c_str());
 Serial.println(lower_data);
  Serial.print(">>>>> ");
  Serial.println("Send data");
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &send_Data, sizeof(send_Data));
  
   if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
 digitalWrite(ledpin,HIGH);
 delay(100);
 digitalWrite(ledpin, LOW);
 
}
void _process() {
  if (current_alt > max_alt) {
    max_alt = current_alt;
  }
  if (max_alt - current_alt >= 10) {
    toFire = true;
  }
  if (toFire == true) {
    if (current_alt - ground_alt >= separationDistance) {
      digitalWrite(mosfet, HIGH);
      current_time = millis();
      toFire = false;
    }
  }
  if (millis() - current_time >= 3000) {
    digitalWrite(mosfet, LOW);
  }
}
