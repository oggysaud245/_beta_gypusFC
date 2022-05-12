#include "WiFi.h"
#define debug Serial
//TaskHandle_t smartConfig_handler = NULL;

void setup() {
  Serial.begin(115200);
  xTaskCreate(smartConfig, "smartConfig_fun", 2000, NULL, 1, NULL);
  //vTaskStartScheduler();

}

void loop() {
  // put your main code here, to run repeatedly:
  // if(WiFi.status() == WL_CONNECTED){
  //   debug.println("WiFi Connected!");
  // }
  // else debug.println("WiFi not Connected!");

}
void smartConfig(void *parameter){
  WiFi.mode(WIFI_AP_STA);
  WiFi.beginSmartConfig();
  static uint32_t time = millis();
  debug.println("Waiting for SmartConfig.");
  for(;;){
      while(!WiFi.smartConfigDone()){
        debug.print(".");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
      if(WiFi.status() == WL_CONNECTED){
        debug.println("WiFi Connected.");
        debug.print("IP Address: ");
        debug.println(WiFi.localIP());
        debug.println("Task is being Deleted");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //vTaskDelete(smartConfig_handler);
      }
  }
}