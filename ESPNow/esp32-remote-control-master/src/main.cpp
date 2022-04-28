#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include "string.h"

#define MAC_ADDRESS_FINDER  0

#if MAC_ADDRESS_FINDER == 1

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
}
 
void loop(){
  Serial.println(WiFi.macAddress());
  delay(2000);
}

#else

uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x30, 0x67, 0xAC};
uint8_t wifi_data_rxed = 0;
uint8_t wifi_data;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
 
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
    wifi_data = *incomingData;
    wifi_data_rxed = 1;
}


void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
   //Serial.println("Failed to add peer");
    return;
  }

  //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)"WiFi Com Test", strlen("WiFi Com Test"));
}

void loop() 
{
  if(Serial.available() > 0)
  {
    uint8_t rs_rx_byte_u8 = Serial.read();
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&rs_rx_byte_u8, 1);


    if(result != ESP_OK)
    {
      //Serial.println("WiFi Com Error!");
    }
  }

    if(wifi_data_rxed == 1)
    {
      Serial.write(wifi_data);
      wifi_data_rxed = 0;
    }
}

#endif