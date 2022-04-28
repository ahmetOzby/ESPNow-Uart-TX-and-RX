#include <esp_now.h>
#include <WiFi.h>

#include <Arduino.h>

#define MAC_ADDRESS_FINDER  0

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
 
}

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



uint8_t rs_rxed = 0;
uint8_t rs_received_byte;
uint8_t broadcastAddress[] = {0x30, 0x83, 0x98, 0x44, 0x11, 0xC8};
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  //Serial.print("data received");
    rs_received_byte = *incomingData;
    rs_rxed = 1;
}

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

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
  //Serial.print("ok");
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
   //Serial.println("Failed to add peer");
    return;
  }

  //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)"WiFi Com Test", strlen("WiFi Com Test"));
}

void loop() {
  if(rs_rxed == 1)
  {
    Serial.write(rs_received_byte);
    rs_rxed = 0;
  }

    if(Serial.available() > 0)
    {
      uint8_t rs_rx_byte_u8 = Serial.read();
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&rs_rx_byte_u8, 1);
      //Serial.print(rs_rx_byte_u8);

      if(result != ESP_OK)
      {
        //Serial.println("WiFi Com Error!");
      }
    }
}

#endif