#include "connection.h"


const WiFiClient * Connection::getWifiClient() {
  return &m_espClient;
}

void Connection::setupConnection(const char* wifi_ssid, const char* wifi_pass) {
  // Connect to WiFi
  Serial.print(s+"Connecting to WiFi: "+wifi_ssid+" ");
  WiFi.begin(wifi_ssid, wifi_pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(s+" connected with IP: "+WiFi.localIP());
  
}



