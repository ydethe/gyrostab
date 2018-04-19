#include <ESP8266WiFi.h>


class Connection {
  private:
    WiFiClient m_espClient;

  public:
    void setupConnection();

    const WiFiClient* getWifiClient();
    
};

