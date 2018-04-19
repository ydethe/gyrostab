#include <Arduino.h>
#include <WebSocketsServer.h>

#define WS_PORT 36416

WebSocketsServer webSocket = WebSocketsServer(WS_PORT);

char ws_msg[1024];

void publisher() {
  sprintf(ws_msg, "%f,%f,%f,%f,%f,%f", ax, ay, az, 
      gx, gy, gz);
      
  webSocket.broadcastTXT(ws_msg);
  
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
       
        // send message to client
        webSocket.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);

            // send message to client
            // webSocket.sendTXT(num, "message here");

            // send data to all connected clients
            // webSocket.broadcastTXT("message here");
            break;
        case WStype_BIN:
            Serial.printf("[%u] get binary length: %u\n", num, length);
            hexdump(payload, length);

            // send message to client
            // webSocket.sendBIN(num, payload, length);
            break;
    }

}

void setup_websocket() {
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}


