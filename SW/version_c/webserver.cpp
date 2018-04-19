#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

ESP8266WebServer server(80);

void setup_webserver() {
  if (MDNS.begin("pendule")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);
  
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");

}

void handleRoot() {
  char buf[5000];
  int ln_read;
  
  File f = SPIFFS.open("livemqttchart.html", "r");
  if (!f) {
    Serial.println("file open failed");
  }

  ln_read = f.readBytes(buf, 10);
  Serial.println(buf);
  f.seek(0, SeekSet);
  ln_read = f.readBytes(buf, 5000);
  f.close();
  server.send(200, "text/html", buf);
  
}

void handleNotFound(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}





