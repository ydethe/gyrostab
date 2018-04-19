#include <math.h>
#include <FS.h>
#include <Thread.h> // https://github.com/ivanseidel/ArduinoThread
#include <ThreadController.h>

#define WIFI_SSID "GreenMill"
#define WIFI_PASS "camillecoeurcoeur"

ThreadController threadControl = ThreadController();
Thread thread = Thread();

String s = "";

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(115200);

  setup_connection();
  
  setup_websocket();

  // Enable Thread
  thread.onRun(publisher);
  thread.setInterval(200);
  threadControl.add(&thread);
  
  setup_imu();
  
  setup_webserver();
  
  if (SPIFFS.begin()) {
      Serial.println("SPIFFS started");
  }
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  threadControl.run();
  server.handleClient();
  webSocket.loop();
}


