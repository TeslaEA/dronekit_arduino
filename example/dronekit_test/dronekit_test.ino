#include <Dronekit.h>
#include <ESP8266WiFi.h>

const char* ssid     = "PixRacer";      // SSID
const char* password = "pixracer";      // Password
const int   port = 14550;            // Port serveur - Server Port

Dronekit v;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();

  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  v.connect(port);
}

void loop() {
  // put your main code here, to run repeatedly:
  mavlink_message_t m;
  m = v.receive_data();
  mavlink_heartbeat_t hb;
  mavlink_msg_heartbeat_decode(&m, &hb);
  Serial.println(hb.custom_mode);
  delay(100);
}
