//upload this sketch to all of your esp32 to get there mac address

#include "WiFi.h"
 
void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}
