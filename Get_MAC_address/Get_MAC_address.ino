#include <ESP8266WiFi.h>

void setup(){
  Serial.begin(115200);

  Serial.print("ESP Board MAC Address: ");
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}