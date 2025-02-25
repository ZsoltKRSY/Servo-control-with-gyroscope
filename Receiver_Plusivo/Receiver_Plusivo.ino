#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>

#define DELAY_TIME 3

typedef struct rotationAngles {
  //normally it would be yaw, pitch, roll (it's the order on the Broadcaster module)
  int yaw, roll, pitch;  //changed up pitch and roll because it feels more natural to hold the "controller" (gyroscope) this way
} rotationAngles;

rotationAngles rAnglesOld, rAnglesNew;
unsigned int highestAngleDiff;

Servo servoYaw;
int servoYawPin = D1;
//unsigned int lastTimeYaw = 0, delayTimeYaw = 0;

Servo servoPitch;
int servoPitchPin = D2;
//unsigned int lastTimePitch = 0, delayTimePitch = 0;

Servo servoRoll;
int servoRollPin = D3;
//unsigned int lastTimeRoll = 0, delayTimeRoll = 0;

//int laserPin = D7;

long lastTime = 0;
long interval = 50;

void onDataReceive(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&rAnglesNew, incomingData, sizeof(rAnglesNew));
  Serial.print("Bytes received: ");
  Serial.println(len);

  Serial.print("Rotation angle Yaw: ");
  Serial.print(rAnglesNew.yaw);
  Serial.print(", Pitch: ");
  Serial.print(rAnglesNew.pitch);
  Serial.print(", Roll: ");
  Serial.print(rAnglesNew.roll);
  Serial.println("");
}
 
void setup() {
  Serial.begin(115200);

  //pinMode(laserPin, OUTPUT);
  //digitalWrite(laserPin, HIGH);
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  //Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataReceive));

  rAnglesOld.yaw = 90;
  rAnglesOld.pitch = 90;
  rAnglesOld.roll = 90;
  rAnglesNew.yaw = 90;
  rAnglesNew.pitch = 90;
  rAnglesNew.roll = 90;

  servoYaw.attach(servoYawPin);
  servoPitch.attach(servoPitchPin);
  servoRoll.attach(servoRollPin);
}

void operateServo(Servo servo, int lastValue, int newValue){
  servo.write(newValue);
  delay(DELAY_TIME);
}

void loop(){
  if((millis() - lastTime) >= interval){
    Serial.print("Rotation angle Yaw: ");
    Serial.print(rAnglesNew.yaw);
    Serial.print(", Pitch: ");
    Serial.print(rAnglesNew.pitch);
    Serial.print(", Roll: ");
    Serial.print(rAnglesNew.roll);
    Serial.println("");

    unsigned int currentTime = millis();
    
    operateServo(servoYaw, rAnglesOld.yaw, rAnglesNew.yaw);
    operateServo(servoPitch, rAnglesOld.pitch, rAnglesNew.pitch);
    operateServo(servoRoll, rAnglesOld.roll, rAnglesNew.roll);

    rAnglesOld = rAnglesNew;

    lastTime = millis();
  }
}
  