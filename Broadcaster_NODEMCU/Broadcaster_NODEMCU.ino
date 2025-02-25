#include <ESP8266WiFi.h>
#include <espnow.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <KickSort.h>

//RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress[] = { 0x48, 0x55, 0x19, 0x14, 0xC6, 0xC6 };

unsigned long lastTime = 0;
unsigned long delayTime = 50;

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
float yawPitchRollNew[3], yawPitchRollOld[3];
uint8_t FIFOBuffer[64];
uint16_t packetSize;

typedef struct rAngles_struct {
  unsigned int yaw, pitch, roll;
} rAngles_struct;

rAngles_struct rAngles;
bool firstData = true;

const float PERMITTED_DIFF = 25.0f;

void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last packet Send status: ");
  if (sendStatus == 0)
    Serial.println("Delivery success");
  else
    Serial.println("Delivery fail");
}

void initESP_now() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(onDataSent);

  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void initMpu() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("Testing MPU6050 connection...");

  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }
  if (mpu.dmpInitialize() != 0) {
    Serial.println("DMP initialization failed");
  }

  // mpu.setXAccelOffset(-2451); //Set your accelerometer offset for axis X
  // mpu.setYAccelOffset(-1765); //Set your accelerometer offset for axis Y
  // mpu.setZAccelOffset(1947); //Set your accelerometer offset for axis Z
  // mpu.setXGyroOffset(10);  //Set your gyro offset for axis X
  // mpu.setYGyroOffset(25);  //Set your gyro offset for axis Y
  // mpu.setZGyroOffset(-13);  //Set your gyro offset for axis Z

  mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate the MPU6050
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);

  packetSize = mpu.dmpGetFIFOPacketSize();
}

void constrainToIntervalFloat(float *value, float minValue, float maxValue) {
  if ((*value) > maxValue)
    (*value) = maxValue;
  else if ((*value) < minValue)
    (*value) = minValue;
}

void getYawPitchRollFloat(float yawPitchRollFloat[]) {
  mpu.dmpGetQuaternion(&q, FIFOBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(yawPitchRollFloat, &q, &gravity);

  for(int i = 0; i < 3; ++i){
    yawPitchRollFloat[i] *= 180.0f / M_PI;
    constrainToIntervalFloat(&(yawPitchRollFloat[i]), -90.0f, 90.0f);
  }
}

void getRotationAngles(rAngles_struct *rAngles) {
  if(firstData){
    for(int i = 0; i < 3; ++i){
      yawPitchRollOld[i] = yawPitchRollNew[i];
    }
    firstData = false;
  }

  if(abs(yawPitchRollNew[0] - yawPitchRollOld[0]) < PERMITTED_DIFF){
    yawPitchRollOld[0] = yawPitchRollNew[0];
    rAngles->yaw = map((unsigned int)yawPitchRollNew[0], -90, 90, 180, 0);
  }
  if(abs(yawPitchRollNew[1] - yawPitchRollOld[1]) < PERMITTED_DIFF){
    yawPitchRollOld[1] = yawPitchRollNew[1];
    rAngles->pitch = map((unsigned int)yawPitchRollNew[1], -90, 90, 0, 180);
  }
  if(abs(yawPitchRollNew[2] - yawPitchRollOld[2]) < PERMITTED_DIFF){
    yawPitchRollOld[2] = yawPitchRollNew[2];
    rAngles->roll = map((unsigned int)yawPitchRollNew[2], -90, 90, 180, 0);
  }
}


void setup() {
  Serial.begin(115200);

  initMpu();
  initESP_now();
}

void loop() {
  if ((millis() - lastTime) > delayTime) {
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
      getYawPitchRollFloat(yawPitchRollNew);
      // Serial.print("ypr:\n");
      // Serial.print(yawPitchRollNew[0]);
      // Serial.print("\t");
      // Serial.print(yawPitchRollNew[1]);
      // Serial.print("\t");
      // Serial.println(yawPitchRollNew[2]);

      getRotationAngles(&rAngles);

      // Serial.print("angles:\t");
      // Serial.print(rAngles.yaw); Serial.print("\t");
      // Serial.print(rAngles.pitch); Serial.print("\t");
      // Serial.println(rAngles.roll);

      esp_now_send(broadcastAddress, (uint8_t *) &rAngles, sizeof(rAngles));
    }

    lastTime = millis();
  }
}