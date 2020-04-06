
#include <ros.h>
#include <geometry_msgs/Vector3.h>


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SampleRate = 10;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

ros::NodeHandle handle;
geometry_msgs::Vector3 vector3;

ros::Publisher handlepublisher("target_publisher", &vector3);

double xPos = 0, yPos = 0, zPos = 0;

void setup() {

  handle.initNode();
  handle.advertise(handlepublisher);
  
  Serial.begin(500000);
  Serial.println(F("Starting BNO055 Test!"));
  if (!bno.begin()) {
    while (1);
    Serial.println(F("No BNO055 detected... Check Wiring or I2C Address"));
  }

  delay(1000);
}

void loop() {
//  sensors_event_t orientationData, angularVelocityData, linearAccelerationData;
//
//  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
////  bno.getEvent(&angularVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
////  bno.getEvent(&linearAccelerationData, Adafruit_BNO055::VECTOR_LINEARACCEL);
//
//  // testing!
//  //printEvent(&orientationData);
//  //printEvent(&angularVelocityData);
//  printEvent(&linearAccelerationData);
//
//  xPos = linearAccelerationData.orientation.x;
//  yPos = orientationData.orientation.y;
//  zPos = orientationData.orientation.z;
//  
////  int8_t boardTemp = bno.getTemp();
////  Serial.print("BNO board temperature: ");
////  Serial.print(boardTemp);
//
//  delay(BNO055_SampleRate);
//
//  vector3.x = xPos;
//  vector3.y = yPos;
//  vector3.z = zPos;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = euler.x();
  yPos = euler.y();
  zPos = euler.z();

  vector3.x = xPos;
  vector3.y = yPos;
  vector3.z = zPos;

  Serial.println(F(""));
  Serial.print(xPos);
  Serial.print(F(", "));
  Serial.print(yPos);
  Serial.print(F(", "));
  Serial.print(zPos);

  delay(BNO055_SampleRate);

  handlepublisher.publish(&vector3);
  
  handle.spinOnce();
  //delay(1);
}
