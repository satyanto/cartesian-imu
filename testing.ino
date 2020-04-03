
#include <ros.h>
#include <geometry_msgs/Vector3.h>


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SampleRate = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

ros::NodeHandle handle;
geometry_msgs::Vector3 vector3;

ros::Publisher handlepublisher("target_publisher", &vector3);

double xPos = 0, yPos = 0, zPos = 0;

void setup() {

  handle.initNode();
  handle.advertise(handlepublisher);
  
  Serial.begin(115200);
  Serial.println("Starting BNO055 Test!");
  if (!bno.begin()) {
    while (1);
    Serial.println("No BNO055 detected... Check Wiring or I2C Address");
  }

  delay(1000);
}

void loop() {
  sensors_event_t orientationData, angularVelocityData, linearAccelerationData;

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
//  bno.getEvent(&angularVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
//  bno.getEvent(&linearAccelerationData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // testing!
  //printEvent(&orientationData);
  //printEvent(&angularVelocityData);
  printEvent(&linearAccelerationData);

  xPos = orientationData.orientation.x;
  yPos = orientationData.orientation.y;
  zPos = orientationData.orientation.z;
  
//  int8_t boardTemp = bno.getTemp();
//  Serial.print("BNO board temperature: ");
//  Serial.print(boardTemp);

  delay(BNO055_SampleRate);

  vector3.x = xPos;
  vector3.y = yPos;
  vector3.z = zPos;

  handlepublisher.publish(&vector3);
  
  handle.spinOnce();
  delay(1);
}

void printEvent(sensors_event_t* event) {
  Serial.println(event->type);
  double x = -1000000, y = -1000000, z = -1000000;

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }

  Serial.print(":  x=");
  Serial.print(x);
  Serial.print("  y=");
  Serial.print(y);
  Serial.print("  z=");
  Serial.print(z);

  Serial.println();
}