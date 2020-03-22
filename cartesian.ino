/*

    Cartesian, Hafidh Satyanto

    Based on rosserial, will send vector3 position messages based on processed BNO055 accelerometer data.
    The BNO055 has built-in motion processors that (annoyingly) auto-calibrates its sensors, but also
    does the heavy sensor fusion stuff so I don't have to (yeet). So, the BNO055 already outputs linearized,
    processed acceleration data based on its gyroscope (which should remove any gravity-influenced values) 
    so we don't need to do AHRS sensor fusion algorithms.

*/


// ROS includes
#include <ArduinoTcpHardware.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>

// Arduino includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// we are integrating so we'll need some... time
uint16_t SampleRateDelay = 15;  // poll / sampling rate from the imu sensor
uint16_t PrintRateDelay = 500;  // we're printing data
uint16_t PrintCounter = 0;      // we're counting our prints so we don't print every sample (15ms)

// our position values!
double xPos = 0, yPos = 0, zPos = 0;
double Acceleration_Velocity_Transition = (double)(SampleRateDelay) / 1000.0;
double Acceleration_Position_Transition = 0.5 * Acceleration_Velocity_Transition * Acceleration_Velocity_Transition;

// the imu sensor outputs in degrees but we'll need to use radians
const double DegreeRadianConversion = 0.01745329251;

// Adafruit BNO055 sensor class (all the hardware-level drivers)
Adafruit_BNO055 Sensor = Adafruit_BNO055(55, 0x28);

void vectorCallback(const geometry_msgs::Vector3& vector3) {

}

// ROS node class initializers
ros::NodeHandle Handle;
ros::Subscriber<geometry_msgs::Vector3> TargetPoseSubscriber("target_pose", vectorCallback);

void setup() {

    // initialize the ROS stuff
    Handle.initNode();
    Handle.subscribe(TargetPoseSubscriber);

    // start a serial monitor for debugging purposes
    Serial.begin(115200);

    // wait until sensor is connected / initialized
    if (Sensor.begin()) {
        Serial.print("No BNO055 detected");
        while (1);
    }

}

void loop(void) {

    unsigned long tStart = micros();
    sensors_event_t OrientationData;
    sensors_event_t LinearAccelerationData;

    Sensor.getEvent(&OrientationData, Adafruit_BNO055::Vector_EULER);
    Sensor.getEvent(&LinearAccelerationData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    xPos = xPos + Acceleration_Position_Transition * LinearAccelerationData.acceleration.x;
    yPos = yPos + Acceleration_Position_Transition * LinearAccelerationData.acceleration.y;
    zPos = zPos + Acceleration_Position_Transition * LinearAccelerationData.acceleration.z;

    if (PrintCounter * SampleRateDelay >= PrintRateDelay) {

        Serial.println("Position X: ");
        Serial.print(xPos);
        Serial.println("(Linear) Acceleration X: ");
        Serial.print(LinearAccelerationData.acceleration.x);

        Serial.println("Position Y: ");
        Serial.print(yPos);
        Serial.println("(Linear) Acceleration Y: ");
        Serial.print(LinearAccelerationData.acceleration.y);

        Serial.println("Position Z: ");
        Serial.print(zPos);
        Serial.println("(Linear) Acceleration Z: ");
        Serial.print(LinearAccelerationData.acceleration.z);

        Serial.println("------------------------------");

        PrintCounter = 0;
    } else {
        PrintCounter = PrintCounter + 1;
    }

    // We are awaiting a poll so that the imu sensor can sample
    while ((micros() - tStart) < (SampleRateDelay * 1000)) {}   

    // ROS spin
    Handle.spinOnce();
    delay(1);
}