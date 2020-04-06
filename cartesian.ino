/*

    Cartesian
    Hafidh Satyanto

    This is an Arduino sketch that will connect to rosserial and publish linear acceleration data as a Vector3
    message on the /raw_linear_accel topic. The BNO055 has built-in motion processors that auto-calibrates its
    sensors and does the heavy sensor fusion stuff (yeet!) of (trying) to remove the gravity vector.

    The raw linear acceleration data was noisier than I thought. So, I will process it further through a separate
    ROS node (as it will run on my laptop instead of the Arduino) so calculations hopefully can be much faster.

    I tested different baud rates using an Arduino UNO with an ATMEGA328P on a Baud rate of 500,000 and it works
    through rosserial... I think it's overkill (and probably a bad idea because of data bit corruption) so...
    I toned it down to 230400 and 250000 but it seems like rosserial does not 'sync' at that rate, so...

    Sidenote: tested without ros::spinOnce() since there were no callbacks, I thought there was no need. WRONG!

*/


// ROS includes 
#include <ros.h>
#include <geometry_msgs/Vector3.h>


// Arduino & Sensor includes
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// The BNO055 has a sample rate of 100Hz, so our maximum sampling rate is 10 milliseconds. (10*1000 microseconds)
uint16_t imuSampleRate = 10000;


// Define our sensor class using the provided library - (by default the address is 0x28 using I2C)
Adafruit_BNO055 imuSensor = Adafruit_BNO055(55, 0x28);


// Defines for our ROS node handle, publisher, and message type
ros::NodeHandle imuHandler;
geometry_msgs::Vector3 imuVector;
ros::Publisher imuPublisher("raw_linear_accel", &imuVector);


void setup() {
    // Initialize our ROS node
    imuHandler.initNode();
    imuHandler.advertise(imuPublisher);
  
    // Open up our serial port at 250,000 baud rate.
    Serial.begin(500000);

    // Pre-check
    Serial.println(F("Starting BNO055...!"));
    if (!imuSensor.begin()) {
        while (1);
        Serial.println(F("No BNO055 detected - Check Wiring or I2C Address..."));
    }

    delay(1000);
}


void loop() {
    // We 'time-stamp' our current board time to more accurately 'poll' the sensor
    unsigned long curTime = micros();

    // The data type is provided by the imumaths library and we call the getVector function to get the data.
    imu::Vector<3> sensorData = imuSensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    imuVector.x = sensorData.x();
    imuVector.y = sensorData.y();
    imuVector.z = sensorData.z();

    // publish our Vector3 message through rosserial
    imuPublisher.publish(&imuVector);
    imuHandler.spinOnce();

    // We then compare our previous time stamp with the current time stamp to poll the sensor
    while ((micros() - curTime) < (imuSampleRate)) {
    }
}
