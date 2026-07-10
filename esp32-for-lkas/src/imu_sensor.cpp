#include "imu_sensor.hpp"

#include <Adafruit_Sensor.h>
#include <Wire.h>

bool ImuSensor::begin(int sda_pin, int scl_pin) {
  Wire.begin(sda_pin, scl_pin);
  if (!mpu_.begin()) {
    return false;
  }

  mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}

// Adafruit's unified sensor event already reports SI units (m/s^2, rad/s), matching
// sensor_msgs/Imu directly -- no unit conversion needed here.
ImuSample ImuSensor::read() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu_.getEvent(&accel, &gyro, &temp);

  ImuSample sample;
  sample.linear_acceleration[0] = accel.acceleration.x;
  sample.linear_acceleration[1] = accel.acceleration.y;
  sample.linear_acceleration[2] = accel.acceleration.z;
  sample.angular_velocity[0] = gyro.gyro.x;
  sample.angular_velocity[1] = gyro.gyro.y;
  sample.angular_velocity[2] = gyro.gyro.z;
  return sample;
}
