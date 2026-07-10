#pragma once

#include <Adafruit_MPU6050.h>

#include "robot_types.hpp"

class ImuSensor {
 public:
  bool begin(int sda_pin, int scl_pin);
  ImuSample read();

 private:
  Adafruit_MPU6050 mpu_;
};
