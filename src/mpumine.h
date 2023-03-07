#ifndef mpumine_h
#define mpumine_h

// #include <Arduino.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>

#include "Wire.h"
#include <MPU6050_light.h>

class Mpumine {
  private:
  //  Adafruit_MPU6050 mpu;
    MPU6050 mpu;
    float gForce;
  public:
    Mpumine(TwoWire kabel);
    void begin();
    float readacc_x(); 
    float readacc_y();
    float readacc_z();
    float readgyro_x();
    float readgyro_y();
    float readgyro_z();
    float readGforce();
    int error_cek();
    float read_roll();
    float read_pitch();
    float read_tiltx();
    float read_tilty();
    void update_sens();
};

#endif
