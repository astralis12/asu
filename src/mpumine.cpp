// #include <Arduino.h>
// #include "mpumine.h"
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>

// #define alpha 0.95

// Mpumine::Mpumine():mpu(Adafruit_MPU6050()){}

// int eror;float roll,pitch,x,y,z,gyro_x,gyro_y,gyro_z;

// void Mpumine::begin() {
//   mpu.begin();
//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
// }

// float Mpumine::readacc_x() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   x = accel.acceleration.x;
//   return accel.acceleration.x;
// }
// float Mpumine::readacc_y() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   y = accel.acceleration.y;
//   return accel.acceleration.y;
// }
// float Mpumine::readacc_z() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   z = accel.acceleration.z;
//   return accel.acceleration.z;
// }
// float Mpumine::readgyro_x() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   gyro_x = gyro.gyro.x;
//   return gyro.gyro.x;
// }
// float Mpumine::readgyro_y() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   gyro_y = gyro.gyro.y;
//   return gyro.gyro.y;
// }
// float Mpumine::readgyro_z() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   gyro_z = gyro.gyro.z;
//   return gyro.gyro.z;
// }

// float Mpumine::readGforce() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   gForce = sqrt(accel.acceleration.x * accel.acceleration.x +
//                 accel.acceleration.y * accel.acceleration.y +
//                 accel.acceleration.z * accel.acceleration.z) / 9.80665;
//   return gForce;
// }

// int Mpumine::error_cek() {
//   Wire1.beginTransmission(0x68);  //buat baca mpu nyambung atau tidak pakai address mpu 0x68
//   return Wire1.endTransmission();
// }

// float Mpumine::read_roll() {
//   roll = atan2(x, y) * RAD_TO_DEG;
//   float gyroRoll = roll + gyro_x * 0.01;
//   roll = alpha * gyroRoll + (1 - alpha) * roll;
//   return roll;
// }

// float Mpumine::read_pitch() {
//   pitch = atan2(-x, sqrt(y * y + z * z)) * RAD_TO_DEG;
//   float gyroPitch = pitch + gyro_y * 0.01;
//   pitch = alpha * gyroPitch + (1 - alpha) * pitch;
//   return pitch;
// }

// float Mpumine::read_tiltx() {
//   float totax = atan2(y, sqrt(x * x + z * z)) * 180 / PI;
//   gyro_x += gyro_x * 0.01;
//   float totgx= gyro_x;
//   return (0.98 * totgx + 0.02 * totax)*100.0;
// }

// float Mpumine::read_tilty() {
//   float totay = atan2(-x, sqrt(y * y + z * z)) * 180 / PI;
//   gyro_y += gyro_y * 0.01;
//   float totgy = gyro_y;
//   return (0.98 * totgy + 0.02 * totay)*100.0;
// }

// #include <Arduino.h>
// #include "mpumine.h"
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>

// #define alpha 0.95

// Mpumine::Mpumine():mpu(Adafruit_MPU6050()){}

// int eror;float roll,pitch,x,y,z,gyro_x,gyro_y,gyro_z;

// void Mpumine::begin() {
//   mpu.begin();
//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
// }

// float Mpumine::readacc_x() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   x = accel.acceleration.x;
//   return accel.acceleration.x;
// }
// float Mpumine::readacc_y() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   y = accel.acceleration.y;
//   return accel.acceleration.y;
// }
// float Mpumine::readacc_z() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   z = accel.acceleration.z;
//   return accel.acceleration.z;
// }
// float Mpumine::readgyro_x() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   gyro_x = gyro.gyro.x;
//   return gyro.gyro.x;
// }
// float Mpumine::readgyro_y() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   gyro_y = gyro.gyro.y;
//   return gyro.gyro.y;
// }
// float Mpumine::readgyro_z() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   gyro_z = gyro.gyro.z;
//   return gyro.gyro.z;
// }

// float Mpumine::readGforce() {
//   sensors_event_t accel,gyro,tempe;
//   mpu.getEvent(&accel, &gyro, &tempe);
//   gForce = sqrt(accel.acceleration.x * accel.acceleration.x +
//                 accel.acceleration.y * accel.acceleration.y +
//                 accel.acceleration.z * accel.acceleration.z) / 9.80665;
//   return gForce;
// }

// int Mpumine::error_cek() {
//   Wire1.beginTransmission(0x68);  //buat baca mpu nyambung atau tidak pakai address mpu 0x68
//   return Wire1.endTransmission();
// }

// float Mpumine::read_roll() {
//   roll = atan2(x, y) * RAD_TO_DEG;
//   float gyroRoll = roll + gyro_x * 0.01;
//   roll = alpha * gyroRoll + (1 - alpha) * roll;
//   return roll;
// }

// float Mpumine::read_pitch() {
//   pitch = atan2(-x, sqrt(y * y + z * z)) * RAD_TO_DEG;
//   float gyroPitch = pitch + gyro_y * 0.01;
//   pitch = alpha * gyroPitch + (1 - alpha) * pitch;
//   return pitch;
// }

// float Mpumine::read_tiltx() {
//   float totax = atan2(y, sqrt(x * x + z * z)) * 180 / PI;
//   gyro_x += gyro_x * 0.01;
//   float totgx= gyro_x;
//   return (0.98 * totgx + 0.02 * totax)*100.0;
// }

// float Mpumine::read_tilty() {
//   float totay = atan2(-x, sqrt(y * y + z * z)) * 180 / PI;
//   gyro_y += gyro_y * 0.01;
//   float totgy = gyro_y;
//   return (0.98 * totgy + 0.02 * totay)*100.0;
// }

#include "mpumine.h"
#include "Wire.h"
#include <MPU6050_light.h>
#define Wire Wire1

Mpumine::Mpumine(TwoWire kabel):mpu(MPU6050(Wire)) {}

void Mpumine::update_sens() {
  mpu.update();
}

void Mpumine::begin() {
  Wire1.begin();
  mpu.begin();
  delay(1000);
  mpu.calcOffsets(true,true);
}

float Mpumine::readacc_x() {
  //mpu.update();
  return mpu.getAccX();
}

float Mpumine::readacc_y(){
  //mpu.update();
  return mpu.getAccY();
}

float Mpumine::readacc_z(){
  //mpu.update();
  return mpu.getAccZ();
}

float Mpumine::readgyro_x(){
  //mpu.update();
  return mpu.getGyroX();
}

float Mpumine::readgyro_y(){
  //mpu.update();
  return mpu.getGyroY();
}

float Mpumine::readgyro_z(){
  //mpu.update();
  return mpu.getGyroZ();
}

float Mpumine::readGforce(){
  //mpu.update();
  gForce = sqrt(mpu.getAccX()* mpu.getAccX() +
  mpu.getAccY()* mpu.getAccY()+
  mpu.getAccZ()* mpu.getAccZ()) / 9.80665;
  return gForce;
}

float Mpumine::read_tiltx(){
  //mpu.update();
  return mpu.getAngleX();
}

float Mpumine::read_tilty(){
  //mpu.update();
  return mpu.getAngleY();
}
