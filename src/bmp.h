#ifndef bmp_h   
#define bmp_h

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <EEPROM.h>

Adafruit_BMP3XX bmp;

float temperature_bmp,pressure_bmp,altitude_bmp,eeprom_ref;

class bmp_read {
  private:
    float reference_altitude;
    float eeprom_ref;

  public:
    void begin() {
      bmp.begin_I2C();  
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    };
    float read_temp() {
      return bmp.temperature;
    }; 
    float read_press() {
      return bmp.pressure / 100.0;
    };
    float read_altitude(float main_ref) {
      return bmp.readAltitude(main_ref);
    };
    float eeprom_value(float main_ref) {
      if (EEPROM.read(0)==255) {
        EEPROM.put(0, main_ref);
        EEPROM.get(0, eeprom_ref);
        return bmp.readAltitude(eeprom_ref);
      }
      return bmp.readAltitude(eeprom_ref);
    };
    float read_altitude_sim(float pc_press) {
      if (bmp.readAltitude(pc_press>0)) {
        return 0.00;
      }
      else {
        return bmp.readAltitude(pc_press);
      }
    };
    void tele_calibration(float ref) {
      bmp.begin_I2C();  
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp.setOutputDataRate(BMP3_ODR_50_HZ);
      
    };
    void flush_eeprom() {
      for (int i = 0; i < 256; i++) {
        EEPROM.write(i, 255);
      }
      Serial.print("Done flush, please comment the function");
    };
    void bmp_error(float ref) {

;
    }
    float output_bmp(float data) {
      if (data<=0) {
        return 0.0;
      }
      else {
        return data;
      }
    }
};

#endif