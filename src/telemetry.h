#ifndef telemetry_h
#define telemetry_h

#include <Arduino.h>
#include "component.h"
extern float accelX,accelY,temp,press,altit,lat,lng;
float sim_press;
enum State {
    STANDBY     =  0,
    ASCENT      =  1,
    SEPARATION  =  2,
    DESCENT     =  3,
    HS_RELEASE  =  4,
    PP_RELEASE  =  5,
    LANDED      =  6,
    INVALID     = -1
};

    String mode;
    char HS_DEPLOYED = 'N';
    char PC_DEPLOYED = 'N';
    char MAST_RAISED = 'N';
    byte State;
    String cmdEcho = "INVALID";
    bool state_1,state_2,state_3,state_4,state_5,state_6,state_7;
    
    /* Pressure sensor */
    float altitude=altit; char str_altitude[10]; //dtostrf(altitude, 1, 1, str_altitude);
    float temperature=temp; char str_temperature[10]; //dtostrf(temperature, 1, 1, str_temperature);
    float pressure=press; char str_pressure[10]; //dtostrf(pressure, 1, 1, str_pressure);

    /* IMU */
    float tiltX=accelX, tiltY=accelY;char str_x[20];char str_y[20];
    //dtostrf(tiltX, 1, 2, str_x); dtostrf(tiltY, 1, 2, str_y);

    /* Voltage divider */
    float voltage; char str_voltage[10]; //dtostrf(voltage, 1, 2, str_voltage);

    /* GPS */
    int hhGPS ;
    int mmGPS ;
    int ssGPS ;
    float latitudeGPS;char str_lat[20];// dtostrf(latitudeGPS, 1, 8, str_lat);
    float longitudeGPS;char str_lng[20];// dtostrf(longitudeGPS, 1, 8, str_lng);
    float altitudeGPS;char str_altigps[10];// dtostrf(altitudeGPS, 1, 1, str_altigps);
    int satsGPS;
    
    unsigned long int packetCount = 0;
    const char teamID[5] = "1084";
    bool tele_command=false,tele_calibration=false,tele_enable=false,tele_sim=false,tele_activate=false;
    const char* getState() {
        //int rand = random(0,4);
        switch(State){
            case STANDBY:
                return "STANDBY";
            case ASCENT:
                return "ASCENT";
            case SEPARATION:
                component().begin_pinservo(3);
                component().servo_run(3);
                return "SEPARATION";
            case DESCENT:
                return "DESCENT";
            case HS_RELEASE:
                component().begin_pinservo(4);
                component().servo_run(4);
                return "HS_RELEASE";
            case PP_RELEASE:
                component().begin_pinservo(5);
                component().servo_run(5);
                return "PP_RELEASE";
            case LANDED:
                component().begin_pinbuzzer(6);
                component().begin_pinflag(7);
                component().buzzer_run(6);
                component().flag_run(7);
                return "LANDED";
            default:
                return "INVALID";
        }
    }

class telemetry {
    private:
    public:
    telemetry(){};
    void detect_state(float paket, float gforce, float press, float altitude, float before_alti) {
    if (paket==0) { state_1 = true;
        telemetry().state(0); //standby
     }
    if (altitude>1&&state_3!=true) { state_2 = true;
        telemetry().state(1); //ascent
    }
    if ((before_alti>altitude+1)&&(altitude<600)&&(altitude>500)) { state_3 = true;
        telemetry().state(2); //command to separation
    }
    if (altitude>400&&state_3==true&&altitude<500) { state_4 = true;
        telemetry().state(3); //descent
    }
    if (altitude<500&&altitude>200&&state_4==true) { state_5 = true;
        telemetry().state(4); //hsrelease
    }
    if (altitude<200&&altitude>100&&state_5==true) { state_6 = true;
        telemetry().state(5); //pprelease
    }
    if (paket>400&&altitude<10) { state_7 = true;
        telemetry().state(6); //landing
    }
    }
    void detect_mode (bool modeparam) {
        if (modeparam==true) {
            mode = "S";
        }
        else {
            mode = "F";
        }
    }
    void state(int condition) {
        State = condition;
    }
    void tele_readcomm(String head, String label, String mode, String content) {
    if (String(head+label+mode+content)=="CMD1084CXON"){ //kalo di string ada kode maka do something
    tele_command = true;;cmdEcho = "CXON";}
    if (String(head+label+mode+content)=="CMD1084CXOFF"){ //kalo di string ada kode maka do something
    tele_command = false;;cmdEcho = "CXOFF";}
    if (String(head+label+mode+content)=="CMD1084CAL"){ //kalo di string ada kode maka do something
    tele_calibration = true;;cmdEcho = "CAL";
    }
    if (String(head+label+mode+content)=="CMD1084SIMACTIVATE"){ //kalo di string ada kode maka do something
    tele_activate = true;;cmdEcho = "SIMACTIVATE";
    }
    if (String(head+label+mode+content)=="CMD1084SIMENABLE"){ //kalo di string ada kode maka do something
    tele_enable = true;;cmdEcho = "SIMENABLE";
    }
    if (String(head+label+mode+content)=="CMD1084SIMDISABLE"){ //kalo di string ada kode maka do something
    tele_enable = false,tele_sim=false,tele_activate=false;;cmdEcho = "SIMDISABLE";
    }
    if (String(head+label+mode)=="CMD1084SIMP"&&tele_enable==true&&tele_activate==true){ //kalo di string ada kode maka do something
    tele_sim = true;sim_press = content.toFloat()/100.0;;cmdEcho = String(sim_press);
    }
    }
    void distort (float a,float t,float p,float x,float y,float v, int j, int m, int d, float lat, float lng, float gps_alti, int satelite) {
    memset(str_altitude, 0, sizeof(str_altitude));
    memset(str_temperature, 0, sizeof(str_temperature));
    memset(str_pressure, 0, sizeof(str_pressure));
    memset(str_x, 0, sizeof(str_x));
    memset(str_y, 0, sizeof(str_y));
    memset(str_voltage, 0, sizeof(str_voltage));
    dtostrf(a, -4, 1, str_altitude);
    dtostrf(t, 3, 1, str_temperature);
    dtostrf(p, 6, 1, str_pressure);
    dtostrf(x, 4, 2, str_x);
    dtostrf(y, 4, 2, str_y);
    dtostrf(v, 3, 2, str_voltage);
    dtostrf(lat, 3, 6, str_lat);
    dtostrf(lng, 3, 6, str_lng);
    dtostrf(gps_alti, -4, 1, str_altigps);
    hhGPS = j;
    mmGPS = m;
    ssGPS = d;
    satsGPS = satelite;
    }
    String constructMessage() {
    memset(buffer, 0, sizeof(buffer));
    snprintf(buffer, sizeof(buffer),
              "%s,"
              "%02d:%02d:%02d,"
              "%ld,"
              "%s,%s,%s,"
              "%c,%c,%c,"
              "%s,%s,%s,"
              "%02d:%02d:%02d,"
              "%s,%s,%s,"
              "%02d,%s,%s,%s\r",
              teamID,
              hhGPS, mmGPS, ssGPS,
              packetCount,
              mode.c_str(), getState(), str_altitude,
              HS_DEPLOYED, PC_DEPLOYED, MAST_RAISED,
              str_temperature, str_voltage, str_pressure,
              hhGPS, mmGPS, ssGPS,
              str_altigps, str_lat, str_lng,
              satsGPS, str_x, str_y, cmdEcho.c_str()
            );

        return String(buffer);
  };
  char buffer[256];
};

#endif