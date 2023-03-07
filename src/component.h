#ifndef component_h   
#define component_h

#include <Arduino.h>

class component {
private: 
    int pinServo;
    int pinbuzzer;
    int pinflag;

public:
    component(){};
    void begin_pinservo(int pinServo) {
        pinMode(pinServo, OUTPUT);
    }
    void begin_pinbuzzer(int pinBuzzer) {
        pinMode(pinBuzzer, OUTPUT);
    }
    void begin_pinflag(int pinFlag) {
        pinMode(pinFlag, OUTPUT);
    }
    void servo_run(int pinServo) {  //90 degree
        digitalWrite(pinServo, HIGH);
        delayMicroseconds(2500);
        digitalWrite(pinServo, LOW);
        delayMicroseconds(2000);
    };
    void buzzer_run(int pinBuzzer) {
        digitalWrite (pinBuzzer, HIGH);
    }
    void flag_run(int pinFlag) {
        digitalWrite(pinFlag, HIGH);
        delayMicroseconds(2500);
        digitalWrite(pinFlag, LOW);
        delayMicroseconds(2000);
    }

};

#endif