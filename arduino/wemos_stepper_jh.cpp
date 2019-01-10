#include <Arduino.h>
#include "wemos_stepper_jh.h"

//CONSTRUCTEUR
WEMOS_STEPPER::WEMOS_STEPPER(){
};

//FONCTIONS
void WEMOS_STEPPER::setup(){
  Wire.begin(); setPos(pos); setSpeed(speed); setTarget(target);
};
void WEMOS_STEPPER::setResolution(byte amicrostep){
  microstep=amicrostep;
  int32_t resolution=32/microstep;
  write_wms(MOTOR_SET_DIR_RES,SLAVE_ADDRESS_1,resolution);
};
void WEMOS_STEPPER::setTarget(int32_t atarget){
    target=atarget;
    write_wms(MOTOR_SET_TARGET,SLAVE_ADDRESS_1,target);
};
void WEMOS_STEPPER::setSpeed(int32_t aspeed){
  speed=aspeed;
   write_wms(MOTOR_SET_TICKS,SLAVE_ADDRESS_1,speed);
};
void WEMOS_STEPPER::setPos(int32_t apos){
  pos=apos;
   write_wms(MOTOR_SET_COUNT,SLAVE_ADDRESS_1,pos);
};
bool WEMOS_STEPPER::isRunning(){
  pos=read_wms( MOTOR_GET_COUNT ,SLAVE_ADDRESS_1);
  return (pos!=target ? 1 : 0);
};
// FONCTIONS I2C WIRE
int32_t WEMOS_STEPPER::read_wms( uint8_t command, int address){
  int32_t pos;
  char *pointer;
  char c;
  pointer=(char*)(&pos);
 Wire.beginTransmission(address);
  Wire.write(command);
  Wire.endTransmission();
  delay(5);
   Wire.requestFrom(address, 4);
    while (Wire.available()) {
    char c = Wire.read();
    (*pointer++)=c;
    }
    return pos;
};

void WEMOS_STEPPER::write_wms( uint8_t command, int address,int32_t value){
  char *pointer;
  pointer=(char*)(&value);
 Wire.beginTransmission(address);
  Wire.write( command);
  Wire.write(pointer,4);
  Wire.endTransmission();
};
