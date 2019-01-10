#ifndef WEMOS_STEPPER_JH
#define WEMOS_STEPPER_JH

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#define SLAVE_ADDRESS_1 0x32
//Set Commands
#define MYSLAVE_SET_REG 0x01
//GET commands
#define MOTOR_GET_COUNT 0x02
#define MOTOR_SET_COUNT 0x03
#define MOTOR_SET_TARGET 0x04
#define MOTOR_GET_TARGET 0x05
#define MOTOR_SET_TICKS 0x06
#define MOTOR_SET_DIR_RES 0x7

#include <Wire.h>

class WEMOS_STEPPER{
  public:
  //CONSTRUCTEUR
    WEMOS_STEPPER();
  //FONCTIONS
    void setup();
    void setResolution(byte microstep);
    void setTarget(int32_t atarget);
    void setSpeed(int32_t aspeed);
    void setPos(int32_t pos);
    bool isRunning();
    //VARIABLES
      int adresse;
      int32_t pos=0;//set motor counter to 0
      int32_t target= 3000;//set desired position 1000
      int32_t speed=10;//20,8 us * 300 =0.624s  ->16,02 microsteps/s
      byte microstep=1; //motor will no run on 16 msteps
  private:
    int32_t read_wms(uint8_t command, int address);
    void write_wms( uint8_t command, int address,int32_t value);
  };
  
#endif
