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
//Used for calculations
#define FREQUENCY_MS 20.8e-6
#define BASE_MS 32
#define STEPS_PER_REV 200.0
#define CCW 1
#define CW 0

//    tour/s = Speed x freq_base x 32ms x SpR
//    Speed = tr/s /  freq_base x 32ms x SpR
//    or Tr/s = nbstep_pers / Spr
//    =>
//    Speed = (Spr/nbstep_perS)  / ( freq_base x 32ms x SpR)


#include <Wire.h>

class WEMOS_STEPPER{
  public:
  //CONSTRUCTEUR
    WEMOS_STEPPER();

  //FONCTIONS

  //SPEED FUNCTIONS
    //revolution per Sec.
    void setRPS(float aRps);        
    void setRotPerSec(float aRps);
    //Sec per Revolution
    void SecsPerRev(float aSpr);
    void setSPR(float aSpr);
    //revolution per Min.
    void setRPM(float aRpm);
    void setRotPerMin(float aRpm);             //Sec per Revolution.
    //setSpeed(nb steps/s)
    void setSpeed(int aSpeed);
    void setMaxSpeed(int aMaxSpeed);
    //setSpeed(I2C original)
    void setSpeed_i2c(int32_t aSpeed_i2c);

  //POSITION FUNCTIONS
    void go(int32_t aTarget);
    void goHome(byte aPin, bool aDir,int aSpeed);
    void goHome(byte aPin, bool aDir);
    void setTarget(int32_t aTarget);
    void setAngle(float aAngle);
    void stop();

  // UTILITY FUNCTIONS
    void setup();
    void setResolution(byte aMicrostep);
    void setMS(byte aMicrostep);
    void resetPosition();
    bool atHome();
    bool isRunning();
    void print();
    void setPos(int32_t sPos);
    int32_t getPosition();
    void wait();

    //VARIABLES
      //VARIABLES PRINCIPALES
      bool enable=true;
      int32_t position=0;
      int32_t target= 0;
      int32_t previous_target= 0;
      int32_t interval_targets= 0;
      int speed=200; //Steps Per Second
      int maxSpeed=400; //Steps Per Second
      bool inverseDirection; //inverse le signe des targets (commodité pour etre dans un referentiel positif apres le homing);
      bool alwaysWait=false;
      int32_t speed_i2c; //speed conversion for i2c
      byte microstep=1;

      //HOME
      byte home_switch;
      int Home_timeOut=20; // timeout in sec.
      bool Home_set=0;
      
      //interval entre 2 Target (utile pour voir la synchro mouvement/stepper ou autospeed)
      long target_millis=millis();
      int interval_bt_target;
      //Mode "AutoSpeed" Compute speed in function of targets and intervals of reception
      bool autoSpeed=0;
      int autoSpeed_v;

      
      //PRINT interval
      int print_interval=100;
      long print_millis=millis();
      bool print_enable=0;
      enum { STEP,ANGLE, I2C };
      byte mode=0; //pour print

  private:
    int32_t read_wms(uint8_t command, int address);
    void write_wms( uint8_t command, int address,int32_t value);
    
    //utilities
    float stepToAngle(int32_t step);
    int32_t AngleToStep(float angle);
    const char* c_float(float afloat);
    };

//Functions to return a number in positive or negative
template <typename type>
type pos(type value){
  return (value<0 ? -value : value);
};
template <typename type>
type neg(type value){
  return( value>0 ? -value : value);
};

extern long lastMillis_loop;
extern void duree_boucle();

#endif
