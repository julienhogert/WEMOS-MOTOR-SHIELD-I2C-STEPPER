#include <Arduino.h>
#include "wemos_stepper_jh.h"

//CONSTRUCTEUR
WEMOS_STEPPER::WEMOS_STEPPER(){
};

//FONCTIONS
void WEMOS_STEPPER::setup(){
  if(!Serial)Serial.begin(115200);
  Wire.begin(); setPos(position); setSpeed(speed); setTarget(target);
};

void WEMOS_STEPPER::setMS(byte aMicrostep){
  setResolution(aMicrostep);
};

void WEMOS_STEPPER::setResolution(byte aMicrostep){
  microstep=aMicrostep;
  int32_t resolution=32/microstep;
  write_wms(MOTOR_SET_DIR_RES,SLAVE_ADDRESS_1,resolution);
  Serial.printf("[WEMOS STEPPER] Microsteps:%i\n",microstep);
};

void WEMOS_STEPPER::setTarget(int32_t aTarget){
  if((inverseDirection?aTarget:-aTarget)!=target){ // on ne change qui si la valeur appelée est differente de l'ancienne
    if (Home_set && aTarget<0){
      Serial.println("Position outside of the stepper range (home button) Target=0");
      aTarget=0; //on ne va pas au dela du bouton Home
    }
    previous_target=target; //on definit l'ancienne cible
    target=(inverseDirection?aTarget:-aTarget); //en fonction de InverseDirection, on change le signe de la Target
    getPosition(); // on recupère la position actuelle du moteur
    interval_bt_target=millis()-target_millis; //calcul du dernier interval entre 2 targets
    target_millis=millis(); // on redifinit AutoSpeed_millis pour le prochain interval
    // mode "AutoSpeed"
    if(autoSpeed && interval_bt_target<1000){ //si l'interval est plus grand que 1s, on ne fait pas d'autospeed
      interval_targets=(target-position)/32; 
      autoSpeed_v = interval_targets*(1000/50);
      if(autoSpeed_v!=0) speed= autoSpeed_v;
    }
      write_wms(MOTOR_SET_TARGET,SLAVE_ADDRESS_1,target);
      if(target!=position) setSpeed(target-position<0 ? neg(speed) : pos(speed)); // on definit le signe de la vitesse en fonction de la position et de la cible
      if(alwaysWait) wait(); //si alwaysWait, on attend que le stepper arrive en position
  print();
  }
};
void WEMOS_STEPPER::go(int32_t aTarget){
  setTarget(aTarget);
};

// ---------------   SPEED FUNCTIONS

//revolution per Sec.
void WEMOS_STEPPER::setRPS(float aRps){
    setSpeed((aRps*STEPS_PER_REV));
};
void WEMOS_STEPPER::setRotPerSec(float aRps){setRPS(aRps);};

//Sec per Revolution
void WEMOS_STEPPER::SecsPerRev(float aSpr){
  setSpeed(float(STEPS_PER_REV/aSpr));
};
void WEMOS_STEPPER::setSPR(float aSpr){SecsPerRev(aSpr);
};

//revolution per Min.
void WEMOS_STEPPER::setRPM(float aRpm){
   setRPS((aRpm/60));
};
void WEMOS_STEPPER::setRotPerMin(float aRpm){ setRPM(aRpm);};

//SetSpeed
void WEMOS_STEPPER::setSpeed(int aSpeed){  // step/s
  if(abs(aSpeed)>maxSpeed){// limite la vitesse du stepper a MaxSpeed
    speed=(aSpeed>0?maxSpeed:-maxSpeed);
  }
  else speed=aSpeed;
  //******* CONVERSION SPEED EN SPEED I2C *********
  speed_i2c=float(float(STEPS_PER_REV/float(aSpeed))/float(FREQUENCY_MS*BASE_MS*STEPS_PER_REV));
  //************************************************
  if(speed_i2c==0) speed_i2c=(speed>0?1:-1);// evite que speed_i2c=0; 
  setSpeed_i2c(speed_i2c);
};
void WEMOS_STEPPER::setSpeed_i2c(int32_t aSpeed_i2c){
  speed_i2c=aSpeed_i2c;
  write_wms(MOTOR_SET_TICKS,SLAVE_ADDRESS_1,speed_i2c);
};
void WEMOS_STEPPER::setMaxSpeed(int aMaxSpeed){
  maxSpeed=aMaxSpeed;
};

// ------------------- POSITION FUNCTIONS

void WEMOS_STEPPER::setAngle(float aAngle){
    mode=ANGLE;
    setTarget(AngleToStep(aAngle));
};
void WEMOS_STEPPER::setPos(int32_t aPosition){
  position=aPosition;
  write_wms(MOTOR_SET_COUNT,SLAVE_ADDRESS_1,position);
};
void WEMOS_STEPPER::resetPosition(){
  setPos(0);
}
void WEMOS_STEPPER::stop(){
  setTarget(getPosition());
};
void WEMOS_STEPPER::wait(){
  long begin_wait=millis();
  int sec=1000;
  while(isRunning()){
    yield();
     if(begin_wait+sec<millis()){
      if(sec==1000)Serial.print("[WEMOS STEPPER] Wait for position");
      Serial.print(".");
      sec+=1000;
    }
  }
  //Serial.println("achieved!");
};

// ----------------- FONCTIONS 
bool WEMOS_STEPPER::isRunning(){
  getPosition();
  //Serial.printf("IsRunning() pos/target:%i %i => mouvement %s\n",position,target, (position==target ? "Non" : "Oui"));
  return (position==target ? 0 : 1);
};

int32_t WEMOS_STEPPER::getPosition(){
  position=read_wms( MOTOR_GET_COUNT ,SLAVE_ADDRESS_1);
  return position;
}
void WEMOS_STEPPER::goHome(byte aPin, bool aDir, int aSpeed=400){
  home_switch=aPin;
  pinMode(home_switch, INPUT_PULLUP);
  inverseDirection=aDir;
  setTarget((aDir?1000000:-1000000));
  setSpeed(aSpeed);
  Serial.print("[WEMOS STEPPER] Homing");
  long begin_homing=millis();
  int sec=1000;
  while(true){
    yield();
    if(begin_homing+sec<millis()){
      Serial.print(".");
      sec+=1000;
    }
    if(atHome()){
      Serial.println("Home!");
      Home_set=true;
      break;
    }
    if(begin_homing+(Home_timeOut*1000)<millis()){
      Serial.println("too long - aborted");
      enable=0;
      break;
    }
  }
  delay(100);
  resetPosition();
  stop();
};
void WEMOS_STEPPER::goHome(byte aPin, bool aDir){
  goHome(aPin, aDir,maxSpeed);
};
bool WEMOS_STEPPER::atHome(){
  return (digitalRead(home_switch)?0:1);
};

void WEMOS_STEPPER::print(){
  if(print_enable && print_millis+print_interval<millis()){
    print_millis=millis();
    Serial.print("[WEMOS STEPPER]");
    //POSITION
    if(mode==STEP)Serial.printf(" Pos:%i |",(inverseDirection?position:-position));
    if(mode==ANGLE)Serial.printf(" Pos:%sº |",c_float(stepToAngle((inverseDirection?position:-position))));
    //TARGET
    if(mode==STEP)Serial.printf(" Targ:%i |",(inverseDirection?target:-target));
    if(mode==ANGLE)Serial.printf(" Targ:%sº |",c_float(stepToAngle((inverseDirection?target:-target))));
    //SPEED
    if(mode==STEP) Serial.printf(" Spd:%i steps/s ",speed);
    else if(mode==ANGLE) Serial.printf(" Spd: %sº/s ",c_float(float(speed/STEPS_PER_REV)));
    if (autoSpeed) {
      Serial.printf("(autoSpd:%i)",autoSpeed_v);
      Serial.printf("int:%i | diff target:%i |",interval_bt_target,interval_targets);
    }
    Serial.printf("| (i2c: %ld) | ",speed_i2c);
    
    //Serial.printf(" Moving?:%s",(isRunning()?"Yes":"No"));
    Serial.printf("intvl: %i ms.",interval_bt_target);
    Serial.println();
    }
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

// UTILITIES
float WEMOS_STEPPER::stepToAngle(int32_t step){
  float angle =(float(step)/(STEPS_PER_REV*BASE_MS))*360;
  return angle;
};
int32_t WEMOS_STEPPER::AngleToStep(float angle){
  return (angle/360)*(STEPS_PER_REV*BASE_MS);
};
const char* WEMOS_STEPPER::c_float(float afloat){ // converti la variable Float en string
  char float_c[6];
  dtostrf(afloat, 4, 2, float_c);
  return float_c;
}

//calcul durée de la boucle

long lastMillis_loop=millis();

void duree_boucle(){
  long duree_loop = millis()-lastMillis_loop;
  lastMillis_loop=millis();
  float duree_loop_s = float(duree_loop)/1000;
  Serial.print("Durée de la boucle (s) :");
  Serial.println(duree_loop_s);
}
