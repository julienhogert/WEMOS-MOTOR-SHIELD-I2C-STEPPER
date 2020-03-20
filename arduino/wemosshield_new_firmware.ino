#include <Wire.h>
#define SLAVE_ADDRESS_1 0x32
//Set Commands
#define MYSLAVE_SET_REG 0x01
//GET commands
#define MOTOR_GET_COUNT 0x02
#define MOTOR_SET_COUNT 0x03
#define MOTOR_SET_TARGET 0x04
#define MOTOR_GET_TARGET 0x05
#define MOTOR_SET_TICKS 0x06
//Is there any default value?

#define MOTOR_SET_DIR_RES 0x7

#define MOTOR_SET_PRESCALER 0x08 // :Send 0x08+ int32 (total 5bytes) **NEW
//What is it for ?

#define MOTOR_SET_WAVE_SCALE 0x09 // :Send 0x09+ int32 (total 5bytes) **NEW

//scale source voltage amplitude with WAVE_SCALE command.
//Its integer value indicates percentage o full wavE MAGNITUDE values from 0 to 100 by default 50%

#define MOTOR_SET_SPEED 0x0A //:Send 0x0A+float (total 5bytes) **NEW
#define MOTOR_SET_TARGET_SPEED 0x0B // :Send 0x0B+float (total 5bytes) **NEW
//Speed can be set in microsteps/s using 32bit signed float

// DOES IT MEANS THAT WE JUST PUT A TARGET AND THEN IT COMPUTE A SPEED AUTOMATICALLY?

// Values for set speed microsteps/s signed  4 bytes float
// SET_SPEED set speed with no acceleration SET_TARGET_SPEED accelerate from current set speed.


void setup() {
 Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
  Serial.printf("Initial motor count: %i \n",read_wms( MOTOR_GET_COUNT ,SLAVE_ADDRESS_1));
}

void loop() {

 //goto from 0 to 1000 
 int32_t pos=0;//set motor counter to 0
 int32_t target= 1000;//set desired position 1000
 // **OLD int32_t speed=3000;//20,8 us * 300 =0.624s  ->16,02 microsteps/s 
 //int32_t speed=1000; // BASICALLY DOES IT MEAN THAT THE SPEED IS 1000 microsteps/s NOW ?
 float speed=100; // SO NOW THE SPEED NEED TO BE A FLOAT, RIGHT?
 Serial.printf("Pos: %i / Target: %i\n",pos,target);
 
 write_wms(MOTOR_SET_COUNT,SLAVE_ADDRESS_1,pos);//send counter // WHY IS IT NECESSARY TO INITIALIZE COUNT?
 //write_wms(MOTOR_SET_TARGET,SLAVE_ADDRESS_1,target);//target set
 //write_wms(MOTOR_SET_SPEED,SLAVE_ADDRESS_1,speed);//command speed
 //write_wms(MOTOR_SET_TARGET_SPEED,SLAVE_ADDRESS_1,target);//target _speed // I´M SUPPOSED TO PUT TARGET OR SPEED HERE? 
 
 while ( (pos=read_wms( MOTOR_GET_COUNT ,SLAVE_ADDRESS_1))!=target) // WHY GET_COUNT AND NOT GET_TARGET?
 //while ( (pos=read_wms( MOTOR_GET_TARGET ,SLAVE_ADDRESS_1))!=target) 
 {
  delay (100);
  Serial.println(pos);}
  // PRINT -1 / -2 / -3 - / -4 /-2 / -1 ...THEN NOTHING...

//motor will turn an stop until reach 1000 counter value

//now we command back 0 pos
  target= 0;//set desired position 1000
 // **OLD speed=-3000;//20,8 us * 300 =0.624s  ->-16,02 microsteps/s 
 
 speed=1000; //DO WE NEED TO PUT NEGATIVE FOR "NEGATIVE" TARGETS ?
 
 Serial.printf("Pos: %i / Target: %i\n",pos,target);
 
 //int32_t resolution=2; //motor will no run on 16 msteps 
 //write_wms(MOTOR_SET_DIR_RES,SLAVE_ADDRESS_1,resolution); //WHAT IS THE BASIC RESOLUTION?
 
 write_wms(MOTOR_SET_TARGET,SLAVE_ADDRESS_1,target);//target set
 write_wms(MOTOR_SET_SPEED,SLAVE_ADDRESS_1,speed);//command speed
 while ( (pos=read_wms( MOTOR_GET_COUNT ,SLAVE_ADDRESS_1))!=target)
 {
  delay (100);
  Serial.println(pos);}
 
}

//********************************
//******** WIRE COMMANDS *********

int32_t read_wms( uint8_t command, int address)
{ int32_t pos;
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
}


void write_wms( uint8_t command, int address,int32_t value)
{ 
  char *pointer;
  pointer=(char*)(&value);
 Wire.beginTransmission(address); 
  Wire.write( command);
  Wire.write(pointer,4);
  Wire.endTransmission();    

}
