
 /*
  * Written on 30/03/2021
  * SCMD library by SparkFun used (https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library)
  * Rest of libraries from Arduino
  * STM32 package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
  * The default ports of I2C com are PB6 and PB7, PC13 for LED
  * 
  * ---------- 
  * 
  * It accelerates/decelerates same ratio, waiting is same time, works fine
  * Change in future the waiting, rearrange variables for imputing only speed / times of ramp and wait
  */
  
#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h"                          //Contains #defines for common SCMD register names and values

#define LEDPIN PC13                               //STM32 LED pin

SCMD DriverOne;                                   //Driver object

int counter=0;                                    //Counter for speed
bool laststate=true;                              //Current/Latest state of motor (false=decceleration, true=acceleration)
bool waiting=false;                               //Check if the motor is at a const speed

int maxspeed=255;                                 //Max velocity to reach
int delayramp=100;                                //Delay for increase in speed, in milisec
int delaywait=5000;                               //Delay on waiting

void counter_set(){
  if(waiting==false){   //State of accel or decel
    if(counter<=maxspeed && laststate==true){               //Acceleration
      counter=counter+1;
      if(counter>maxspeed){                                 //Max speed reach
        waiting=true;
        counter=maxspeed;
      }
    }else if(counter>=0 && laststate==false){               //Deceleration
      counter=counter-1;
      if(counter<0){                                        //0 speed reach
        waiting=true;
        counter=0;
      }
    }  
  }else{
    waiting=false;
    if(laststate){                                          //Start of decel
      laststate=false;
    }else{
      laststate=true;                                       //Start of accel
    }
  }

    DriverOne.setDrive(0,0,counter);   
}

void setup() {
  
  counter=0;                                      //Reset of counter
  pinMode(LEDPIN, OUTPUT);                        //LED definition
  
    Timer1.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
    Timer1.setPeriod(200000); // in microseconds
    Timer1.setCompare(TIMER_CH1, 1);      // overflow might be small

   
  Serial.begin(9600);                             //Serial Monitor initiation
  Serial.println("Start");
    
  DriverOne.settings.commInterface = I2C_MODE;    //Driver mode definition
  DriverOne.settings.I2CAddress = 0x5D;           //Driver adress definition (0x5D by default)

  while(DriverOne.begin() != 0xA9){               //Wait for idle   
    Serial.println("ID Mismatch");
    delay(200);
  }
  Serial.println("ID Match");

  Serial.println("Waiting for enumeration");      //Wait for peripherals (enumeration)
  while(DriverOne.ready() == false);
  Serial.println("Ready");

  while(DriverOne.busy());                        //Enables the driver
  DriverOne.enable();

    Timer1.attachInterrupt(TIMER_CH1, counter_set);
}

void loop() {

//------------Test 8: millis

                      
  Serial.print("State: ");
  if(waiting==false){
    if(laststate==true){
      Serial.print("ACCELERATION");
    }else{
      Serial.print("DECELERATION");
    }
  }else{
    Serial.print("WAITING");
  }
  Serial.print(", Speed: ");
  Serial.println(counter);
  
}
