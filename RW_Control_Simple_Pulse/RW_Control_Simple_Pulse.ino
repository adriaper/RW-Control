/*
 * Written on 28/04/2021
 * STM32 package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
 * SCMD library by SparkFun used (https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library)
 * 
 */

//------------------------------------Libraries
#include <Arduino.h>
#include <stdint.h>
#include <SCMD.h>   //Driver library
#include <SCMD_config.h>

//------------------------------------Definitions

#define LEDPIN PC13    //Integrated LED definition

SCMD DriverOne;    //Driver Object definition

//------------------------------------Global variables



//------------------------------------Functions
void OBC_data_receive(){
  if(Serial.available()>0){
    int mode_value = Serial.read();
    mode_select(mode_value);
  }
}

void prueba(){
  Serial.println("Hola mundo");
}

//------------------------------------MODE_SELECT
void mode_select(int mode_value){
  switch(mode_value){
    default:   //----------------------Mode Normal (0): Waiting for OBC
    mode_normal();
    break;
    case 1:   //----------------------Mode Positioning RW only
    mode_positioning();
    break;
    case 2:   //----------------------Mode Detumbling
    break;
    case 3:   //----------------------Mode Solar Pointing
    break;
  }
}

void mode_normal(){    
  Timer1.attachInterrupt(TIMER_CH1, OBC_data_receive);
}



void mode_positioning(){
  Timer1.detachInterrupt(TIMER_CH1);
  Timer1.attachInterrupt(TIMER_CH2, prueba);
}

void setup() {

Serial.begin(9600);
Serial.println("START");

//-------------------------Integrated LED Setup
pinMode(LEDPIN,OUTPUT);

//-------------------------Driver Setup
DriverOne.settings.commInterface = I2C_MODE;    //Driver Comm Mode
DriverOne.settings.I2CAddress = 0x5D;    //Driver Adress (0x5D by Defalut)

while(DriverOne.begin()!=0xA9){   //Driver wait for idle
  Serial.println("ID Mismatch");
  delay(200);
}
Serial.println("ID Match");

Serial.println("Waiting for enumeration");    //Driver wait for peripherals
while(DriverOne.ready()==false);
Serial.println("Ready");

while(DriverOne.busy());      //Driver enable
DriverOne.enable();

//---------------------------IMU

//---------------------------Timers

    Timer1.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
    Timer1.setPeriod(100000); // in microseconds
    Timer1.setCompare(TIMER_CH1, 1);      // overflow might be small

    Timer1.attachInterrupt(TIMER_CH1, OBC_data_receive);
}

void loop() {


  }
