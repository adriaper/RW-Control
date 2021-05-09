/* Written on 04/05/2021
 * STM32 package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
 * SCMD library by SparkFun used (https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library)
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

int OBC_value=0;    //Increment of angle, here for now, should be in positioning type select
int IMU_degree_value=0;
int IMU_speed_value=1;
int rw_speed=0;

//------------------------------------Functions

//Function to read OBC_Data
void OBC_mode_receive(){            //Get data from OBC. Uses Timer CH1
   if (Serial.available())
   {
    int mode_value = (char)Serial.read();
    Serial.flush();
    //decoder(mode_value);
    mode_select(mode_value);
   }
  Serial.println("Reading OBC mode");
}

void OBC_data_receive(){            //Get data from OBC. Uses Timer CH1
 if (Serial.available() > 0) {
    //Se crea una variable que servirá como buffer
    String bufferString = "";
    /*
     * Se le indica a Arduino que mientras haya datos
     * disponibles para ser leídos en el puerto serie
     * se mantenga concatenando los caracteres en la
     * variable bufferString
     */
    while (Serial.available() > 0) {
      bufferString += (char)Serial.read();
    }
    //Se transforma el buffer a un número entero
    OBC_value = bufferString.toInt();
    positioning_coarse();
  }
  Serial.println("Reading OBC data");
}

//Function to read IMU_Data (and refreshes the global variables for IMU data)
void read_IMU(){
  //IMU_values=readImufunction; all of them
}

//Function to set the motor at a direction and speed
void get_impulse(bool rw_direction, int new_rw_speed){  //acount for change in direction
    if(rw_direction){
      DriverOne.setDrive(0,0,new_rw_speed);  //Change direction depending on motor conection
    }else{
      DriverOne.setDrive(0,1,new_rw_speed);
    }
}

void ramp_definition(bool rw_direction, int Acc_ramp_time_reach, int rw_ramp_speed_reach){ 
  //Single Impulse, we need to put the motor at full speed, get the time before starts and the time when it reaches max speed
  //As we dont know the time it lasts, we have to see if speed changes on the cubesat. That is the use of the while.
  Serial.println("Ramp Start");
  int Prev_speed=1;
  int Curr_speed=0;
  
  get_impulse(rw_direction, rw_ramp_speed_reach);
  
  while(Prev_speed-Curr_speed!=0){  //Change to a less than with a small value
    Prev_speed=Curr_speed;
    read_IMU();
    Curr_speed=IMU_speed_value;
  }
      
}

//---------------------------------------------------------------------------MODE_SELECT
void mode_select(int mode_value){
  Timer1.detachInterrupt(TIMER_CH1);
  switch(mode_value){
    default:   //----------------------Mode Normal (0): Waiting for OBC
    mode_normal();
    break;
    case '1':   //----------------------Mode Positioning RW only
    Serial.println("Mode Positioning");
    mode_positioning();
    break;
    case '2':   //----------------------Mode Detumbling
    break;
    case '3':   //----------------------Mode Solar Pointing
    break;
  }
}

void mode_normal(){    
  Timer1.attachInterrupt(TIMER_CH1, OBC_mode_receive);
}

//-----------------------------MODE POSITIONING
void mode_positioning(){
  Serial.println("Positioning begin");
  positioning_type_select();
  //mode_normal();
}

void positioning_type_select(){
//  Insert code to select if the positioning will be coarse (impulses) or fine (PD)
/* It should be:
 * read IMU degree value
 * get OBC value from OBC and save in global variable
 * comparison to OBC value (wanted position), add to the void function definition
 * if (comparison<acceptable_error){
 * positioning_fine();
 * }else{
 * positioning_coarse();
 * }
 */
    Serial.println("-----");
    Timer1.attachInterrupt(TIMER_CH2, OBC_data_receive);
}

void positioning_coarse(){     //For now simple. 
  Timer1.detachInterrupt(TIMER_CH2);
  
  bool rw_direction=true;   //true=positive (CCW), false=negative (CW)
   //In case OBC_value is positive, value remains the same and rw direction is true, the default value of the variable
   //In case OBC_value is negative, value change to positive and rw direction is false
  if(OBC_value<0){     
    OBC_value=-OBC_value;
    rw_direction=false;
  }

  int Initial_time, Final_time, ACC_time, DEC_time, Wait_time, Total_time;
  int Wait_time_est, Total_time_est;
  int Initial_IMU_degree_value, Final_IMU_degree_value, Delta_degree_ramp;
  
  read_IMU();
  Initial_IMU_degree_value=IMU_degree_value;  //Stores initial degree
  //Initial_time = Timer1.getTime();  //Stores initial Time

  ramp_definition(rw_direction, 0, 255);  
  //0 will not be used as we dont define the time the motor last to do an impulse. 255 is the max value
  //This values will be sustituted by how we want the time and speed to reach in the ramp.
       Serial.println(OBC_value);
  /* 
  //Final_time = Timer1.getTime();
  ACC_time=Final_time-Initial_time;
  read_IMU();
  Final_IMU_degree_value=IMU_degree_value;
  Delta_degree_ramp=Final_IMU_degree_value-Initial_IMU_degree_value;

  //Estimations, may be useful
  Wait_time_est=(OBC_value-(2*(Final_IMU_degree_value-Initial_IMU_degree_value)))/IMU_speed_value; 
  Total_time_est=Wait_time_est-(2*ACC_time);

  //Initial_time = Timer1.getTime();  //Stores initial Time
  //Timer1.attachInterrupt(TIMER_CH2, read_IMU());
  while((IMU_degree_value-Final_IMU_degree_value)<(OBC_value-(2*Delta_degree_ramp))){
    Serial.println("Waiting");//degrees that turns in wait vs total degrees of the wait
  }
  //Timer1.detachInterrupt(TIMER_CH2);
  //Final_time = Timer1.getTime();
  Wait_time=Final_time-Initial_time;
  
  //Initial_time = Timer1.getTime();  //Stores initial Time
  ramp_definition(rw_direction, 0, 255);  
  //Final_time = Timer1.getTime();
  DEC_time=Final_time-Initial_time;
  Total_time=ACC_time+Wait_time+DEC_time;

  read_IMU();
  if((IMU_degree_value-Initial_IMU_degree_value)!=OBC_value){
    positioning_fine();
  }
  */
}

void positioning_fine(){     //For now simple. 
  //PD controller
  OBC_value=0;
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
  Timer1.pause();

  Timer1.setPrescaleFactor(7200);   //72MHz Clock / 7200 = 10KHz timer
  Timer1.setOverflow(1000);    //Overflow occurs at 1000, each 100 ms timer restarts
  
  Timer1.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);    //Configure channel to OUTPURCOMPARE
  Timer1.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);    //Configure channel to OUTPURCOMPARE
  
  Timer1.setCompare(TIMER_CH1, 1);
  Timer1.setCompare(TIMER_CH2, 1);
  
  Timer1.attachInterrupt(TIMER_CH1, OBC_mode_receive);
  Timer1.refresh();   //Refresh timer and start over
  Timer1.resume();
}

void loop() {

}
