/* Written on 30/05/2021
 * STM32 package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
 * SCMD library by SparkFun used (https://github.com/sparkfun/SparkFun_Serial1_Controlled_Motor_Driver_Arduino_Library)
 * MPU9250 library by Rafa Castalla used (https://github.com/rafacastalla/MPU9250-1)
 * read_IMU() and IMU setup content extracted from the code "IMU_SPI.3_Pitch-Roll" by Andrés Gómez and Miquel Reurer, from the PLATHON group (magnetorquers section)
 * IMU code might be needed to change further
 */

//██████████████████████████████████████████████████████████████████████ DEFINITIONS
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ LIBRARIES
#include <Arduino.h>
#include <stdint.h>
#include <SCMD.h>   //Driver library
#include <SCMD_config.h>
#include <SPI.h>   //SPI library
#include <MPU9250.h>   //IMU library

//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ DEFINITIONS
#define LEDPIN PC13    //Integrated LED definition
#define CS1 PA4   //STM32 NCS pin
//MOSI on PA7, MISO on PA6 and CLK on PA5 by default on STM32

#define STM32_CLOCK 72000   //72MHz clock works (in KHz so period is milisec)
#define rad_to_deg 57.29577951    //Conversions
#define deg_to_rad 0.01745329 
#define Degree_Total_Tolerancy 1   //+-1 degree of tolerancy (acceptable error on pointing)
#define Degree_PointingMode_Tolerancy 5   //+-5 degree of tolerancy (acceptable value to begin fine pointing)
#define Accel_Tolerancy 0.5   //+-0.5 unit of acceleration of tolerancy (aceptable error of acceleration)
#define Gyro_Tolerancy 1 //+-1 unit of gyroscope Z tolerancy. Accounts to know if it is rotating or not at a cte speed for the ramp
#define PI 3.14159265

SCMD DriverOne;    //Driver Object definition
MPU9250 IMU(SPI,4);   //MPU object

//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GLOBAL VARIABLES
float IMU_gyro_data_X, IMU_gyro_data_Y, IMU_gyro_data_Z;    //Values of local IMU
float IMU_accel_data_X, IMU_accel_data_Y, IMU_accel_data_Z;
float IMU_mag_data_X, IMU_mag_data_Y, IMU_mag_data_Z;
float IMU_deg_data_X, IMU_deg_data_Y, IMU_deg_data_Z=0;
bool first_read=true;

//Outside IMU, as for some reason initiates variables to 0 without defining them
float Gpitch , Groll , Gyaw;
float Atotal , Apitch , Aroll , Ayaw;
float Mpitch , Mroll;
float M_x_eq , M_y_eq ;
float Mag_x_damp , Mag_y_damp ;
float Heading;
float G_pitch_output, G_roll_output , G_yaw_output ;
bool Gyro_sync;
float prev_IMU_accel_data_X=0, prev_IMU_accel_data_Y=0;
float offset=0, gyro_Z_offseted;



int RW_speed=0;   //Value of Reaction Wheel Speed
int OBC_data_value=0;
int OBC_mode_value=0;    //values to get from OBC

//PID
float degree_to_reach=0;
bool zero_state=false;
double PID_output=0;  //Value for PD

//██████████████████████████████████████████████████████████████████████ FUNCTIONS
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GENERAL USE FUNCTIONS
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ OBC Functions
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Get mode from OBC. Uses Timer CH1. Serial1 as a substitution
void OBC_mode_receive(){            
  if (Serial1.available() > 0) {
    String bufferString = "";   //String for buffer of Serial1
    while (Serial1.available() > 0) {
      bufferString += (char)Serial1.read();  //Adds chars to the Serial1 buffer
    }
    OBC_mode_value = bufferString.toInt();   //Conversion from String to int
    Serial1.print("Mode Number: ");
    Serial1.println(OBC_mode_value);
  
  }
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Get data from OBC. Uses Timer CH1. Serial1 as a substitution
void OBC_data_receive(){            
  if (Serial1.available() > 0) {
    String bufferString = "";   //String for buffer of Serial1
    while (Serial1.available() > 0) {
      bufferString += (char)Serial1.read();  //adds chars to the Serial1 buffer
    }
    OBC_data_value = bufferString.toInt();   //Conversion from String to int
    Serial1.print("Value to turn: ");
    Serial1.println(OBC_data_value);
    
  }
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Driver Functions
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to set the motor at a fixed direction and speed
void get_impulse(bool rw_direction, int new_rw_speed){  //acount for change in direction
  if(rw_direction){
    DriverOne.setDrive(0,0,new_rw_speed);  //Change direction depending on motor conection
    RW_speed=new_rw_speed;
  }else{
    DriverOne.setDrive(0,1,new_rw_speed);
    RW_speed=-new_rw_speed;    
  }
  
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to create ramps
void ramp_definition(bool rw_direction, int Acc_ramp_time_reach, int rw_ramp_speed_reach, bool state){ 
  //Single Impulse, we need to put the motor at full speed, get the time before starts and the time when it reaches max speed
  //As we dont know the time it lasts, we have to see if speed changes on the cubesat. That is the use of the while.
  Serial1.println("Ramp Start");
  int tolerancia=0;
  bool waiting=true;
 
  get_impulse(rw_direction, rw_ramp_speed_reach);

  Timer1.attachInterrupt(TIMER_CH4, read_show_IMU);
  waiting=true;
  
  while(waiting){ //Range of tolerancy
    if(state){  //if accelerating
      if(abs(gyro_Z_offseted)>Gyro_Tolerancy){ //if gyro is not 0 or so, means it has cte speed of rotation
        if(abs(IMU_accel_data_X)<Accel_Tolerancy){ //Stop of acceleration
          waiting=false;
        }
      }
    }else{  //if decelerating
      if(abs(gyro_Z_offseted)<Gyro_Tolerancy){ //if gyro is not 0 or so, means it has cte speed of rotation
        if(abs(IMU_accel_data_X)<Accel_Tolerancy){ //Stop of acceleration
          waiting=false;
        }
      }
    }
    delay(1);  //Change
  }
  waiting=true;
  Timer1.detachInterrupt(TIMER_CH4);

}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Functions
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to read IMU_Data (and refreshes the global variables for IMU data)
void read_IMU(){

  float alpha=0.95;
  IMU.readSensor();

  IMU_accel_data_X=IMU.getAccelX_mss();
  IMU_accel_data_Y=IMU.getAccelY_mss(); 
  IMU_accel_data_Z=IMU.getAccelZ_mss();
  IMU_gyro_data_X=(IMU.getGyroX_rads()*rad_to_deg);
  IMU_gyro_data_Y=(IMU.getGyroY_rads()*rad_to_deg);
  IMU_gyro_data_Z=(IMU.getGyroZ_rads()*rad_to_deg);
  IMU_mag_data_X=IMU.getMagX_uT();
  IMU_mag_data_Y=IMU.getMagY_uT();
  IMU_mag_data_Z=IMU.getMagZ_uT();

   //GetTime of IRS, in case value is correct if timer definition changes
  //First division of clock by preescaler (frequency of timer), then inversion (period of timer), then multiplication by overflow (period of channel) 
  float dT = ((1/((float)STM32_CLOCK/(float)Timer1.getPrescaleFactor()))*Timer1.getOverflow())*0.001;  //in seconds

  Gpitch += dT*(IMU.getGyroY_rads())*rad_to_deg;
  Groll += dT*(IMU.getGyroX_rads())*rad_to_deg;  

  
  Gpitch = Gpitch + Groll*sin(dT*(IMU.getGyroZ_rads()));   //gimbal lock compensation
  Groll = Groll - Gpitch*sin(dT*(IMU.getGyroZ_rads()));

  Atotal = sqrt((IMU.getAccelX_mss() * IMU.getAccelX_mss()) + (IMU.getAccelY_mss() * IMU.getAccelY_mss()) + (IMU.getAccelZ_mss() * IMU.getAccelZ_mss()));   // Calculate the total (3D) vector
  Apitch = asin((float)IMU.getAccelX_mss() / Atotal) * rad_to_deg;                         //Calculate the pitch angle
  Aroll = asin((float)IMU.getAccelY_mss() / Atotal) * rad_to_deg;                         //Calculate the roll angle

  Apitch -= 2.4;
  Aroll -= -0.9;
  
  if (Gyro_sync)
  {
    // ----- Gyro & accel have been synchronised
    Gpitch = Gpitch * 0.95 + Apitch * 0.05;        //Correct the drift of the gyro pitch angle with the accelerometer pitch angle //(ANDRES) Els valors originals eren 0.9996 i 0.0004
    Groll = Groll * 0.95 + Aroll * 0.05;           //Correct the drift of the gyro roll angle with the accelerometer roll angle   // Pero si ho proveu veureu que tal com està ara s'ajusta tot més ràpid

  }else{                                                                                                                               // El 0.98 i 0.02 surten de http://www.pieter-jan.com/node/11else{
    // ----- Synchronise gyro & accel
    Gpitch = Apitch;                                       //Set the gyro pitch angle equal to the accelerometer pitch angle
    Groll = Aroll;                                         //Set the gyro roll angle equal to the accelerometer roll angle
  
    Gyro_sync = true;                                      //Set the IMU started flag
  }

  G_pitch_output = G_pitch_output * 0.9 + Gpitch * 0.1;    
  G_roll_output = G_roll_output * 0.9 + Groll * 0.1;  

  //ONLY ACCOUNTS GYROSCOPE, NO CORRECTION BUT ATTEMPS TO CORRECT WITH MAG 
  //It seems mag does not work properly, as values in each orientation do not correlate even with calibration from library
  //However, acc in x and y can be used to tell the adcs if the cubesat is or is not rotating, so we can correct the gyro drift.
  //it does not work well at all, but is a useful correction aproximation at least.
  //Acc deviation is within 0.04 units or so. Must be considered yaw is only correct if pitch and roll is 0.
  //In theory, experimentation is in a 2d plane so there should not be pitch or roll

  
  float accel_dev=0.04;
  float accel_dev_t=abs((Apitch-prev_IMU_accel_data_X))+abs(Aroll-prev_IMU_accel_data_Y);
  
  if(accel_dev_t<accel_dev && IMU_gyro_data_Z<Gyro_Tolerancy){  //we consider it is stopped, modify values to be acurate
    offset=IMU_gyro_data_Z;
  }else{
    offset=0;
  }
  
  Gyaw = Gyaw + (IMU_gyro_data_Z-offset)*dT; 
  gyro_Z_offseted=IMU_gyro_data_Z-offset;
  
  //Ayaw = atan2(-IMU_accel_data_X,sqrt(IMU_accel_data_Y*IMU_accel_data_Y+IMU_accel_data_Z*IMU_accel_data_Z))*rad_to_deg;
  
  //IMU_deg_data_Z = Gyaw * alpha + Ayaw * (1-alpha);
  IMU_deg_data_Z = Gyaw * alpha + Apitch * (1-alpha);

  if (IMU_deg_data_Z < 0) IMU_deg_data_Z += 360;
  if (IMU_deg_data_Z >= 360) IMU_deg_data_Z -= 360;
  
  IMU_accel_data_X=Apitch;
  IMU_accel_data_Y=Aroll;
  IMU_deg_data_X=G_pitch_output;
  IMU_deg_data_Y=G_roll_output;
  
  prev_IMU_accel_data_X=Apitch;
  prev_IMU_accel_data_Y=Aroll;
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to show IMU_Data on the Serial1 Monitor
void show_IMU(){

  Serial1.print("A: "); 
  Serial1.print(IMU_accel_data_X,4);   Serial1.print('\t');
  Serial1.print(IMU_accel_data_Y,4);   Serial1.print('\t');
  //Serial1.print(IMU_accel_data_Z,4); 
  //      Serial1.println("");
  //Serial1.print(" / G: ");  
  //Serial1.print(IMU_gyro_data_X,4);  Serial1.print('\t');
  //Serial1.print(IMU_gyro_data_Y,4);  Serial1.print('\t');
  //Serial1.print(IMU_gyro_data_Z,4);   Serial1.print('\t');
  //Serial1.print(gyro_Z_offseted,4);
  //     Serial1.println("");
  //Serial1.print("MAGS: "); 
  //Serial1.print(IMU_mag_data_X,4);    Serial1.print('\t');
  //Serial1.print(IMU_mag_data_Y,4);    Serial1.println('\t');
  //Serial1.print(IMU_mag_data_Z,4);
  //      Serial1.println("");
  //Serial1.print(" / D: "); 
  //Serial1.print(IMU_deg_data_X,4);    
  //Serial1.print(IMU_deg_data_Y,4);    
  //Serial1.print(IMU_deg_data_Z,4); 
       Serial1.println("");
  
 
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to both read and show IMU_Data
void read_show_IMU(){
  read_IMU();
  show_IMU();
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ PID Functions
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to calculate PID
void computePID(){

double error, lastError;                 // Initialize error and previousError
double input, output, setPoint;          // Initialize input variable from IMU, the output variable, and the desired setPoint
double cumError, rateError;              // Initialize the cumulative Error (Integral) and the rate of Error (Derivative)

double kp = 3; // Proportional contribution
double ki = 4; // Integral contribution
double kd = 0; // Derivative contribution

  float dT = ((1/((float)STM32_CLOCK/(float)Timer1.getPrescaleFactor()))*Timer1.getOverflow())*0.001;  //in seconds

  read_IMU();

  if(zero_state){    //saves if the pid pass through 0, if it does it moves the zone away from 0
    IMU_deg_data_Z+180;   //Addition of 180 deg to all values. the error is a substract, so the difference is the same
    degree_to_reach+180;
  }

if (IMU_deg_data_Z >= 360) IMU_deg_data_Z -= 360;   //one of them will increase over 360, it is a correction
if (degree_to_reach >= 360) degree_to_reach -= 360;
  
  // Percentage
  volatile float IMU_deg_data_Z_perc = (IMU_deg_data_Z - 360)/(-360)*100;

  // Transform angle to percentage
  volatile float degree_to_reach_perc = (degree_to_reach - 360)/(-360)*100;


  
  // Errors
  error = degree_to_reach_perc - IMU_deg_data_Z_perc;       // Calculate error (Proportional)
  cumError += error * dT;               // Calculate the cumulative error (Integral)
  rateError = (error - lastError) / dT; // Calculate the rate of error (Derivative)

  // PID Control
  float PID_P = kp * error;     // Proportional
  float PID_I = ki * cumError;  // Integral
  float PID_D = kd * rateError; // Derivative

  if (PID_P > 255) PID_P = 255;
  if (PID_P < -255) PID_P = -255;
  if (PID_I > 255) PID_I = 255;
  if (PID_I < -255) PID_I = -255;
  if (PID_D > 255) PID_D = 255;
  if (PID_D < -255) PID_D = -255;

  float PID_output = PID_P + PID_I + PID_D; // PID control

  if (PID_output > 255) PID_output = 255;
  if (PID_output < -255) PID_output = -255;

  bool rw_direction=true;   //true=positive (CCW), false=negative (CW)
  
  if(PID_output<0){     
    PID_output=-PID_output;
    rw_direction=false;
  }
  
  get_impulse(rw_direction, PID_output);
  // Save current error and time for next iteration
  lastError = error;          // Save current error
}













//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ MODES OF OPERATION
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Mode Selection
void mode_Select(int mode_value){
  switch(mode_value){
    default:   //----------------------Mode OBC Imput Waiting (0): Waiting for OBC
    Serial1.println("Reading mode from OBC");
    mode_OBC_Imput_Wait();
    break;
    case 1:   //----------------------Mode Positioning RW only
    Serial1.println("Mode Positioning");
    mode_Positioning_RW();
    break;
    case 2:   //----------------------Mode IMU reading (NO USE)
    Serial1.println("Reading IMU");
    mode_IMU_reading();
    break;
  }
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ MODE IMU ONLY FOR TESTING
void mode_IMU_reading(){ 
  OBC_mode_value=0;
  Timer1.attachInterrupt(TIMER_CH4, read_show_IMU);
  //read_show_IMU();
  //mode_Select(OBC_mode_value);
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 0. Mode Default, OBC Reading
void mode_OBC_Imput_Wait(){ 
  OBC_mode_value=0;
  Timer1.attachInterrupt(TIMER_CH3, OBC_mode_receive);
  while(OBC_mode_value==0){ 
    delay(1); //If not used the while function does not work
    //it can be added more conditions to evade being blocked until a data is received.
    //for example, it could function an interrupt with a forced exit and an if after or something
  }
  Timer1.detachInterrupt(TIMER_CH3);
  Serial1.println(OBC_mode_value);
  mode_Select(OBC_mode_value);
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 1. Mode Positioning (RW only)
void mode_Positioning_RW(){
  OBC_mode_value=0;
  Serial1.println ("Positioning begin");
//  Insert code to select if the positioning will be coarse (impulses) or fine (PD)
/* It should be something like:
 * read IMU degree value
 * get OBC value from OBC and save in global variable
 * comparison to OBC value (wanted position), add to the void function definition
 * if (comparison<acceptable_error){
 * positioning_fine();
 * }else{
 * positioning_coarse();
 * }
 */
 read_IMU();
 Serial1.print("Axis Z position: ");
 Serial1.println(IMU_deg_data_Z);
 Serial1.println("Insert degree value to turn");
 OBC_data_value=0;
 Timer1.attachInterrupt(TIMER_CH3, OBC_data_receive);
 while(OBC_data_value==0){ 
   delay(1); //If not used the while function does not work
 }
 Timer1.detachInterrupt(TIMER_CH3);

 if(abs(OBC_data_value)<=Degree_PointingMode_Tolerancy){
   positioning_Fine();
 }else{
   positioning_Coarse();
 }
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ 1.1. Mode Positioning Coarse
void positioning_Coarse(){     //For now simple
  Serial1.println("Mode Positioning Coarse");

  float degree_turn_value=OBC_data_value;   //Stores a new variable to not overwrite the original value
  bool rw_direction=true;   //true=positive (CCW), false=negative (CW)
//  In case OBC_data_value is positive, value remains the same and rw direction is true, the default value of the variable
//  In case OBC_data_value is negative, value change to positive and rw direction is false

  if(degree_turn_value<0){     
    degree_turn_value=-degree_turn_value;
    rw_direction=false;
  }

  float Initial_IMU_degree_value, Final_IMU_degree_value, Delta_degree_ramp, Degree_stop_wait;
  float Prev_IMU_deg_data_Z, Check_IMU_deg_data_Z;
  int overlap_count=0; 
  bool waiting=true;

//----------------------------------------------------------ACCELERATION

  read_show_IMU();
  Initial_IMU_degree_value=IMU_deg_data_Z;    //Stores initial degree, only in Z
  int initial_RW_speed=RW_speed;
  ramp_definition(rw_direction, 0, 255, 1);  
//0 will not be used as we dont define the time the motor lasts to do an impulse. 255 is the max value
//This values will be sustituted by how we want the time and speed to reach in the ramp.
  read_IMU();
  Final_IMU_degree_value=IMU_deg_data_Z;    //Stores final degree, only in Z
  if(rw_direction){
     Delta_degree_ramp=Final_IMU_degree_value-Initial_IMU_degree_value;    //degree turnt on acc, only in Z. Shoud be positive
  }else{
    Delta_degree_ramp=Initial_IMU_degree_value-Final_IMU_degree_value;    //degree turnt on acc, only in Z. Shoud be positive
  }
  
  if(Delta_degree_ramp<0){
    Delta_degree_ramp += 360; //In case its negative adds 360
    //Negative cases: changes goes by 0º. EX: 330 to 30 when CCW (should be 60 but calculus is 30-330= -300)
    //                                    EX: 30 to 330 when CW  (should be 60 but calculus is 30-330= -300)
  }
    Serial1.print("ID: ");
    Serial1.print(Initial_IMU_degree_value); Serial1.print(" / T: ");
    Serial1.print(Delta_degree_ramp); Serial1.print(" / ED: ");
    Serial1.println(Final_IMU_degree_value);
//--------------------------------------------------------------WAITING
  waiting=true;
  if(rw_direction){ //CASE CCW
    if((degree_turn_value - 2*Delta_degree_ramp)>0){ //if its <0, it must be done inmediatly after, and still it would be too much turn.
      Degree_stop_wait = Final_IMU_degree_value + (degree_turn_value - 2*Delta_degree_ramp);  //Get value of degree to start
    
      Timer1.attachInterrupt(TIMER_CH4, read_IMU);
      Serial1.println("Waiting");
      while(waiting){   //Stays as long as waiting is true.
        
        
        //CAUTION WITH READING VALUES; AS IT IS ALWAYS FROM 0 TO 360
        // Degree_stop_wait will always be > IMU_deg_data_Z. If IMU_deg_data_Z>> (ex: 359º),  Degree_stop_wait can be >360. Thus the value of If IMU_deg_data_Z would never reach Degree_stop_wait
        // Degree_stop_wait cant be decreased, as the way to check if its reached is by a greater. If it is decreased by 360º (so it stays in relative place), it could be < IMU_deg_data_Z and would inmediatly exit the while without waiting.
        // the way to do is check if there is a heavy change on IMU_deg_data_Z (pass on 0) to check iff adding or substracting a lap, making it go out of the 0 and 360 range.
        //It should not be a high acceleration enough to make a jump of degree of 180º in such short time,so it sholud be fine.
        //Caution disconnection from IMU, could give a false jump, that is why a check on IMU_deg_data_Z first.
        
        if(IMU_deg_data_Z<0.0001 && IMU_deg_data_Z>-0.0001){  //if it is almost exactly 0 then most probably there is a disconnection. Close numbers should not trigger it.  
        }else if(IMU_deg_data_Z-Prev_IMU_deg_data_Z<-180){ //Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 359 to 0.
          overlap_count += 1; //adds a lap
        }else if(IMU_deg_data_Z-Prev_IMU_deg_data_Z>+180){ //Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 0 to 359. (Not possible in theory, as it increases, but deviations could mess it up)
          overlap_count -= 1; //sustracts a lap
        } //if niether are triggered, change has been samall or no change has been done yet
        Prev_IMU_deg_data_Z=IMU_deg_data_Z;
        Check_IMU_deg_data_Z=IMU_deg_data_Z+(360*overlap_count);  //Rewriting of value, in new variable so it does not add itself.
        if(Check_IMU_deg_data_Z>Degree_stop_wait){  //As it is CCW, degree increases. When reading is > to stop value, exits the while
          waiting=false;  //exit condition
        }
        delay(1); //To solve errors
        //No writing of read_IMU as it is already done by a timer. Just to remind IMU_deg_data_Z is constantly reading values.
      }
      Timer1.detachInterrupt(TIMER_CH4);
    }
  }else{  //CASE CW
    if((degree_turn_value - 2*Delta_degree_ramp)>0){ //if its <0, it must be done inmediatly after, and still it would be too much turn.
      Degree_stop_wait = Final_IMU_degree_value - (degree_turn_value - 2*Delta_degree_ramp);  //Get value of degree to start
    
      Timer1.attachInterrupt(TIMER_CH4, read_IMU);
      while(waiting){   //Stays as long as waitin is true.
        Serial1.println("Waiting");
        
        //CAUTION WITH READING VALUES; AS IT IS ALWAYS FROM 0 TO 360
        // Degree_stop_wait will always be < IMU_deg_data_Z. If IMU_deg_data_Z<< (ex: 1º),  Degree_stop_wait can be <0. Thus the value of If IMU_deg_data_Z would never reach Degree_stop_wait
        // Degree_stop_wait cant be increased, as the way to check if its reached is by a greater. If it is increased by 360º (so it stays in relative place), it could be > IMU_deg_data_Z and would inmediatly exit the while without waiting.
        // the way to do is check if there is a heavy change on IMU_deg_data_Z (pass on 0) to check iff adding or substracting a lap, making it go out of the 0 and 360 range.
        //It should not be a high acceleration enough to make a jump of degree of 180º in such short time, so it sholud be fine.
        //Caution disconnection from IMU, could give a false jump, that is why a check on IMU_deg_data_Z first.
        
        Check_IMU_deg_data_Z=IMU_deg_data_Z;  //New variable so it does not change during an iteration
        if(Check_IMU_deg_data_Z<0.0001 && Check_IMU_deg_data_Z>-0.0001){  //if it is almost exactly 0 then most probably there is a disconnection. Close numbers should not trigger it.  
        }else if(Check_IMU_deg_data_Z-Prev_IMU_deg_data_Z>+180){ //Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 359 to 0. (Not possible in theory, as it decreases, but deviations could mess it up)
          overlap_count += 1; //adds a lap
        }else if(Check_IMU_deg_data_Z-Prev_IMU_deg_data_Z<-180){ //Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 0 to 359.
          overlap_count -= 1; //sustracts a lap
        } //if niether are triggered, change has been samall or no change has been done yet
        
        Prev_IMU_deg_data_Z=Check_IMU_deg_data_Z;
        Check_IMU_deg_data_Z=Check_IMU_deg_data_Z+(360*overlap_count);  //Rewriting of value. It does not add itself as it gets the read value in each iteration. In other words, it will not go throug the iteration with numbers outside 0 and 360 range
        if(Check_IMU_deg_data_Z<Degree_stop_wait){  //As it is CCW, degree increases. When reading is > to stop value, exits the while
          waiting=false;  //exit condition
        }
        delay(1); //To solve errors
        //No writing of read_IMU as it is already done by a timer. Just to remind IMU_deg_data_Z is constantly reading values.
      }
      Timer1.detachInterrupt(TIMER_CH4);
    }
  }

//----------------------------------------------------------DECELERATION
    Serial1.print("ID: ");
    Serial1.print(Final_IMU_degree_value); Serial1.print(" / ED: ");
    Serial1.println(Degree_stop_wait);

   
   ramp_definition(rw_direction, initial_RW_speed, 0, 0); //Initial_RW_Speed returns speed to original value instead of 0
   //This is to assure the impulse is the same, as if there is some momentum accumulation, returning to 0 would be a different impulse
//0 will not be used as we dont define the time the motor lasts to do an impulse. 255 is the max value
//This values will be sustituted by how we want the time and speed to reach in the ramp.
  read_IMU();
  Final_IMU_degree_value=IMU_deg_data_Z; //can be overwritten, final degree of manouver
    if(rw_direction){
     Delta_degree_ramp=Final_IMU_degree_value-Initial_IMU_degree_value;    //Shoud be positive
  }else{
    Delta_degree_ramp=Initial_IMU_degree_value-Final_IMU_degree_value;    //Shoud be positive
  }
  if(Delta_degree_ramp<0){  //can be overwritten, real turn of manouver
    Delta_degree_ramp += 360; //In case its negative adds 360
    //Negative cases: changes goes by 0º. EX: 330 to 30 when CCW (should be 60 but calculus is 30-330= -300)
    //                                    EX: 30 to 330 when CW  (should be 60 but calculus is 30-330= -300)
  }
  
    Serial1.print("ID: ");
    Serial1.print(Degree_stop_wait); Serial1.print(" / T: ");
    Serial1.print(Delta_degree_ramp); Serial1.print(" / ED: ");
    Serial1.println(Final_IMU_degree_value);
    
  if((Delta_degree_ramp-degree_turn_value)<Degree_Total_Tolerancy && (Delta_degree_ramp-degree_turn_value)>Degree_Total_Tolerancy){  //Real turn vs wanted turn, checks if it is considered good
     mode_Select(OBC_mode_value); //Valid position
  }else{
    positioning_Fine(); //Correction

  }
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ 1.2. Mode Positioning Coarse
void positioning_Fine(){     //For now nothing 
Serial1.println("Mode Positioning Fine");
  
bool waiting=true;

degree_to_reach = OBC_data_value + IMU_deg_data_Z;  //Get value to reach, contained in 360
if (degree_to_reach < 0){
  degree_to_reach += 360;
  zero_state=true;    //Used to store if it passes 0.
  //A PD passing through 0 could give great problems, as it has a very big change in value. Further used in computePID()
}else if (degree_to_reach >= 360){
  degree_to_reach -= 360; //Estas dos lineas contienen el valor del heading en 0-360 grados.
  zero_state=true;
}else{
  zero_state=false;
}

Timer1.attachInterrupt(TIMER_CH4, computePID);
while(waiting){ //Range of tolerancy
  if(abs(gyro_Z_offseted)>Gyro_Tolerancy && abs(IMU_accel_data_X)<Accel_Tolerancy){  //we consider it is stopped, modify values to be acurate
    waiting=false;
  }
    delay(1);  //Change
  }
  waiting=true;
Timer1.detachInterrupt(TIMER_CH4);

//At exit, RW_speed could not be 0
OBC_data_value=0;
Serial1.println("End of Manouver");
mode_Select(OBC_mode_value);
}

//██████████████████████████████████████████████████████████████████████ VOID SETUP
void setup() {
  delay(1000); // wait to let open the serial
  //▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Timers
  Timer1.pause();
  Timer1.setPrescaleFactor(7200);   //72MHz Clock / 7200 = 10KHz timer
  Timer1.setOverflow(1000);    //Overflow occurs at 1000, each 100 ms timer restarts
    
  Timer1.setMode(TIMER_CH3, TIMER_OUTPUT_COMPARE);    //Configure channel to OUTPURCOMPARE: Channel for OBC read values
  Timer1.setMode(TIMER_CH4, TIMER_OUTPUT_COMPARE);    //Channel for IMU read values
  Timer1.setCompare(TIMER_CH3, 1);    //Phase value in Overflow range
  Timer1.setCompare(TIMER_CH4, 1);
  
  Timer1.refresh();   //Refresh timer and start over
  Timer1.resume();
  
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Initiations
  Serial1.begin(9600);
  SPI.begin();

  Serial1.println("START");
  
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Setups

  pinMode(LEDPIN,OUTPUT);   //Integrated LED 
  pinMode(CS1, OUTPUT);   //NCS Pin definition

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Driver Setup
  Serial1.println("Configuring Driver...");
  DriverOne.settings.commInterface = I2C_MODE;    //Driver Comm Mode
  DriverOne.settings.I2CAddress = 0x5D;    //Driver Adress (0x5D by Defalut)

  while(DriverOne.begin()!=0xA9){   //Driver wait for idle
    Serial1.println("ID Mismatch");
    delay(200);
  }
  Serial1.println("ID Match");

  Serial1.println("Waiting for enumeration");    //Driver wait for peripherals
  while(DriverOne.ready()==false);
  Serial1.println("Ready");

  while(DriverOne.busy());      //Driver enable
  DriverOne.enable();
  Serial1.println("Driver Ready to Use!");

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Setup
  Serial1.println("Configuring MPU9250...");
  int  status = IMU.begin();
  if (status < 0) {
    Serial1.println("IMU initialization unsuccessful");
    Serial1.println("Check IMU wiring or try cycling power");
    Serial1.print("Status: ");
    Serial1.println(status);   
    while(1){}
  }

  IMU.setGyroRange(IMU.GYRO_RANGE_500DPS);

  

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Calibration
   //Use for getting the values on calibration setting below
  
  IMU.calibrateAccel();
  IMU.setAccelCalX(IMU.getAccelBiasX_mss(),IMU.getAccelScaleFactorX()); //Primer valor el MagBias, y el segundo el ScaleFactor!
  IMU.setAccelCalY(IMU.getAccelBiasY_mss(),IMU.getAccelScaleFactorY());
  IMU.setAccelCalZ(IMU.getAccelBiasZ_mss(),IMU.getAccelScaleFactorZ());
  
  IMU.calibrateGyro();
  IMU.setGyroBiasX_rads(IMU.getGyroBiasX_rads());
  IMU.setGyroBiasY_rads(IMU.getGyroBiasY_rads());
  IMU.setGyroBiasZ_rads(IMU.getGyroBiasZ_rads());
  
  IMU.calibrateMag(); 
  IMU.setMagCalX(IMU.getMagBiasX_uT(),IMU.getMagScaleFactorX()); //Primer valor el MagBias, y el segundo el ScaleFactor!
  IMU.setMagCalY(IMU.getMagBiasY_uT(),IMU.getMagScaleFactorY());
  IMU.setMagCalZ(IMU.getMagBiasZ_uT(),IMU.getMagScaleFactorZ());

  Serial1.println("MPU9250 Ready to Use!");

  Serial1.println("Reading mode from OBC");
  mode_OBC_Imput_Wait();
  
}

//██████████████████████████████████████████████████████████████████████ VOID LOOP
void loop() {

}
