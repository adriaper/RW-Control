/* Written on 12/05/2021
 * STM32 package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
 * SCMD library by SparkFun used (https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library)
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
#define Accel_Tolerancy 1   //+-1 unit of acceleration of tolerancy (aceptable error of acceleration)

SCMD DriverOne;    //Driver Object definition
MPU9250 IMU(SPI,4);   //MPU object

//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GLOBAL VARIABLES
float IMU_gyro_data_X, IMU_gyro_data_Y, IMU_gyro_data_Z;    //Values of local IMU
float IMU_accel_data_X, IMU_accel_data_Y, IMU_accel_data_Z;
float IMU_deg_data_X, IMU_deg_data_Y, IMU_deg_data_Z;

int RW_speed=0;   //Value of Reaction Wheel Speed
int OBC_data_value=0;
int OBC_mode_value=0;    //values to get from OBC

//██████████████████████████████████████████████████████████████████████ FUNCTIONS
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GENERAL USE FUNCTIONS
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ OBC Functions
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Get mode from OBC. Uses Timer CH1. Serial as a substitution
void OBC_mode_receive(){            
  if (Serial.available() > 0) {
    String bufferString = "";   //String for buffer of Serial
    while (Serial.available() > 0) {
      bufferString += (char)Serial.read();  //Adds chars to the Serial buffer
    }
    OBC_mode_value = bufferString.toInt();   //Conversion from String to int
    Serial.print("Mode Number: ");
    Serial.println(OBC_mode_value);
    Serial.flush();
  }
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Get data from OBC. Uses Timer CH1. Serial as a substitution
void OBC_data_receive(){            
  if (Serial.available() > 0) {
    String bufferString = "";   //String for buffer of Serial
    while (Serial.available() > 0) {
      bufferString += (char)Serial.read();  //adds chars to the Serial buffer
    }
    OBC_data_value = bufferString.toInt();   //Conversion from String to int
    Serial.print("Value to turn: ");
    Serial.println(OBC_data_value);
    Serial.flush();
  }
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Driver Functions
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to set the motor at a fixed direction and speed
void get_impulse(bool rw_direction, int new_rw_speed){  //acount for change in direction
  if(rw_direction){
    DriverOne.setDrive(0,0,new_rw_speed);  //Change direction depending on motor conection
  }else{
    DriverOne.setDrive(0,1,new_rw_speed);
  }
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to create ramps
void ramp_definition(bool rw_direction, int Acc_ramp_time_reach, int rw_ramp_speed_reach){ 
  //Single Impulse, we need to put the motor at full speed, get the time before starts and the time when it reaches max speed
  //As we dont know the time it lasts, we have to see if speed changes on the cubesat. That is the use of the while.
  Serial.println("Ramp Start");
  int tolerancia=0;

  get_impulse(rw_direction, rw_ramp_speed_reach);

  Timer1.attachInterrupt(TIMER_CH2, read_IMU);
  while(IMU_accel_data_X>Accel_Tolerancy && IMU_accel_data_X<Accel_Tolerancy){ //Range of tolerancy
    //delay(1);  //Change
  }
  Timer1.detachInterrupt(TIMER_CH2);
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Functions
//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to read IMU_Data (and refreshes the global variables for IMU data)
void read_IMU(){
  float Gpitch = 0, Groll = 0, Gyaw = 0;
  float Atotal = 0, Apitch = 0, Aroll = 0;
  float Mpitch = 0, Mroll = 0;
  float M_x_eq = 0, M_y_eq = 0;
  float Mag_x_damp = 0, Mag_y_damp = 0;
  float Heading = 0;
  float G_pitch_output = 0, G_roll_output = 0;
  bool Gyro_sync = false;
  
  IMU.readSensor();

  IMU_accel_data_X=(IMU.getAccelX_mss());
  IMU_accel_data_Y=IMU.getAccelY_mss(); 
  IMU_accel_data_Z=IMU.getAccelZ_mss();
  IMU_gyro_data_X=(IMU.getGyroX_rads()*rad_to_deg);
  IMU_gyro_data_Y=(IMU.getGyroY_rads()*rad_to_deg);
  IMU_gyro_data_Z=(IMU.getGyroZ_rads()*rad_to_deg);

  //GetTime of IRS, in case value is correct if timer definition changes
  //First division of clock by preescaler (frequency of timer), then inversion (period of timer), then multiplication by overflow (period of channel) 
  //unsigned long dT = (1/(STM32_CLOCK/Timer1.getPrescaleFactor()))*Timer1.getOverflow();  
  unsigned long dT = 100;
  Gpitch += dT*(IMU.getGyroY_rads())*rad_to_deg*0.001; //0,001 es para pasar de milisegundos a segundos  (ANDRES)
  Groll += dT*(IMU.getGyroX_rads())*rad_to_deg*0.001;  
  Gyaw += dT*(IMU.getGyroZ_rads())*rad_to_deg*0.001;
  
  Gpitch = Gpitch + Groll*sin(dT*(IMU.getGyroZ_rads())*0.001);   //gimbal lock compensation
  Groll = Groll - Gpitch*sin(dT*(IMU.getGyroZ_rads())*0.001);

  Atotal = sqrt((IMU.getAccelX_mss() * IMU.getAccelX_mss()) + (IMU.getAccelY_mss() * IMU.getAccelY_mss()) + (IMU.getAccelZ_mss() * IMU.getAccelZ_mss()));   // Calculate the total (3D) vector
  Apitch = asin((float)IMU.getAccelX_mss() / Atotal) * rad_to_deg;                         //Calculate the pitch angle
  Aroll = asin((float)IMU.getAccelY_mss() / Atotal) * rad_to_deg;                         //Calculate the roll angle
  Aroll = asin((float)IMU.getAccelY_mss() / Atotal) * rad_to_deg;                         //Calculate the roll angle

  ///////Els dos valors següents serveixen per calibrar l'accelerometre (tècnicament no és perfecte perque només es el bias, pero funciona prou be) (Andres)///////

        //Per trobarlos: 1r.- Posar un 0.0 als dos valors i descomentar els tres prints que hi ha a sota (nomes els prints, les variables sempre descomentades!)
        //               2n.- Si es vol, comentar la resta de prints del programa per nommés obtenir aquestss valors
        //               3r.- Al compilar, deixar la IMU inmobil per a que es calibri be el giroscopi
        //               4t.- Els valors llegits anotar-los aquí i al tornar a compilar com estava al principi (abans del 1r.), tot haurien de ser zeros coma algo.

  Apitch -= 1.0;
  Aroll -= 0.4;

  //Serial.print(Apitch,6);
  //Serial.print("\t ");
  //Serial.println(Aroll,6);

  if (Gyro_sync)
  {
    // ----- Gyro & accel have been synchronised
    Gpitch = Gpitch * 0.98 + Apitch * 0.02;        //Correct the drift of the gyro pitch angle with the accelerometer pitch angle //(ANDRES) Els valors originals eren 0.9996 i 0.0004
    Groll = Groll * 0.98 + Aroll * 0.02;           //Correct the drift of the gyro roll angle with the accelerometer roll angle   // Pero si ho proveu veureu que tal com està ara s'ajusta tot més ràpid
  }else{                                                                                                                               // El 0.98 i 0.02 surten de http://www.pieter-jan.com/node/11else{
    // ----- Synchronise gyro & accel
    Gpitch = Apitch;                                       //Set the gyro pitch angle equal to the accelerometer pitch angle
    Groll = Aroll;                                         //Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_sync = true;                                      //Set the IMU started flag
  }

  G_pitch_output = G_pitch_output * 0.9 + Gpitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value //Amortigua los valores (filtra, para que no haya tanto pico)
  G_roll_output = G_roll_output * 0.9 + Groll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value   //Los comentarios del codigo https://www.instructables.com/Tilt-Compensated-Compass/
                                                                                                                                  //lo explican (yo creo que un poco así así...)

  //(ANDRES) Hasta aquí el pitch i el roll se equilibran muy bien, siempre vuelven al 0 esta bastante conseguido, dentro de un rango no muy grande (p.e. el pitch a partir de 80º o así ya no va muy fino)
  //pero el yaw no, se puede probar moviendolo a lo loco y se verá que no vuelve al 0, (gimbal lock?). Por eso a partir de aqui el codigo viene de https://www.instructables.com/Tilt-Compensated-Compass/

  Mpitch = -G_roll_output * deg_to_rad;
  Mroll = G_pitch_output * deg_to_rad;

  M_x_eq = IMU.getMagX_uT() * cos(Mpitch) + IMU.getMagY_uT() * sin(Mroll) * sin(Mpitch) - IMU.getMagZ_uT() * cos(Mroll) * sin(Mpitch);
  M_y_eq = IMU.getMagY_uT() * cos(Mroll) + IMU.getMagZ_uT() * sin(Mroll); //eq de equilibrado

  Mag_x_damp = Mag_x_damp * 0.9 + M_x_eq * 0.1;  //Amortigua los valores (Igual que G_pitch_output)
  Mag_y_damp = Mag_y_damp * 0.9 + M_y_eq * 0.1;

  Heading = -atan2(Mag_y_damp, Mag_x_damp) * rad_to_deg;  // Magnetic North //(Andres) He cambiado alguna cosa (x por y) y el signo del principio. Sin mucho criterio pero ahora funciona ( a Mí)
                                                                            // Originalmente, era x/y (hacia que la lectura 0 estuviese 90 grados desplazada) y sin el negativo del principio (hacía
                                                                            // que la lectura sumase los angulos en sentido antihorario en vez de horario).
  
  //Heading += Declination; //Esta linea se puede descomentar si se quiere ser preciso del todo. Añade la declinación del lugar geográfico en el que nos encontramos
                            //https://www.ign.es/web/gmt-declinacion-magnetica (Barcelona dice que es un grado y pico) (ANDRES)

  if (Heading < 0) Heading += 360;
  if (Heading >= 360) Heading -= 360; //Estas dos lineas contienen el valor del heading en 0-360 grados.

//----------------------------------------------------------------------------------------------------------------------REVISAR
  IMU_deg_data_X=G_pitch_output;
  IMU_deg_data_Y=G_roll_output;
  IMU_deg_data_Z=Heading;
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to show IMU_Data on the Serial Monitor
void show_IMU(){

  Serial.print("ACCEL in X, Y, Z:");   Serial.print('\t');
  Serial.print(IMU_accel_data_X,6);   Serial.print('\t');
  Serial.print(IMU_accel_data_Y,6);   Serial.print('\t');
  Serial.println(IMU_accel_data_Z,6);

  Serial.print("GYROS in X, Y, Z:");   Serial.print('\t');
  Serial.print(IMU_gyro_data_X,6);  Serial.print('\t');
  Serial.print(IMU_gyro_data_Y,6);  Serial.print('\t');
  Serial.println(IMU_gyro_data_Z,6);

  Serial.print("DEG º in X, Y, Z:");   Serial.print('\t');
  Serial.print(IMU_deg_data_X,6);    Serial.print('\t');
  Serial.print(IMU_deg_data_Y,6);    Serial.print('\t');
  Serial.println(IMU_deg_data_Z,6);
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to both read and show IMU_Data
void read_show_IMU(){
  read_IMU();
  show_IMU();
}

//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ MODES OF OPERATION
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Mode Selection
void mode_Select(int mode_value){
  switch(mode_value){
    default:   //----------------------Mode OBC Imput Waiting (0): Waiting for OBC
    Serial.println("Reading mode from OBC");
    mode_OBC_Imput_Wait();
    break;
    case 1:   //----------------------Mode Positioning RW only
    Serial.println("Mode Positioning");
    mode_Positioning_RW();
    break;
    case 2:   //----------------------Mode IMU reading (NO USE)
    Serial.println("Reading IMU");
    mode_IMU_reading();
    break;
  }
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ MODE IMU ONLY FOR TESTING
void mode_IMU_reading(){ 
  OBC_mode_value=0;
  Timer1.attachInterrupt(TIMER_CH2, read_show_IMU);
  //read_show_IMU();
  //mode_Select(OBC_mode_value);
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 0. Mode Default, OBC Reading
void mode_OBC_Imput_Wait(){ 
  OBC_mode_value=0;
  Timer1.attachInterrupt(TIMER_CH1, OBC_mode_receive);
  while(OBC_mode_value==0){ 
    delay(1); //If not used the while function does not work
    //it can be added more conditions to evade being blocked until a data is received.
    //for example, it could function an interrupt with a forced exit and an if after or something
  }
  Timer1.detachInterrupt(TIMER_CH1);
  Serial.println(OBC_mode_value);
  mode_Select(OBC_mode_value);
}

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 1. Mode Positioning (RW only)
void mode_Positioning_RW(){
  OBC_mode_value=0;
  Serial.println ("Positioning begin");
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
 Serial.print("Axis Z position: ");
 Serial.println(IMU_deg_data_Z);
 Serial.println("Insert degree value to turn");
 OBC_data_value=0;
 Timer1.attachInterrupt(TIMER_CH1, OBC_data_receive);
 while(OBC_data_value==0){ 
   delay(1); //If not used the while function does not work
 }
 Timer1.detachInterrupt(TIMER_CH1);

 if(OBC_data_value<=Degree_PointingMode_Tolerancy){
   positioning_Fine();
 }else{
   positioning_Coarse();
 }
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ 1.1. Mode Positioning Coarse
void positioning_Coarse(){     //For now simple
  Serial.println("Mode Positioning Coarse");
  float degree_turn_value=OBC_data_value;   //Stores a new variable to not overwrite the original value
  bool rw_direction=true;   //true=positive (CCW), false=negative (CW)
//  In case OBC_data_value is positive, value remains the same and rw direction is true, the default value of the variable
//  In case OBC_data_value is negative, value change to positive and rw direction is false

  if(degree_turn_value<0){     
    degree_turn_value=-degree_turn_value;
    rw_direction=false;
  }

  float Initial_IMU_degree_value, Final_IMU_degree_value, Delta_degree_ramp, Degree_stop_wait, 
  float Prev_IMU_deg_data_Z, Check_IMU_deg_data_Z;
  int overlap_count=0; 
  bool waiting=true;

//----------------------------------------------------------ACCELERATION

  read_show_IMU();
  Initial_IMU_degree_value=IMU_deg_data_Z;    //Stores initial degree, only in Z

  ramp_definition(rw_direction, 0, 255);  
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

//--------------------------------------------------------------WAITING
  
  if(rw_direction){ //CASE CCW
    if((degree_turn_value - 2*Delta_degree_ramp)>0){ //if its <0, it must be done inmediatly after, and still it would be too much turn.
      Degree_stop_wait = Final_IMU_degree_value + (degree_turn_value - 2*Delta_degree_ramp);  //Get value of degree to start
    
      Timer1.attachInterrupt(TIMER_CH2, read_IMU());
      while(waiting){   //Stays as long as waitin is true.
        Serial.println("Waiting");
        
        //CAUTION WITH READING VALUES; AS IT IS ALWAYS FROM 0 TO 360
        // Degree_stop_wait will always be > IMU_deg_data_Z. If IMU_deg_data_Z>> (ex: 359º),  Degree_stop_wait can be >360. Thus the value of If IMU_deg_data_Z would never reach Degree_stop_wait
        // Degree_stop_wait cant be decreased, as the way to check if its reached is by a greater. If it is decreased by 360º (so it stays in relative place), it could be < IMU_deg_data_Z and would inmediatly exit the while without waiting.
        // the way to do is check if there is a heavy change on IMU_deg_data_Z (pass on 0) to check iff adding or substracting a lap, making it go out of the 0 and 360 range.
        //It should not be a high acceleration enough to make a jump of degree of 180º in such short time,so it sholud be fine.
        //Caution disconnection from IMU, could give a false jump, that is why a check on IMU_deg_data_Z first.
        
        if(IMU_deg_data_Z<0.0001 && IMU_deg_data_Z>-0.0001){  //if it is almost exactly 0 then most probably there is a disconnection. Close numbers should not trigger it.  
        }else if(IMU_deg_data_Z-Prev_IMU_deg_data_Z>-180){ //Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 359 to 0.
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
      Timer1.detachInterrupt(TIMER_CH2);
    }
  }else{  //CASE CW
    if((degree_turn_value - 2*Delta_degree_ramp)>0){ //if its <0, it must be done inmediatly after, and still it would be too much turn.
      Degree_stop_wait = Final_IMU_degree_value - (degree_turn_value - 2*Delta_degree_ramp);  //Get value of degree to start
    
      Timer1.attachInterrupt(TIMER_CH2, read_IMU());
      while(waiting){   //Stays as long as waitin is true.
        Serial.println("Waiting");
        
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
        }else if(Check_IMU_deg_data_Z-Prev_IMU_deg_data_Z>-180){ //Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 0 to 359.
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
      Timer1.detachInterrupt(TIMER_CH2);
    }
  }

//----------------------------------------------------------DECELERATION

   ramp_definition(rw_direction, 0, 0); 
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
  
  if(Delta_degree_ramp-degree_turn_value)<Degree_Total_Tolerancy || (IMU_degree_value-degree_turn_value)>Degree_Total_Tolerancy){  //Real turn vs wanted turn, checks if it is considered good
     mode_Select(OBC_mode_value); //Valid position
  }else{
     positioning_Fine(OBC_data_value); //Correction
  }
  
}

//░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ 1.2. Mode Positioning Coarse
void positioning_Fine(){     //For now nothing 
Serial.println("Mode Positioning Fine");
//PD controller
OBC_data_value=0;
Serial.println("End of Manouver");
mode_Select(OBC_mode_value);
}

//██████████████████████████████████████████████████████████████████████ VOID SETUP
void setup() {
  
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Initiations
  Serial.begin(9600);
  SPI.begin();
  delay(5000);
  Serial.println("START");
  
//▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Setups

  pinMode(LEDPIN,OUTPUT);   //Integrated LED 
  pinMode(CS1, OUTPUT);   //NCS Pin definition

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Driver Setup
  Serial.println("Configuring Driver...");
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
  Serial.println("Driver Ready to Use!");

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Setup
  Serial.println("Configuring MPU9250...");
  int  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);   
    while(1){}
  }

  IMU.setGyroRange(IMU.GYRO_RANGE_500DPS);

  

//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Calibration
   //Use for getting the values on calibration setting below
  IMU.calibrateMag();
  Serial.println("Done");    
 
  Serial.print(IMU.getMagBiasX_uT());
  Serial.print(", ");
  Serial.print(IMU.getMagBiasY_uT());
  Serial.print(", ");
  Serial.println(IMU.getMagBiasZ_uT());

  Serial.print(IMU.getMagScaleFactorX());
  Serial.print(", ");
  Serial.print(IMU.getMagScaleFactorY());
  Serial.print(", ");
  Serial.println(IMU.getMagScaleFactorZ());

  Serial.println("Calibrating MPU9250");
  IMU.setMagCalX(11.27,2.18); //Primer valor el MagBias, y el segundo el ScaleFactor!
  IMU.setMagCalY(-5.27,4.57);
  IMU.setMagCalZ(58.66,0.43);

  Serial.println("MPU9250 Ready to Use!");
  
//▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Timers
  Serial.println("Configuring Timers...");
  Timer1.pause();
  Timer1.setPrescaleFactor(7200);   //72MHz Clock / 7200 = 10KHz timer
  Timer1.setOverflow(1000);    //Overflow occurs at 1000, each 100 ms timer restarts
    
  Timer1.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);    //Configure channel to OUTPURCOMPARE: Channel for OBC read values
  Timer1.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);    //Channel for IMU read values
  Timer1.setCompare(TIMER_CH1, 1);    //Phase value in Overflow range
  Timer1.setCompare(TIMER_CH2, 1);
  
  //Timer1.attachInterrupt(TIMER_CH1, OBC_mode_receive);  //Interruption for OBC receive of mode
  Timer1.refresh();   //Refresh timer and start over
  Timer1.resume();

  Serial.println("Timers Ready to Use!");
  Serial.println("ADCS Control START");
  Serial.println("Reading mode from OBC");
  mode_OBC_Imput_Wait();
}

//██████████████████████████████████████████████████████████████████████ VOID LOOP
void loop() {

}
