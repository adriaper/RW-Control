/* Reaction Wheel controller
 *
 * The following code provides a control for PLATHON project's CubeSat.
 *
 * 
 * The following libraries are used:
 * STM32 Bluepill microcontroller package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
 * SCMD Driver library by SparkFun used (https:// github.com/sparkfun/SparkFun_Serial1_Controlled_Motor_Driver_Arduino_Library)
 * MPU9250 IMU library by Rafa Castalla used (https://github.com/rafacastalla/MPU9250-1)
 * read_IMU() and IMU setup content code extraconstantd from Andrés Gómez and Miquel Reurer, from the PLATHON group (magnetorquers section)
 */

// ██████████████████████████████████████████████████████████████████████ DEFINITIONS
// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ LIBRARIES
#include <Arduino.h>     // Arduino library
#include <stdint.h>      // Integer types library
#include <SCMD.h>        // Serial Controlled Motor Driver library
#include <SCMD_config.h> // Serial Controlled Motor Driver Configuration library
#include <SPI.h>         // SPI library
#include <MPU9250.h>     // IMU library

// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ DEFINITIONS
#define LEDPIN PC13                // Integrated LED of the Bluepill
#define CS1 PA4                    // STM32 NCS Chip select (SPI mode only) pin. MOSI on PA7, MISO on PA6 and CLK on PA5 by default on STM32
#define STM32_CLOCK 72000          // Internal STM32 Clock (72MHz, in kHz so period is millisec)
#define PI 3.14159265              // Number PI
#define Rad_to_deg 57.29577951     // Convert radians to degrees
#define Deg_to_rad 0.01745329      // Convert degrees to radians
#define Final_pointing_tolerancy 1 // Tolerancy for final pointing (+-1 degree of tolerancy) (Check if position is within the margin)
#define Pointing_mode_tolerancy 5  // Tolerancy for selecting pointing mode (+-5 degree of tolerancy) (Select whether to use coarse or fine mode)
#define Accel_tolerancy 0.5        // Tolerancy for acceleration measurement (+-0.5 unit of acceleration of tolerancy) (Acceptable error of acceleration)
#define Gyro_tolerancy 1           // Tolerancy for gyro measurement (+-1 unit of gyroscope Z tolerancy) (Accounts to know if it is rotating or not at a constant speed for the ramp)

SCMD DriverOne;      // Driver Object definition
MPU9250 IMU(SPI, 4); // MPU object definition

// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GLOBAL VARIABLES
float IMU_accel_data_X, IMU_accel_data_Y, IMU_accel_data_Z; // Values of local IMU accelerometer (m/s^2)
float IMU_gyro_data_X, IMU_gyro_data_Y, IMU_gyro_data_Z;    // Values of local IMU gyroscope (degrees/s)
float IMU_mag_data_X, IMU_mag_data_Y, IMU_mag_data_Z;       // Values of local IMU magnetometer (uT)
float Pitch_deg, Roll_deg, Yaw_deg = 0;                     // Pitch, Roll and Yaw (degrees)

// Declare variables outside IMU, since variables initiates to 0 without defining them in function
float Accel_total_vector_modulus;                 // Accelerometer total vector magnitude (m/s^2)
float Accel_pitch_deg, Accel_roll_deg;            // Accelerometer pitch and roll angles (degrees)
float Gyro_pitch_deg, Gyro_roll_deg;              // Gyroscope pitch and roll angles (degrees)
bool Gyro_Accel_sync;                             // Check if Gyroscope and Accelerometer are synchronized
float Mag_X_calc, Mag_Y_calc;                     // Calculated Magnetometer value (uT)
float Mag_X_correconstantd, Mag_Y_correconstantd; // Correconstantd Magnetometer value (with damping) (uT)
float Pitch_rad, Roll_rad;                        // Pitch and Roll angles (rad)

int RW_speed = 0;       // Value of Reaction Wheel speed [from -255 to 255]
int OBC_mode_value = 0; // Value of On Board Computer mode (waiting, positioning, detumbling, etc.)
int OBC_data_value = 0; // Value of On Board Computer data (desired angle turn, etc.)

// PID control definitions
float Deg_to_reach = 0;  // Setpoint angle (degrees)
bool Zero_state = false; // Check if PID encompass yaw of 0 degrees (to turn the value to a workable zone)
double PID_output = 0;   // Output value of the PID [from 0 to 255]

// ██████████████████████████████████████████████████████████████████████ FUNCTIONS
// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GENERAL USE FUNCTIONS
// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ OBC Functions
void OBC_mode_receive()
{
  /*
    Function to set the Bluepill to receiving mode. Uses Timer CH3. Serial1 as UART communication

    INPUT: User Serial1 input
    OUTPUT: Save OBC_mode_value
  */

  // Check if UART Serial1 is available
  if (Serial1.available() > 0)
  {
    String bufferString = ""; // String for buffer of Serial1
    // Keep saving input prompt
    while (Serial1.available() > 0)
    {
      bufferString += (char)Serial1.read(); // Adds chars to the Serial1 buffer
    }
    // Conversion from String to int
    OBC_mode_value = bufferString.toInt();
    Serial1.print("Mode Number: ");
    Serial1.println(OBC_mode_value);
  }
}

void OBC_data_receive()
{
  /*
    Function to set the Bluepill to receiving data mode. Uses Timer CH3. Serial1 as UART communication

    INPUT: User Serial1 input
    OUTPUT: Save OBC_data_value
  */

  // Check if UART Serial1 is available
  if (Serial1.available() > 0)
  {
    String bufferString = ""; // String for buffer of Serial1
    // Keep saving input prompt
    while (Serial1.available() > 0)
    {
      bufferString += (char)Serial1.read(); // adds chars to the Serial1 buffer
    }
    OBC_data_value = bufferString.toInt(); // Conversion from String to int
    Serial1.print("Value to turn: ");      // In this case turn, but can be what is needed
    Serial1.println(OBC_data_value);
  }
}

// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Driver Functions
void set_impulse(bool RW_direction, int New_RW_speed)
{
  /*
    Function to set the motor at a fixed direction and speed

    INPUT: 
      RW_direction: Reaction Wheel direction [Clockwise or Counter Clockwise]
      New_RW_speed: New reaction wheel speed [from 0 to 255]
    OUTPUT: 
      Save speed of the Reaction Wheel
  */

  // Account for change in direction
  if (RW_direction) // Reaction Wheel Counter Clock Wise
  {
    DriverOne.setDrive(0, 0, New_RW_speed); // Change direction depending on motor connection
    RW_speed = New_RW_speed;
  }
  else // Reaction Wheel Clock Wise
  {
    DriverOne.setDrive(0, 1, New_RW_speed);
    RW_speed = -New_RW_speed;
  }
}

void generate_ramp(bool RW_direction, int Acc_ramp_time_duration, int RW_ramp_speed_reach, bool Acc_Dec_state)
{
  /*
    Function to generate ramps. In this case it is a Single Impulse.
    As we dont know the time it lasts, we have to see if speed changes on the cubesat. That is the use of the while.

    INPUT: 
      RW_direction: Reaction Wheel direction [Clockwise or Counter Clockwise]
      Acc_ramp_time_duration: Duration of ramp
      RW_ramp_speed_reach: Final speed reached
      Acc_Dec_state: Check wether in acceleration state ('1') or deceleration state ('0')
    OUTPUT: 
      Save speed of the Reaction Wheel
  */

  Serial1.println("Ramp Start");
  bool waiting = true;

  set_impulse(RW_direction, RW_ramp_speed_reach);

  Timer1.attachInterrupt(TIMER_CH4, read_show_IMU);
  waiting = true;

  while (waiting)
  { // Range of tolerancy
    if (Acc_Dec_state)
    { // If accelerating
      if (abs(IMU_gyro_data_Z) > Gyro_tolerancy)
      { // If gyro is not 0 or so, means it has constant speed of rotation
        if (abs(IMU_accel_data_X) < Accel_tolerancy)
        { // Stop of acceleration
          waiting = false;
        }
      }
    }
    else
    { // If decelerating
      if (abs(IMU_gyro_data_Z) < Gyro_tolerancy)
      { // If gyro is not 0 or so, means it has constant speed of rotation
        if (abs(IMU_accel_data_X) < Accel_tolerancy)
        { // Stop of acceleration
          waiting = false;
        }
      }
    }
    delay(1); // Delay for ensuring getting inside the while loop
  }
  waiting = true;
  Timer1.detachInterrupt(TIMER_CH4);
}

// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Functions
// ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to read IMU_Data (and refreshes the global variables for IMU data)
void read_IMU()
{

  IMU.readSensor();

  IMU_accel_data_X = IMU.getAccelX_mss();
  IMU_accel_data_Y = IMU.getAccelY_mss();
  IMU_accel_data_Z = IMU.getAccelZ_mss();
  IMU_gyro_data_X = (IMU.getGyroX_rads() * Rad_to_deg);
  IMU_gyro_data_Y = (IMU.getGyroY_rads() * Rad_to_deg);
  IMU_gyro_data_Z = (IMU.getGyroZ_rads() * Rad_to_deg);
  IMU_mag_data_X = IMU.getMagX_uT();
  IMU_mag_data_Y = IMU.getMagY_uT();
  IMU_mag_data_Z = IMU.getMagZ_uT();

  // GetTime of IRS, in case value is correct if timer definition changes
  // First division of clock by preescaler (frequency of timer), then inversion (period of timer), then multiplication by overflow (period of channel)
  float dT = ((1 / ((float)STM32_CLOCK / (float)Timer1.getPrescaleFactor())) * Timer1.getOverflow()) * 0.001; // in seconds

  Gyro_pitch_deg += dT * (IMU.getGyroY_rads()) * Rad_to_deg;
  Gyro_roll_deg += dT * (IMU.getGyroX_rads()) * Rad_to_deg;

  Gyro_pitch_deg = Gyro_pitch_deg + Gyro_roll_deg * sin(dT * (IMU.getGyroZ_rads())); // gimbal lock compensation
  Gyro_roll_deg = Gyro_roll_deg - Gyro_pitch_deg * sin(dT * (IMU.getGyroZ_rads()));

  Accel_total_vector_modulus = sqrt((IMU.getAccelX_mss() * IMU.getAccelX_mss()) + (IMU.getAccelY_mss() * IMU.getAccelY_mss()) + (IMU.getAccelZ_mss() * IMU.getAccelZ_mss())); //  Calculate the total (3D) vector
  Accel_pitch_deg = asin((float)IMU.getAccelX_mss() / Accel_total_vector_modulus) * Rad_to_deg;                                                                               // Calculate the pitch angle
  Accel_roll_deg = asin((float)IMU.getAccelY_mss() / Accel_total_vector_modulus) * Rad_to_deg;                                                                                // Calculate the roll angle

  /* Calibracion del acelerometro. */

  //  Para calibrar el acelerometro se pone en las variables los valores 0.0 y se descomenta los 3 prints.
  //  Al compilar, deja la IMU inmobil para que se calibre correctamente el giroscopio.
  //  Una vez obtenidos lo valores, se anotan y se vuelve a compilar como se habia dejado antes.

  Accel_pitch_deg -= 2.62;
  Accel_roll_deg -= 4.01;

  //    Serial.print(Accel_pitch_deg,6);
  //    Serial.print("\t");
  //    Serial.println(Accel_roll_deg,6);

  if (Gyro_Accel_sync)
  {
    //  ----- Gyro & accel have been synchronized
    Gyro_pitch_deg = Gyro_pitch_deg * 0.95 + Accel_pitch_deg * 0.05; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle // (ANDRES) Els valors originals eren 0.9996 i 0.0004
    Gyro_roll_deg = Gyro_roll_deg * 0.95 + Accel_roll_deg * 0.05;    // Correct the drift of the gyro roll angle with the accelerometer roll angle   //  Pero si ho proveu veureu que tal com està ara s'ajusta tot més ràpid
  }
  else
  { //  El 0.98 i 0.02 surten de http:// www.pieter-jan.com/node/11else{
    //  ----- Synchronize gyro & accel
    Gyro_pitch_deg = Accel_pitch_deg; // Set the gyro pitch angle equal to the accelerometer pitch angle
    Gyro_roll_deg = Accel_roll_deg;   // Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_Accel_sync = true;           // Set the IMU started flag
  }

  Pitch_deg = Pitch_deg * 0.9 + Gyro_pitch_deg * 0.1;
  Roll_deg = Roll_deg * 0.9 + Gyro_roll_deg * 0.1;

  Pitch_rad = -Roll_deg * Deg_to_rad;
  Roll_rad = Pitch_deg * Deg_to_rad;

  Mag_X_calc = IMU.getMagX_uT() * cos(Pitch_rad) + IMU.getMagY_uT() * sin(Roll_rad) * sin(Pitch_rad) - IMU.getMagZ_uT() * cos(Roll_rad) * sin(Pitch_rad);
  Mag_Y_calc = IMU.getMagY_uT() * cos(Roll_rad) + IMU.getMagZ_uT() * sin(Roll_rad);

  //  Amortigua los valores del mismo modo que Pitch_deg.

  Mag_X_correconstantd = Mag_X_correconstantd * 0.9 + Mag_X_calc * 0.1;
  Mag_Y_correconstantd = Mag_Y_correconstantd * 0.9 + Mag_Y_calc * 0.1;

  //  El norte magnetico se ha modificado, originalmente, era de x a y (hacia que la lectura 0 estuviese 90 grados desplazada)
  //  y sin el negativo del principio (hacia que la lectura sumase los angulos en sentido antihorario en vez de horario).

  Yaw_deg = -atan2(Mag_Y_correconstantd, Mag_X_correconstantd) * Rad_to_deg;

  //  Si se quiere ser preciso del todo, se añade la declinacion del lugar geografico en el que nos encontramos (https:// www.ign.es/web/gmt-declinacion-magnetica)

  // Yaw_deg += Declination;

  //  Las siguientes dos lineas contienen el valor de Yaw_deg entre 0 y 360 grados.

  if (Yaw_deg < 0)
    Yaw_deg += 360;
  if (Yaw_deg >= 360)
    Yaw_deg -= 360;
}

// ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to show IMU_Data on the Serial1 Monitor
void show_IMU()
{

  Serial1.print("A: ");
  Serial1.print(IMU_accel_data_X, 4);
  Serial1.print('\t');
  Serial1.print(IMU_accel_data_Y, 4);
  Serial1.print('\t');
  // Serial1.print(IMU_accel_data_Z,4);
  //       Serial1.println("");
  // Serial1.print(" / G: ");
  // Serial1.print(IMU_gyro_data_X,4);  Serial1.print('\t');
  // Serial1.print(IMU_gyro_data_Y,4);  Serial1.print('\t');
  // Serial1.print(IMU_gyro_data_Z,4);   Serial1.print('\t');
  // Serial1.print(gyro_Z_offseted,4);
  //      Serial1.println("");
  // Serial1.print("MAGS: ");
  // Serial1.print(IMU_mag_data_X,4);    Serial1.print('\t');
  // Serial1.print(IMU_mag_data_Y,4);    Serial1.println('\t');
  // Serial1.print(IMU_mag_data_Z,4);
  //       Serial1.println("");
  // Serial1.print(" / D: ");
  // Serial1.print(Pitch_deg,4);
  // Serial1.print(Roll_deg,4);
  // Serial1.print(Yaw_deg,4);
  Serial1.println("");
}

// ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to both read and show IMU_Data
void read_show_IMU()
{
  read_IMU();
  show_IMU();
}

// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ PID Functions
// ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ Function to calculate PID
void computePID()
{

  double error, lastError;        //  Initialize error and previousError
  double input, output, setPoint; //  Initialize input variable from IMU, the output variable, and the desired setPoint
  double cumError, rateError;     //  Initialize the cumulative Error (Integral) and the rate of Error (Derivative)

  double kp = 3; //  Proportional contribution
  double ki = 4; //  Integral contribution
  double kd = 0; //  Derivative contribution

  float dT = ((1 / ((float)STM32_CLOCK / (float)Timer1.getPrescaleFactor())) * Timer1.getOverflow()) * 0.001; // in seconds

  read_IMU();

  if (Zero_state)
  {                // saves if the pid pass through 0, if it does it moves the zone away from 0
    Yaw_deg + 180; // Addition of 180 deg to all values. the error is a substract, so the difference is the same
    Deg_to_reach + 180;
  }

  if (Yaw_deg >= 360)
    Yaw_deg -= 360; // one of them will increase over 360, it is a correction
  if (Deg_to_reach >= 360)
    Deg_to_reach -= 360;

  //  Percentage
  volatile float Yaw_deg_perc = (Yaw_deg - 360) / (-360) * 100;

  //  Transform angle to percentage
  volatile float Deg_to_reach_perc = (Deg_to_reach - 360) / (-360) * 100;

  //  Errors
  error = Deg_to_reach_perc - Yaw_deg_perc; //  Calculate error (Proportional)
  cumError += error * dT;                   //  Calculate the cumulative error (Integral)
  rateError = (error - lastError) / dT;     //  Calculate the rate of error (Derivative)

  //  PID Control
  float PID_P = kp * error;     //  Proportional
  float PID_I = ki * cumError;  //  Integral
  float PID_D = kd * rateError; //  Derivative

  if (PID_P > 255)
    PID_P = 255;
  if (PID_P < -255)
    PID_P = -255;
  if (PID_I > 255)
    PID_I = 255;
  if (PID_I < -255)
    PID_I = -255;
  if (PID_D > 255)
    PID_D = 255;
  if (PID_D < -255)
    PID_D = -255;

  float PID_output = PID_P + PID_I + PID_D; //  PID control

  if (PID_output > 255)
    PID_output = 255;
  if (PID_output < -255)
    PID_output = -255;

  bool RW_direction = true; // true=positive (CCW), false=negative (CW)

  if (PID_output < 0)
  {
    PID_output = -PID_output;
    RW_direction = false;
  }

  set_impulse(RW_direction, PID_output);
  //  Save current error and time for next iteration
  lastError = error; //  Save current error
}

// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ MODES OF OPERATION
// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Mode Selection
void mode_Select(int mode_value)
{
  /*
    ['0': Wainting; '1': Positioning; '2': Reading]
  */
  switch (mode_value)
  {
  default: // ----------------------Mode OBC Imput Waiting (0): Waiting for OBC
    Serial1.println("Reading mode from OBC");
    mode_OBC_Imput_Wait();
    break;
  case 1: // ----------------------Mode Positioning RW only
    Serial1.println("Mode Positioning");
    mode_Positioning_RW();
    break;
  case 2: // ----------------------Mode IMU reading (NO USE)
    Serial1.println("Reading IMU");
    mode_IMU_reading();
    break;
  }
}

// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ MODE IMU ONLY FOR TESTING
void mode_IMU_reading()
{
  OBC_mode_value = 0;
  Timer1.attachInterrupt(TIMER_CH4, read_show_IMU);
  // read_show_IMU();
  // mode_Select(OBC_mode_value);
}

// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 0. Mode Default, OBC Reading
void mode_OBC_Imput_Wait()
{
  OBC_mode_value = 0;
  Timer1.attachInterrupt(TIMER_CH3, OBC_mode_receive);
  while (OBC_mode_value == 0)
  {
    delay(1); // If not used the while function does not work
    // it can be added more conditions to evade being blocked until a data is received.
    // for example, it could function an interrupt with a forced exit and an if after or something
  }
  Timer1.detachInterrupt(TIMER_CH3);
  Serial1.println(OBC_mode_value);
  mode_Select(OBC_mode_value);
}

// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ 1. Mode Positioning (RW only)
void mode_Positioning_RW()
{
  OBC_mode_value = 0;
  Serial1.println("Positioning begin");
  //   Insert code to select if the positioning will be coarse (impulses) or fine (PD)
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
  Serial1.println(Yaw_deg);
  Serial1.println("Insert degree value to turn");
  OBC_data_value = 0;
  Timer1.attachInterrupt(TIMER_CH3, OBC_data_receive);
  while (OBC_data_value == 0)
  {
    delay(1); // If not used the while function does not work
  }
  Timer1.detachInterrupt(TIMER_CH3);

  if (abs(OBC_data_value) <= Pointing_mode_tolerancy)
  {
    positioning_Fine();
  }
  else
  {
    positioning_Coarse();
  }
}

// ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ 1.1. Mode Positioning Coarse
void positioning_Coarse()
{ // For now simple
  Serial1.println("Mode Positioning Coarse");

  float degree_turn_value = OBC_data_value; // Stores a new variable to not overwrite the original value
  bool RW_direction = true;                 // true=positive (CCW), false=negative (CW)
                                            //   In case OBC_data_value is positive, value remains the same and rw direction is true, the default value of the variable
                                            //   In case OBC_data_value is negative, value change to positive and rw direction is false

  if (degree_turn_value < 0)
  {
    degree_turn_value = -degree_turn_value;
    RW_direction = false;
  }

  float Initial_IMU_degree_value, Final_IMU_degree_value, Delta_degree_ramp, Degree_stop_wait;
  float Prev_Yaw_deg, Check_Yaw_deg;
  int overlap_count = 0;
  bool waiting = true;

  // ----------------------------------------------------------ACCELERATION

  read_show_IMU();
  Initial_IMU_degree_value = Yaw_deg; // Stores initial degree, only in Z
  int initial_RW_speed = RW_speed;
  generate_ramp(RW_direction, 0, 255, 1);
  // 0 will not be used as we dont define the time the motor lasts to do an impulse. 255 is the max value
  // This values will be sustituted by how we want the time and speed to reach in the ramp.
  read_IMU();
  Final_IMU_degree_value = Yaw_deg; // Stores final degree, only in Z
  if (RW_direction)
  {
    Delta_degree_ramp = Final_IMU_degree_value - Initial_IMU_degree_value; // degree turnt on acc, only in Z. Shoud be positive
  }
  else
  {
    Delta_degree_ramp = Initial_IMU_degree_value - Final_IMU_degree_value; // degree turnt on acc, only in Z. Shoud be positive
  }

  if (Delta_degree_ramp < 0)
  {
    Delta_degree_ramp += 360; // In case its negative adds 360
    // Negative cases: changes goes by 0º. EX: 330 to 30 when CCW (should be 60 but calculus is 30-330= -300)
    //                                     EX: 30 to 330 when CW  (should be 60 but calculus is 30-330= -300)
  }
  Serial1.print("ID: ");
  Serial1.print(Initial_IMU_degree_value);
  Serial1.print(" / T: ");
  Serial1.print(Delta_degree_ramp);
  Serial1.print(" / ED: ");
  Serial1.println(Final_IMU_degree_value);
  // --------------------------------------------------------------WAITING
  waiting = true;
  if (RW_direction)
  { // CASE CCW
    if ((degree_turn_value - 2 * Delta_degree_ramp) > 0)
    {                                                                                          // if its <0, it must be done inmediatly after, and still it would be too much turn.
      Degree_stop_wait = Final_IMU_degree_value + (degree_turn_value - 2 * Delta_degree_ramp); // Get value of degree to start

      Timer1.attachInterrupt(TIMER_CH4, read_IMU);
      Serial1.println("Waiting");
      while (waiting)
      { // Stays as long as waiting is true.

        // CAUTION WITH READING VALUES; AS IT IS ALWAYS FROM 0 TO 360
        //  Degree_stop_wait will always be > Yaw_deg. If Yaw_deg>> (ex: 359º),  Degree_stop_wait can be >360. Thus the value of If Yaw_deg would never reach Degree_stop_wait
        //  Degree_stop_wait cant be decreased, as the way to check if its reached is by a greater. If it is decreased by 360º (so it stays in relative place), it could be < Yaw_deg and would inmediatly exit the while without waiting.
        //  the way to do is check if there is a heavy change on Yaw_deg (pass on 0) to check iff adding or substracting a lap, making it go out of the 0 and 360 range.
        // It should not be a high acceleration enough to make a jump of degree of 180º in such short time,so it sholud be fine.
        // Caution disconnection from IMU, could give a false jump, that is why a check on Yaw_deg first.

        if (Yaw_deg < 0.0001 && Yaw_deg > -0.0001)
        { // if it is almost exactly 0 then most probably there is a disconnection. Close numbers should not trigger it.
        }
        else if (Yaw_deg - Prev_Yaw_deg < -180)
        {                     // Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 359 to 0.
          overlap_count += 1; // adds a lap
        }
        else if (Yaw_deg - Prev_Yaw_deg > +180)
        {                     // Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 0 to 359. (Not possible in theory, as it increases, but deviations could mess it up)
          overlap_count -= 1; // sustracts a lap
        }                     // if niether are triggered, change has been samall or no change has been done yet
        Prev_Yaw_deg = Yaw_deg;
        Check_Yaw_deg = Yaw_deg + (360 * overlap_count); // Rewriting of value, in new variable so it does not add itself.
        if (Check_Yaw_deg > Degree_stop_wait)
        {                  // As it is CCW, degree increases. When reading is > to stop value, exits the while
          waiting = false; // exit condition
        }
        delay(1); // To solve errors
        // No writing of read_IMU as it is already done by a timer. Just to remind Yaw_deg is constantly reading values.
      }
      Timer1.detachInterrupt(TIMER_CH4);
    }
  }
  else
  { // CASE CW
    if ((degree_turn_value - 2 * Delta_degree_ramp) > 0)
    {                                                                                          // if its <0, it must be done inmediatly after, and still it would be too much turn.
      Degree_stop_wait = Final_IMU_degree_value - (degree_turn_value - 2 * Delta_degree_ramp); // Get value of degree to start

      Timer1.attachInterrupt(TIMER_CH4, read_IMU);
      while (waiting)
      { // Stays as long as waitin is true.
        Serial1.println("Waiting");

        // CAUTION WITH READING VALUES; AS IT IS ALWAYS FROM 0 TO 360
        //  Degree_stop_wait will always be < Yaw_deg. If Yaw_deg<< (ex: 1º),  Degree_stop_wait can be <0. Thus the value of If Yaw_deg would never reach Degree_stop_wait
        //  Degree_stop_wait cant be increased, as the way to check if its reached is by a greater. If it is increased by 360º (so it stays in relative place), it could be > Yaw_deg and would inmediatly exit the while without waiting.
        //  the way to do is check if there is a heavy change on Yaw_deg (pass on 0) to check iff adding or substracting a lap, making it go out of the 0 and 360 range.
        // It should not be a high acceleration enough to make a jump of degree of 180º in such short time, so it sholud be fine.
        // Caution disconnection from IMU, could give a false jump, that is why a check on Yaw_deg first.

        Check_Yaw_deg = Yaw_deg; // New variable so it does not change during an iteration
        if (Check_Yaw_deg < 0.0001 && Check_Yaw_deg > -0.0001)
        { // if it is almost exactly 0 then most probably there is a disconnection. Close numbers should not trigger it.
        }
        else if (Check_Yaw_deg - Prev_Yaw_deg > +180)
        {                     // Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 359 to 0. (Not possible in theory, as it decreases, but deviations could mess it up)
          overlap_count += 1; // adds a lap
        }
        else if (Check_Yaw_deg - Prev_Yaw_deg < -180)
        {                     // Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 0 to 359.
          overlap_count -= 1; // sustracts a lap
        }                     // if niether are triggered, change has been samall or no change has been done yet

        Prev_Yaw_deg = Check_Yaw_deg;
        Check_Yaw_deg = Check_Yaw_deg + (360 * overlap_count); // Rewriting of value. It does not add itself as it gets the read value in each iteration. In other words, it will not go throug the iteration with numbers outside 0 and 360 range
        if (Check_Yaw_deg < Degree_stop_wait)
        {                  // As it is CCW, degree increases. When reading is > to stop value, exits the while
          waiting = false; // exit condition
        }
        delay(1); // To solve errors
        // No writing of read_IMU as it is already done by a timer. Just to remind Yaw_deg is constantly reading values.
      }
      Timer1.detachInterrupt(TIMER_CH4);
    }
  }

  // ----------------------------------------------------------DECELERATION
  Serial1.print("ID: ");
  Serial1.print(Final_IMU_degree_value);
  Serial1.print(" / ED: ");
  Serial1.println(Degree_stop_wait);

  generate_ramp(RW_direction, initial_RW_speed, 0, 0); // Initial_RW_Speed returns speed to original value instead of 0
                                                       // This is to assure the impulse is the same, as if there is some momentum accumulation, returning to 0 would be a different impulse
                                                       // 0 will not be used as we dont define the time the motor lasts to do an impulse. 255 is the max value
                                                       // This values will be sustituted by how we want the time and speed to reach in the ramp.
  read_IMU();
  Final_IMU_degree_value = Yaw_deg; // can be overwritten, final degree of manouver
  if (RW_direction)
  {
    Delta_degree_ramp = Final_IMU_degree_value - Initial_IMU_degree_value; // Shoud be positive
  }
  else
  {
    Delta_degree_ramp = Initial_IMU_degree_value - Final_IMU_degree_value; // Shoud be positive
  }
  if (Delta_degree_ramp < 0)
  {                           // can be overwritten, real turn of manouver
    Delta_degree_ramp += 360; // In case its negative adds 360
    // Negative cases: changes goes by 0º. EX: 330 to 30 when CCW (should be 60 but calculus is 30-330= -300)
    //                                     EX: 30 to 330 when CW  (should be 60 but calculus is 30-330= -300)
  }

  Serial1.print("ID: ");
  Serial1.print(Degree_stop_wait);
  Serial1.print(" / T: ");
  Serial1.print(Delta_degree_ramp);
  Serial1.print(" / ED: ");
  Serial1.println(Final_IMU_degree_value);

  if ((Delta_degree_ramp - degree_turn_value) < Final_pointing_tolerancy && (Delta_degree_ramp - degree_turn_value) > Final_pointing_tolerancy)
  {                              // Real turn vs wanted turn, checks if it is considered good
    mode_Select(OBC_mode_value); // Valid position
  }
  else
  {
    positioning_Fine(); // Correction
  }
}

// ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░ 1.2. Mode Positioning Coarse
void positioning_Fine()
{ // For now nothing
  Serial1.println("Mode Positioning Fine");

  bool waiting = true;

  Deg_to_reach = OBC_data_value + Yaw_deg; // Get value to reach, contained in 360
  if (Deg_to_reach < 0)
  {
    Deg_to_reach += 360;
    Zero_state = true; // Used to store if it passes 0.
    // A PD passing through 0 could give great problems, as it has a very big change in value. Further used in computePID()
  }
  else if (Deg_to_reach >= 360)
  {
    Deg_to_reach -= 360; // Estas dos lineas contienen el valor del Yaw_deg en 0-360 grados.
    Zero_state = true;
  }
  else
  {
    Zero_state = false;
  }

  Timer1.attachInterrupt(TIMER_CH4, computePID);
  while (waiting)
  { // Range of tolerancy
    if (abs(IMU_gyro_data_Z) > Gyro_tolerancy && abs(IMU_accel_data_X) < Accel_tolerancy)
    { // we consider it is stopped, modify values to be acurate
      waiting = false;
    }
    delay(1); // Change
  }
  waiting = true;
  Timer1.detachInterrupt(TIMER_CH4);

  // At exit, RW_speed could not be 0
  OBC_data_value = 0;
  Serial1.println("End of Manouver");
  mode_Select(OBC_mode_value);
}

// ██████████████████████████████████████████████████████████████████████ VOID SETUP
void setup()
{
  delay(1000); //  wait to let open the serial
  // ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Timers
  Timer1.pause();
  Timer1.setPrescaleFactor(7200); // 72MHz Clock / 7200 = 10KHz timer
  Timer1.setOverflow(1000);       // Overflow occurs at 1000, each 100 ms timer restarts

  Timer1.setMode(TIMER_CH3, TIMER_OUTPUT_COMPARE); // Configure channel to OUTPURCOMPARE: Channel for OBC read values
  Timer1.setMode(TIMER_CH4, TIMER_OUTPUT_COMPARE); // Channel for IMU read values
  Timer1.setCompare(TIMER_CH3, 1);                 // Phase value in Overflow range
  Timer1.setCompare(TIMER_CH4, 1);

  Timer1.refresh(); // Refresh timer and start over
  Timer1.resume();

  // ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Initiations
  Serial1.begin(9600);
  SPI.begin();

  Serial1.println("START");

  // ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Setups

  pinMode(LEDPIN, OUTPUT); // Integrated LED
  pinMode(CS1, OUTPUT);    // NCS Pin definition

  // ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Driver Setup
  Serial1.println("Configuring Driver...");
  DriverOne.settings.commInterface = I2C_MODE; // Driver Comm Mode
  DriverOne.settings.I2CAddress = 0x5D;        // Driver Adress (0x5D by Defalut)

  while (DriverOne.begin() != 0xA9)
  { // Driver wait for idle
    Serial1.println("ID Mismatch");
    delay(200);
  }
  Serial1.println("ID Match");

  Serial1.println("Waiting for enumeration"); // Driver wait for peripherals
  while (DriverOne.ready() == false)
    ;
  Serial1.println("Ready");

  while (DriverOne.busy())
    ; // Driver enable
  DriverOne.enable();
  Serial1.println("Driver Ready to Use!");

  // ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Setup
  Serial1.println("Configuring MPU9250...");
  int status = IMU.begin();
  if (status < 0)
  {
    Serial1.println("IMU initialization unsuccessful");
    Serial1.println("Check IMU wiring or try cycling power");
    Serial1.print("Status: ");
    Serial1.println(status);
    while (1)
    {
    }
  }

  IMU.setGyroRange(IMU.GYRO_RANGE_500DPS);

  // ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Calibration
  // Use for getting the values on calibration setting below

  //  Para calibrar el magnetometro o la brujula se comenta todo excepto las 3 ultimas lineas.
  //  Se mueve en forma de ocho aproximadamente unos 2 min. Se recomienda realizar varias veces hasta afinar las medidas.
  //  Se descomenta todo y se pone los valores obtenidos del MagBias y ScaleFactor para visualizar los resultados del magnetometro.

  // IMU.calibrateMag();
  //  Serial.println("Done");
  //
  //   Serial.print(IMU.getMagBiasX_uT());
  //   Serial.print(",");
  //   Serial.print(IMU.getMagBiasY_uT());
  //   Serial.print(",");
  //   Serial.println(IMU.getMagBiasZ_uT());
  //
  //   Serial.print(IMU.getMagScaleFactorX());
  //   Serial.print(",");
  //   Serial.print(IMU.getMagScaleFactorY());
  //   Serial.print(",");
  //   Serial.println(IMU.getMagScaleFactorZ());

  IMU.setMagCalX(7.59, 1.04); //  El primer valor corresponde al MagBias, y el segundo el ScaleFactor.
  IMU.setMagCalY(10.29, 0.91);
  IMU.setMagCalZ(-22.11, 1.07);
}
IMU.setMagCalX(IMU.getMagBiasX_uT(), IMU.getMagScaleFactorX()); // Primer valor el MagBias, y el segundo el ScaleFactor!
IMU.setMagCalY(IMU.getMagBiasY_uT(), IMU.getMagScaleFactorY());
IMU.setMagCalZ(IMU.getMagBiasZ_uT(), IMU.getMagScaleFactorZ());

Serial1.println("MPU9250 Ready to Use!");

Serial1.println("Reading mode from OBC");
mode_OBC_Imput_Wait();
}

// ██████████████████████████████████████████████████████████████████████ VOID LOOP
void loop()
{
}
