/* Reaction Wheel controller
 *
 * The following code calibrates the IMU.
 *
 * 
 * The following libraries are used:
 * STM32 Bluepill microcontroller package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
 * SCMD Driver library by SparkFun used (https:// github.com/sparkfun/SparkFun_Serial1_Controlled_Motor_Driver_Arduino_Library)
 * MPU9250 IMU library by Rafa Castalla used (https://github.com/rafacastalla/MPU9250-1)
 * read_IMU() and IMU setup content code extracted from Andres Gomez and Miquel Reurer, from the PLATHON group (magnetorquers section)
 */

// ██████████████████████████████████████████████████████████████████████ DEFINITIONS
// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ LIBRARIES
#include <Arduino.h>     // Arduino library
#include <stdint.h>      // Integer types library
#include <SCMD.h>        // Serial Controlled Motor Driver library
#include <SCMD_config.h> // Serial Controlled Motor Driver Configuration library
#include <SPI.h>         // SPI library
#include <MPU9250.h>     // IMU library
#include <Wire.h>

// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ DEFINITIONS
#define LEDPIN PC13                // Integrated LED of the Bluepill
#define CS1 4                      // STM32 NCS Chip select (SPI mode only) pin. MOSI on PA7, MISO on PA6 and CLK on PA5 by default on STM32
#define STM32_CLOCK 72000          // Internal STM32 Clock (72MHz, in kHz so period is millisec)
#define PI 3.14159265              // Number PI
#define Rad_to_deg 57.29577951     // Convert radians to degrees
#define Deg_to_rad 0.01745329      // Convert degrees to radians
#define Final_pointing_tolerance 1 // Tolerance for final pointing (+-1 degree of tolerance) (Check if position is within the margin)
#define Pointing_mode_tolerance 5  // Tolerance for selecting pointing mode (+-5 degree of tolerance) (Select whether to use coarse or fine mode)
#define Accel_tolerance 0.5        // Tolerance for acceleration measurement (+-0.5 unit of acceleration of tolerance) (Acceptable error of acceleration)
#define Gyro_tolerance 1           // Tolerance for gyro measurement (+-1 unit of gyroscope Z tolerance) (Accounts to know if it is rotating or not at a constant speed for the ramp)

SCMD DriverOne;        // Driver Object definition
MPU9250 IMU(SPI, CS1); // MPU object definition

// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GLOBAL VARIABLES
float IMU_accel_data_X, IMU_accel_data_Y, IMU_accel_data_Z; // Values of local IMU accelerometer (m/s^2)
float IMU_gyro_data_X, IMU_gyro_data_Y, IMU_gyro_data_Z;    // Values of local IMU gyroscope (degrees/s)
float IMU_mag_data_X, IMU_mag_data_Y, IMU_mag_data_Z;       // Values of local IMU magnetometer (uT)
float Pitch_deg, Roll_deg, Yaw_deg = 0;                     // Pitch, Roll and Yaw (degrees)

// Declare variables outside IMU, since variables initiates to 0 without defining them in function
float Accel_total_vector_modulus;       // Accelerometer total vector magnitude (m/s^2)
float Accel_pitch_deg, Accel_roll_deg;  // Accelerometer pitch and roll angles (degrees)
float Gyro_pitch_deg, Gyro_roll_deg;    // Gyroscope pitch and roll angles (degrees)
bool Gyro_Accel_sync = false;           // Check if Gyroscope and Accelerometer are synchronized
float Mag_X_calc, Mag_Y_calc;           // Calculated Magnetometer value (uT)
float Mag_X_corrected, Mag_Y_corrected; // Corrected value Magnetometer value (with damping) (uT)
float Pitch_rad, Roll_rad;              // Pitch and Roll angles (rad)

float deg_offset = 0;
float spd_offset = 0;
float acc_offset = 0;

int RW_speed = 0;       // Value of Reaction Wheel speed [from -255 to 255]
int OBC_mode_value = 0; // Value of On Board Computer mode (waiting, positioning, detumbling, etc.)
int OBC_data_value = 0; // Value of On Board Computer data (desired angle turn, etc.)
int Stop_state = 0;     // If Stop_state != 0, motor stops rotating

// PID control definitions
double PID_error, PID_last_error;            // Initialize error and previousError
double PID_cumulative_error, PID_rate_error; // Initialize the cumulative Error (Integral) and the rate of Error (Derivative)
float Current_Yaw_deg = 0;                   // Addition of 180 deg to all values. The error is a substract, so the difference is the same

double kp = 3; // Proportional contribution
double ki = 4; // Integral contribution
double kd = 0; // Derivative contribution

float Deg_to_reach = 0;  // Setpoint angle (degrees)
bool Zero_state = false; // Check if PID encompass yaw of 0 degrees (to turn the value to a workable zone)
int PID_output = 0;      // Output value of the PID [from 0 to 255]


void WAITFORINPUT(){            
  while(!Serial1.available()){};  
  while(Serial1.available()){     
    Serial1.read();            
  };                            
}           


void read_IMU()
{
  /*
    Function to read IMU_Data (and refreshes the global variables for IMU data).

    INPUT: 
    None
    OUTPUT: 
    None, but saves IMU variables and Roll, Pitch and Yaw
  */

  // Begin reading IMU and pitch, yaw, roll
  IMU.readSensor();

  // Accelerations
  IMU_accel_data_X = IMU.getAccelX_mss();
  IMU_accel_data_Y = IMU.getAccelY_mss();
  IMU_accel_data_Z = IMU.getAccelZ_mss();
  // Gyroscope
  IMU_gyro_data_X = (IMU.getGyroX_rads() * Rad_to_deg);
  IMU_gyro_data_Y = (IMU.getGyroY_rads() * Rad_to_deg);
  IMU_gyro_data_Z = (IMU.getGyroZ_rads() * Rad_to_deg);
  //Magnetometer
  IMU_mag_data_X = IMU.getMagX_uT();
  IMU_mag_data_Y = IMU.getMagY_uT();
  IMU_mag_data_Z = IMU.getMagZ_uT();

  // GetTime of IRS, in case value is correct if timer definition changes.
  // The dT is the time step of each interrupt, thus, each read of the IMU
  // First division of clock by preescaler (frequency of timer), then inversion (period of timer), then multiplication by overflow (period of channel)
  float dT = 1; // dT in seconds

  // Get the angle Pitch and Roll angle(w = w0 + dT*angular_vel)
  Gyro_pitch_deg += dT * (IMU.getGyroY_rads()) * Rad_to_deg;
  Gyro_roll_deg += dT * (IMU.getGyroX_rads()) * Rad_to_deg;

  // Gimbal lock compensation
  Gyro_pitch_deg = Gyro_pitch_deg + Gyro_roll_deg * sin(dT * (IMU.getGyroZ_rads()));
  Gyro_roll_deg = Gyro_roll_deg - Gyro_pitch_deg * sin(dT * (IMU.getGyroZ_rads()));

  // Modulus of the acceleration vector, calculate the total (3D) vector
  Accel_total_vector_modulus = sqrt((IMU.getAccelX_mss() * IMU.getAccelX_mss()) + (IMU.getAccelY_mss() * IMU.getAccelY_mss()) + (IMU.getAccelZ_mss() * IMU.getAccelZ_mss()));
  Accel_pitch_deg = asin((float)IMU.getAccelX_mss() / Accel_total_vector_modulus) * Rad_to_deg; // Calculate the pitch angle from accelerometer
  Accel_roll_deg = asin((float)IMU.getAccelY_mss() / Accel_total_vector_modulus) * Rad_to_deg;  // Calculate the roll angle from accelerometer

  /* Accelerometer calibration */

  // To calibrate the accelerometer, put the values ​​0.0 in the variables and uncomment the 3 Serial1.prints
  // When compiling, leave the IMU immobile so that the accelerometer calibrates properly.
  // Once the values ​​are obtained, they are noted and it is recompiled as it had been before.

  Accel_pitch_deg -= -0.49;
  Accel_roll_deg -= 2.08;

        Serial1.print(Accel_pitch_deg,6);
        Serial1.print("\t");
        Serial1.println(Accel_roll_deg,6);

  // If gyroscope and accelerometer are synchronized
  if (Gyro_Accel_sync)
  {
    //  ----- Gyro & accel have been synchronized
    Gyro_pitch_deg = Gyro_pitch_deg * 0.95 + Accel_pitch_deg * 0.05; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle. Original values were 0.9996 and 0.0004m, respectively.
    Gyro_roll_deg = Gyro_roll_deg * 0.95 + Accel_roll_deg * 0.05;    // Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  { //  The 0.98 and 0.02 values are taken from http:// www.pieter-jan.com/node/11else{
    //  ----- Synchronize gyro & accel
    Gyro_pitch_deg = Accel_pitch_deg; // Set the gyro pitch angle equal to the accelerometer pitch angle
    Gyro_roll_deg = Accel_roll_deg;   // Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_Accel_sync = true;           // Set the IMU started flag
  }

  // Final corrected Pitch and Roll angle (degrees)
  Pitch_deg = Pitch_deg * 0.9 + Gyro_pitch_deg * 0.1;
  Roll_deg = Roll_deg * 0.9 + Gyro_roll_deg * 0.1;

  // Final corrected Pitch and Roll angle (radians)
  Pitch_rad = -Roll_deg * Deg_to_rad;
  Roll_rad = Pitch_deg * Deg_to_rad;

  // Calculated values of Magnetometer along X and Y axis
  Mag_X_calc = IMU.getMagX_uT() * cos(Pitch_rad) + IMU.getMagY_uT() * sin(Roll_rad) * sin(Pitch_rad) - IMU.getMagZ_uT() * cos(Roll_rad) * sin(Pitch_rad);
  Mag_Y_calc = IMU.getMagY_uT() * cos(Roll_rad) + IMU.getMagZ_uT() * sin(Roll_rad);

  // Correct drif (same as Pitch_deg)
  Mag_X_corrected = Mag_X_corrected * 0.9 + Mag_X_calc * 0.1;
  Mag_Y_corrected = Mag_Y_corrected * 0.9 + Mag_Y_calc * 0.1;

  // The magnetic north has been modified, originally, it was from X to Y (towards which the reading 0 was 90 degrees offset) and without the negative sign of the beginning (towards which the reading would add the angles counterclockwise instead of clockwise).

  Yaw_deg = -atan2(Mag_Y_corrected, Mag_X_corrected) * Rad_to_deg;

  // If you want to be precise, add the declination of the geographical place where you are (https: // www.ign.es/web/gmt-declinacion-magnetica)
  // Yaw_deg += Declination;

  // The next two lines contain the value of Yaw_deg between 0 and 360 degrees.
  if (Yaw_deg < 0)
    Yaw_deg += 360;
  if (Yaw_deg >= 360)
    Yaw_deg -= 360;

  // To initate at zero
  Yaw_deg = Yaw_deg - deg_offset;
}

void show_IMU()
{
  /*
    Function to show IMU_Data on the Serial1 Monitor

    INPUT: 
    None
    OUTPUT: 
    None
  */

  // Print accelerometer values
  //  Serial1.print("Acc: ");
  //  Serial1.print(IMU_accel_data_X, 4);
  //  Serial1.print('\t');
  //  Serial1.print(IMU_accel_data_Y, 4);
  //  Serial1.print('\t');
  // Serial1.print(IMU_accel_data_Z,4);
  // Serial1.println("");

  // Print Gyro values
  // Serial1.print(" / G: ");
  // Serial1.print(IMU_gyro_data_X,4);  Serial1.print('\t');
  // Serial1.print(IMU_gyro_data_Y,4);  Serial1.print('\t');
  // Serial1.print(IMU_gyro_data_Z,4);   Serial1.print('\t');
  // Serial1.println("");

  // Print Mag values
  //   Serial1.print("MAGS: ");
  //   Serial1.print(IMU_mag_data_X,4);    Serial1.print(';');
  //   Serial1.print(IMU_mag_data_Y,4);    Serial1.print(';');
  //   Serial1.print(IMU_mag_data_Z,4);
  //   Serial1.println("");

  // Print orientation angles
  Serial1.print(" / D: ");
  Serial1.print(Pitch_deg, 4);
  Serial1.print(';');
  Serial1.print(Roll_deg, 4);
  Serial1.print(';');
  Serial1.print(Yaw_deg, 4);
  Serial1.println("");
}
















void setup() {
  delay(1000); //  wait to let open the serial

  // ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Initiations
  Serial1.begin(9600);
  SPI.begin();

  Serial1.println("START");

  // ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Setups

  pinMode(LEDPIN, OUTPUT); // Integrated LED
  pinMode(CS1, OUTPUT);    // NCS Pin definition

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
  // Use for getting the values ​​on calibration setting below

  // To calibrate the magnetometer or the compass, everything is commented except the last 3 lines.
  // It moves in the shape of an eight approximately 2 min. It is recommended to carry out several times until the measurements are fine-tuned.
  // Uncomment everything and enter the values ​​obtained from the MagBias and ScaleFactor to view the results of the magnetometer.
  //
//      IMU.calibrateMag();
//      Serial1.println("Done");
//  
//      Serial1.print(IMU.getMagBiasX_uT());
//      Serial1.print(",");
//      Serial1.print(IMU.getMagBiasY_uT());
//      Serial1.print(",");
//      Serial1.println(IMU.getMagBiasZ_uT());
//  
//      Serial1.print(IMU.getMagScaleFactorX());
//      Serial1.print(",");
//      Serial1.print(IMU.getMagScaleFactorY());
//      Serial1.print(",");
//      Serial1.println(IMU.getMagScaleFactorZ());

  IMU.setMagCalX(8.89, 2.83); // The first value corresponds to the MagBias, and the second the ScaleFactor.
  IMU.setMagCalY(11.67, 0.69);
  IMU.setMagCalZ(-24.10, 0.83);

  Serial1.println("MPU9250 Ready to Use!");

  Serial1.println("Reading mode from OBC");

  WAITFORINPUT();

}

void loop() {
  read_IMU();
  // show_IMU();
  delay(100);

}
