/* 
 *  PID Control
*/
// Libraries
#include <Arduino.h>     // Arduino library
#include <time.h>        // Time library
#include <SCMD.h>        // Driver library
#include <SCMD_config.h> // SCMD configuration library
#include <MPU9250.h>     //IMU library

//PID constants
double kp = 3; // Proportional contribution
double ki = 4; // Integral contribution
double kd = 2; // Derivative contribution

// Parameters
unsigned long currentTime, previousTime; // Initialize current time and previous time as unsigned long
double elapsedTime;                      // Initialize elapsed time = previousTime - currentTime
double error, lastError;                 // Initialize error and previousError
double input, output, setPoint;          // Initialize input variable from IMU, the output variable, and the desired setPoint
double cumError, rateError;              // Initialize the cumulative Error (Integral) and the rate of Error (Derivative)

char inputData = 0; // Bluetooth input data

// Main setup
void setup()
{
    Serial1.begin(9600); // Initialize UART port for Bluetooth communication
    Serial1.println("Fine Control stablished");
    setPoint = 0; // Desired angle to turn
}

// Main loop
void loop()
{
    // input = dmp.imu etc // Read angle from IMU

    // Check if serial is available
    if (Serial1.available() > 0)
    {
        inputData = Serial1.read();
    }

    // TODO: check how input data is formatted (if direction + angle or just angle)

    // Compute PID control
    output = computePID(inputData);

    // Delay 100 ms
    delay(100);

    // Fine control (motor, direction (0,1), strength)
    DriverOne.setDrive(0, 0, output);
}

// PID function
double computePID(double inp)
{
    // Time
    currentTime = millis();                             // Get current time
    elapsedTime = (double)(currentTime - previousTime); // Compute elapsed time from previous time computation

    // Errors
    error = setPoint - inp;                        // Calculate error (Proportional)
    cumError += error * elapsedTime;               // Calculate the cumulative error (Integral)
    rateError = (error - lastError) / elapsedTime; // Calculate the rate of error (Derivative)

    // PID Control
    PID_P = kp * error;     // Proportional
    PID_I = ki * cumError;  // Integral
    PID_D = kd * rateError; // Derivative

    double PID_output = PID_P + PID_I + PID_D; // PID control

    // Save current error and time for next iteration
    lastError = error;          // Save current error
    previousTime = currentTime; // Save current time

    // PID control return
    return PID_output;
}
