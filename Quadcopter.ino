/*
MIT License

Copyright (c) 2020 David Poves, Iván Megía, Natalia Alonso, Daniel Pérez.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Define pin ports from Pins.h
#include "Pins.h"

// Include setup libraries.
#include "Motors.h"
#include "OpticalSensorSetup.h"
#include "IMU_QUAD.h"
#include "MeasureHeight.h";
#include "PID.h";

// Define degrees to radians conversions.
const float DEG2RAD = PI/180.0;

// Declare Euler angles.
float roll_angle = 20.0, initial_roll_angle = 0, pitch_angle = 20.0, initial_pitch_angle = 0;
float yaw_angle = -10000.0, initial_yaw_angle = 0;

// Declare Euler angle rates [rad/s];
float roll_rate = 20.0; float pitch_rate = 20.0;
float yaw_rate = -10000.0;

// Declare temperature and height variables to non-sense values.
int8_t temperature = 100; float height = 100000.0;

// Declare acceleration measurements to non-sense values.
float x_accel = -999.0; float y_accel = -999.0; float z_accel = -999.0;

// Define outputs from the pitch and roll PID controllers.
int output_pitch = 0, output_roll = 0, output_yaw = 0, output_thrust = 0;

// Define the maximum height at which the quadcopter can fly.
const int max_height = 10;  // [cm].
const int tolerable_height_gap = 1.5;  // [cm].

// Define PID thresholds.
const float pid_angle_threshold = 1.5;

// Define elapsed time.
int time_elapsed = 0;

// define the type of navigation to be used.
boolean dead_reckoning = true;


/*
 * This is the main script for the Quadcopter project. Before uploading the code,
 * read the instructions below:
 * 0. Modify pins at which motors, optical sensors and IMU will be connected in Pins.h
 * 1. If your motors require of calibration, change REQUIRE_CALIBRATION in MotorSetup.h
 *    to true.
 * 2. Either if your motors are calibrated or not, the user may change the
 *    minimum and maximum pulse widths as desired in MotorSetup.h by changing the
 *    values of MIN_PULSE_LENGTH and MAX_PULSE_LENGTH. If your motors are calibrated
 *    under some other values, change these variables as required. If your motors are
 *    not calibrated, these variables may be changed by user requirements.
 * 3. If calibration is required, follow the instructions given on screen. Otherwise,
 *    proceed with the execution.
 * 4. At this point, motors will start rotating in a given sequence at minimum
 *    throttle.
 * 5. Next, the IMU is initialized. At this point the serial monitor should already
 *    be opened to follow instructions.
 * 6. If no IMU calibration is found on Arduino's EEPROM, the user may follow the
 *    instructions given in:
 *    https://es.mathworks.com/help/supportpkg/arduinoio/ug/calibrate-sensors.html
 *    The code will automatically stop the calibration once the full process is done.
 *    The calibration profile will be saved. If a calibration profile is found,
 *    the IMU may require to slightly move the device to calibrate the magnetometer.
 * 7. Optical Sensors are initialized.
 */

void setup() {

  // Initialize the Serial Monitor.
  Serial.begin(9600);
  delay(1000);
  
  // Initialize the Motors.
  MotorSetup();

  // Initialize IMU.
  initializeIMU();

  delay(5000);

  // Get initial angles.
  getEulerAngles(initial_roll_angle, initial_pitch_angle, initial_yaw_angle);

  // Arm motors to run at minimum pulse width.
  setIdle();
  
  // Initialize the Optical Sensors.
  OpticalSensorSetup();

  // Initialize the pitch and roll PIDs.
  initializePitchPID(initial_pitch_angle);
  initializeRollPID(initial_roll_angle);
  initializeYawPID(initial_yaw_angle);
  initializeThrustPID(3, max_height);
  
}

void loop() {

  /**************************************************************************/
  /*
                    DEAD RECKONING IMPLEMENTATION.
      */
  /**************************************************************************/

  if (dead_reckoning){
    /**************************************************************************/
    /*
                      Get all information from the IMU.
        */
    /**************************************************************************/
    // Get Euler angles.
    getEulerAngles(roll_angle, pitch_angle, yaw_angle);
    pitch_angle -= initial_pitch_angle;
    roll_angle -= initial_roll_angle;
    yaw_angle -= initial_yaw_angle;
    Serial.print("Pitch Angle:\t");
    Serial.println(pitch_angle);
    Serial.print("Roll Angle:\t");
    Serial.println(roll_angle);
    Serial.print("Yaw Angle:\t");
    Serial.println(yaw_angle);
  
    if (abs(roll_angle) >= 25 || abs(pitch_angle) >= 25){
      stopMotors();
      Serial.println("For security reasons, motors were stopped!");
      delay(1e6);
    }
  
    if (abs(360-yaw_angle) < abs(yaw_angle)){
      yaw_angle -= 360;
    }
  
    // Get Angular velocities.
    getAngularVelocities(roll_rate, pitch_rate, yaw_rate);
  
    // Get Height in centimeters from the ultrasonic sensor.
    getTemperature(temperature);
    MeasureHeight(temperature, height);
  
    Serial.print("Height above ground\t");
    Serial.print(height*cos(DEG2RAD*pitch_angle)*cos(DEG2RAD*roll_angle));
    Serial.println("\t centimetres.");
    
  
    // Get acceleration data.
    getAcceleration(x_accel, y_accel, z_accel);
  
      /**************************************************************************/
    /*
                                      PID
        */
    /**************************************************************************/
    // Run pitch and roll PIDs.
    output_pitch = runPitchPID(pitch_angle);
    output_roll = runRollPID(roll_angle);
    output_yaw = runYawPID(yaw_angle);
    output_thrust = runThrustPID(height);
  
    // Decide which motors should be corrected by the sign of the pitch angle.
    if (pitch_angle > pid_angle_threshold){
      decreasePitch(output_pitch, pitch_angle);
    }
    else if (pitch_angle < -pid_angle_threshold){
      increasePitch(output_pitch, pitch_angle);
    }
  
    if (abs(height - max_height) >= tolerable_height_gap){
      if (height <= max_height){
        increaseThrust(output_thrust);
      }
      else {
        decreaseThrust(output_thrust);
      }
    }
  
    if (roll_angle > pid_angle_threshold){
      decreaseRoll(output_roll, roll_angle);
    }
    else if (roll_angle < -pid_angle_threshold){
      increaseRoll(output_roll, roll_angle);
    }
  
    if (yaw_angle > pid_angle_threshold){
      turnLeft(output_yaw);
    }
    else if (yaw_angle < -pid_angle_threshold){
      turnRight(output_yaw);
    }
  
    // Establish delay for new measurements.
    delay(BNO055_SAMPLERATE_DELAY_MS);

    // IMPLEMENTATION OF THE TRAJECTORY.
    if (time_elapsed < 4000){
      // Nothing to do here, since the previous code hovers the drone.
    }
    else if (4000 <= time_elapsed < 7000){
      // Move the drone forward for 3 seconds (3000ms).
      decreasePitch(output_pitch, pitch_angle);
    }
    else if (7000 <= time_elapsed <= 10000){
      // Turn drone to the right for 3 seconds.
      increaseRoll(output_roll, roll_angle);
    }
    else if (10000 <= time_elapsed <= 13000){
      // Move the drone forward for 3 seconds.
      decreasePitch(output_pitch, pitch_angle);
    }
    else if (13000 <= time_elapsed <= 16000){
      // Turn drone to the left for 3 seconds.
      decreaseRoll(output_roll, roll_angle);
    }
    else if (16000 <= time_elapsed <= 19000){
      // Move the drone forward for 3 seconds.
      decreasePitch(output_roll, roll_angle);
    }
    time_elapsed += BNO055_SAMPLERATE_DELAY_MS;
  }
}
