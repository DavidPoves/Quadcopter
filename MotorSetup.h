#include <Servo.h>

// Define the Motor Objects as Servo Objects.
/*
Despite these engines are brushless dc, with this library we can
handle the esc by sending pwm pulses, which are then used by the
electronic speed controller.
*/
Servo Motor1, Motor2, Motor3, Motor4;

// Define minimum and maximum desired pulse lengths.
const int MIN_PULSE_LENGTH =  1000; // Minimum pulse length in µs
const int MAX_PULSE_LENGTH = 2000; // Maximum pulse length in µs
#define MOTOR_CALIBRATION false  // Set if motors calibration is required.

// Let compiler know that pins' variables are coming from external file.
extern int pin_Motor1; extern int pin_Motor2; extern int pin_Motor3;
extern int pin_Motor4;

/*
 * Define the security delay, which is defined as the time ellapsed
 * between the arming sequence and the beginning of the rotation of
 * the motors.
 */
unsigned int security_delay = 5000;  // [miliseconds].

// Define the delay between the beginning of rotation between motors.
unsigned int start_rotation_delay = 1500;  // [miliseconds].

// Define calibration variables.
char option;
boolean finish_calibration = false;

/**************************************************************************/
/*
    Define functions for the arming sequence and to start
                          minimum rotation.
    */
/**************************************************************************/

void attachMotors(){
    // Attach motors to corresponding pins.
    Motor1.attach(pin_Motor1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    Motor2.attach(pin_Motor2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    Motor3.attach(pin_Motor3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    Motor4.attach(pin_Motor4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
}

void runMotors(int pulse_length){
    Motor1.writeMicroseconds(pulse_length);
    Motor2.writeMicroseconds(pulse_length);
    Motor3.writeMicroseconds(pulse_length);
    Motor4.writeMicroseconds(pulse_length);
}

void displayCalibrationInstructions(){
  Serial.println("Starting motors calibration process...");
  Serial.println("WARNING: READ INSTRUCTIONS BELOW BEFORE SENDING ANY COMMAND!");
  Serial.println("-----------------------------------------------------------------------------------------------------------------------------------------------");
  Serial.println("\t1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode.");
  Serial.println("\t2. Power up your ESCs. You must hear tones meaning the power supply is OK.");
  Serial.println("\t3. After 2sec, two tones may emit (depends on esc), meaning the throttle highest point has been correctly confirmed.");
  Serial.println("\t4. Type 0 to send min throttle.");
  Serial.println("\t5. Several beep tones emits, which means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo).");
  Serial.println("\t6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed.");
  Serial.println("\t7. Type 2 to launch test function. This will send min to max throttle to ESCs to test them");
  Serial.println("-----------------------------------------------------------------------------------------------------------------------------------------------");
  Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
  Serial.println("\t0 : Send min throttle");
  Serial.println("\t1 : Send max throttle");
  Serial.println("\t2 : Run test function\n");
}

void MotorCalibration(int min_pulse, int max_pulse){
  /*
   * This function will guide the user along the calibration process. This function
   * becomes handy when maybe different ESCs are used for the motors, each of them
   * having their own default calibration. Hence, following the steps indicated
   * within the calibration instructions, the user will set the same operating
   * range for all the ESCs.
   * Inputs are an integer corresponding to the minimum pulse width of the range and
   * the second input is the same but for the maximum width of the range.
   * Units are MICROSECONDS.
   */
  
   displayCalibrationInstructions();

   while (!finish_calibration){

     if (Serial.available()){
      option = Serial.read();
      
      switch (option){
        // Option 0.
        case 48: Serial.println("Sending minimum throttle instruction...");
                 runMotors(MIN_PULSE_LENGTH);
        break;
  
        // Option 1.
        case 49: Serial.println("Sending maximum throttle instruction...");
                 runMotors(MAX_PULSE_LENGTH);
        break;
  
        // Option 2.
        case 50: Serial.print("Running test in 5");
                 delay(1000);
                 Serial.print(" 4");
                 delay(1000);
                 Serial.print(" 3");
                 delay(1000);
                 Serial.print(" 2");
                 delay(1000);
                 Serial.println(" 1");
                 delay(1000);
                 Serial.println("Starting test!");
                 for (int i = MIN_PULSE_LENGTH; i<=MAX_PULSE_LENGTH; i+=5){
                  Serial.print("Pulse Width: ");
                  Serial.println(i);
                  runMotors(i);
                  delay(200);
                 }
                 finish_calibration = true;
                 Serial.println("Test finished! Going back to main program...");
                 runMotors(MIN_PULSE_LENGTH);
        break;
      }
     }
   }
}

void MotorSetup() {
  Serial.println("                                                            MOTOR SETUP SEQUENCE                                                               ");
  Serial.println("-----------------------------------------------------------------------------------------------------------------------------------------------");
  // Run setup of ESC.
  attachMotors();  // Attach the motors to the corresponding pins.

  // Check if calibration of motors is required.
  if (MOTOR_CALIBRATION){
    MotorCalibration(MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);  // Calibrate all motors.
  }
  else{
    Serial.println("No calibration required! Arming motors...");
    runMotors(MIN_PULSE_LENGTH);
  }

  Serial.println("-----------------------------------------------------------------------------------------------------------------------------------------------");

  // Introduce a security delay.
  delay(security_delay);

}

void setIdle() {
  // Arm motors to run at minimum pulse width.
  Motor1.writeMicroseconds(MIN_PULSE_LENGTH + 65);
  delay(start_rotation_delay);
  Motor2.writeMicroseconds(MIN_PULSE_LENGTH + 65);
  delay(start_rotation_delay);
  Motor3.writeMicroseconds(MIN_PULSE_LENGTH + 65);
  delay(start_rotation_delay);
  Motor4.writeMicroseconds(MIN_PULSE_LENGTH + 65);
  delay(start_rotation_delay);
}
