// Load the necessary library.
#include <PID_v2.h>;

// Pitch and Roll PID parameters.

// Pitch and roll conservative and aggressive PID constants.
double aggKp = 1.2, aggKi = 0.045, aggKd = 18;
double consKp = 1., consKi = 0.04, consKd = 10;

// Yaw PID constants.
double consKp_yaw = 3.0, consKi_yaw = 0.02, consKd_yaw = 0;

// Thrust PID constants.
double consKp_thrust = 1.50, consKi_thrust = 0.035, consKd_thrust = 10;

// Define output limits for the pitch and roll PIDs ({min, max}).
int pitch_limits[2] = {-200, 200}, roll_limits[2] = {-200, 200};

// Define the limit (in degrees) at which aggressive or conservative PID constants may be used.
/*
 * Within the controller, a gap is computed between the desired angle and its current value.
 * If that gap is bigger than the defined limit, aggressive constants may be used to restore
 * equilibrium faster. Otherwise, conservative values will be used.
 */
 int aggressive_limit_angles = 10;  // [degrees].

// Create PID controllers for the pitch and roll.
PID_v2 pitchPID(consKp, consKi, consKd, PID::Direct);
PID_v2 rollPID(consKp, consKi, consKd, PID::Direct);
PID_v2 yawPID(consKp_yaw, consKi_yaw, consKd_yaw, PID::Direct);
PID_v2 thrustPID(consKp_thrust, consKi_thrust, consKd_thrust, PID::Direct);

void initializePitchPID(float pitch_angle){
  // Initialize the PID controller for the pitch.
  /*
   * In order to initialize the PID, the Start method from the PID lib
   * is used. The first input is the variable to be controlled, the
   * second is the current output and the third one is the setpoint
   * (the objective to be achieved by the PID controller).
   */
  pitchPID.Start(pitch_angle, 0, 0);
  pitchPID.SetOutputLimits(pitch_limits[0], pitch_limits[1]);
}

void initializeRollPID(float roll_angle){
  // Initialize the PID controller for the roll.
  /*
   * In order to initialize the PID, the Start method from the PID lib
   * is used. The first input is the variable to be controlled, the
   * second is the current output and the third one is the setpoint
   * (the objective to be achieved by the PID controller).
   */
  rollPID.Start(roll_angle, 0, 0);
  rollPID.SetOutputLimits(roll_limits[0], roll_limits[1]);
}

void initializeYawPID(float yaw_angle){
  // Initialize the PID controller for the roll.
  /*
   * In order to initialize the PID, the Start method from the PID lib
   * is used. The first input is the variable to be controlled, the
   * second is the current output and the third one is the setpoint
   * (the objective to be achieved by the PID controller).
   */
  yawPID.Start(yaw_angle, 0, yaw_angle);
  yawPID.SetOutputLimits(roll_limits[0], roll_limits[1]);
}

void initializeThrustPID(float height, int max_height){
  // Initialize the PID controller for the roll.
  /*
   * In order to initialize the PID, the Start method from the PID lib
   * is used. The first input is the variable to be controlled, the
   * second is the current output and the third one is the setpoint
   * (the objective to be achieved by the PID controller).
   */
  thrustPID.Start(height, 0, max_height);
  thrustPID.SetOutputLimits(roll_limits[0], roll_limits[1]);
}

int runPitchPID(float pitch_angle){
  // Compute the gap between the desired pitch angle and its current value.
  float gap = abs(pitchPID.GetSetpoint() - pitch_angle);

  /*
   * Depending on the existing gap, the PID controller may use aggressive or conservative
   * PID constants.
   */
   if (gap >= aggressive_limit_angles){
    pitchPID.SetTunings(aggKp, aggKi, aggKd);
   }
   else {
    pitchPID.SetTunings(consKp, consKi, consKd);
   }

   // Run the PID.
   const int output = pitchPID.Run(pitch_angle);

   return output;
}

int runRollPID(float roll_angle){
  // Compute the gap between the desired roll angle and its current value.
  float gap = abs(rollPID.GetSetpoint() - roll_angle);

  /*
   * Depending on the existing gap, the PID controller may use aggressive or conservative
   * PID constants.
   */
   if (gap >= aggressive_limit_angles){
    rollPID.SetTunings(aggKp, aggKi, aggKd);
   }
   else {
    rollPID.SetTunings(consKp, consKi, consKd);
   }

   // Run the PID.
   const int output = rollPID.Run(roll_angle);

   return output;
}

int runYawPID(float yaw_angle){
  
    yawPID.SetTunings(consKp_yaw, consKi_yaw, consKd_yaw);

   // Run the PID.
   const int output = yawPID.Run(yaw_angle);

   return output;
}

int runThrustPID(float height){
    thrustPID.SetTunings(consKp, consKi, consKd);

   // Run the PID.
   const int output = thrustPID.Run(height);

   return output;
}
