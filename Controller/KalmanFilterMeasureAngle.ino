#include <Arduino_LSM6DS3.h>
#include <Wire.h>  // Include the Wire library for I2C communication.
// #include <PulsePosition.h>  // Need to use "Teensy" H/W
float GyroX, GyroY, GyroZ;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateRoll, RatePitch, RateYaw;
float RateCaliRoll, RateCaliPitch, RateCaliYaw; // Calibrate the rotation rate
int RateCaliNum;
uint32_t LoopTimer; // Define the parameter containing the length of each control loop

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;  // Define the predicted of the initial angles and the uncertainties (std dev) on the initial angle
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0}; //Initialize the output of the filter (angle prediction, uncertainty of the prediction)

float AccZInertial;
float VelocityVertical;

void setup() {
  Serial.begin(9600);   // initialze the serial comm between microcontroller and a connected device at a baud rate of 9600 bps
  while (!Serial);

  if (!IMU.begin()) { // Initialize the IMU
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Rate_calibration();
  LoopTimer=micros(); // start the timer
}

void loop() {  
  read_accel();
  angle_by_accel(); // roll and pitch angles by accel
  calibrated_rate(); // roll, pitch, yaw rate
  // kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);  // Start of the iteration for the Kalman filter with the roll and pitch angles
  // KalmanAngleRoll=Kalman1DOutput[0];
  // KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  // kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  // KalmanAnglePitch=Kalman1DOutput[0];
  // KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  // Serial.print("Roll Angle [deg] ");
  // Serial.print(KalmanAngleRoll);
  // Serial.print(" Pitch Angle [deg] ");
  // Serial.println(KalmanAnglePitch);
  // // Calculate the acceleration in the inertial Z axis
  // AccZInertial = -sin(KalmanAnglePitch*(3.142/180))*AccX
  // + cos(KalmanAnglePitch*(3.142/180))*sin(KalmanAngleRoll*(3.142/180))*AccY
  // + cos(KalmanAnglePitch*(3.142/180))*cos(KalmanAngleRoll*(3.142/180))*AccZ;
  // AccZInertial = (AccZInertial -1)*9.81*100; // Convert the acceleration[g] to [cm/s2]
  // VelocityVertical = VelocityVertical + AccZInertial*0.004; // Calculate the vertical velocity
  // Serial.print("Vertical Velocity [cm/s]: ");
  // Serial.println(VelocityVertical);
  while(micros() - LoopTimer < 4000); // Finish the 250Hz control loop
  LoopTimer=micros();
  // Save KalmanAngleRoll and KalmanAnglePitch values to txt files
  //saveToTxt("KalmanAngleRoll.txt", KalmanAngleRoll);
  //saveToTxt("KalmanAnglePitch.txt", KalmanAnglePitch);
}

void saveToTxt(const char* filename, float value) {
  Serial.print(filename);
  Serial.print(": ");
  Serial.println(value);
}