#include <Arduino_LSM6DS3.h>
float GyroX, GyroY, GyroZ;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float RateRoll, RatePitch, RateYaw;
float RateCaliRoll, RateCaliPitch, RateCaliYaw; // Calibrate the rotation rate
int RateCaliNum;
uint32_t LoopTimer;

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;  // Define the predicted of the initial angles and the uncertainties (std dev) on the initial angle
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0}; //Initialize the output of the filter (angle prediction, uncertainty of the prediction)

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // KalmanInput = rotation rate, KalmanMeasurement = accelerometer angle
  // KalmanState = angle calculated with the Kalman filter
  KalmanState = KalmanState + 0.004 * KalmanInput;  // Predict the current state of the system
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;  // Calculate the uncertainty of the prediction
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3); // Calculate the Kalman gain from the uncertainties on the predictions and measurements
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState); // Update the predicted state of the system with the measurement of the state through the Kalman gain
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty; // Update the uncertainty of the predicted state
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}
void gyro_signals(){
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);
    AccX=AccX+0.03; // Calibrate the Acceleration
    AccY=AccY+0.02;
    AccZ=AccZ-0.02;
    AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*180/3.142;
    AnglePitch = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*180/3.142; 
    // Serial.print("AccX [g's]= ");
    // Serial.print(AccX);
    // Serial.print(" AccY [g's]= ");
    // Serial.print(AccY);
    // Serial.print(" AccZ [g's]= ");
    // Serial.print(AccZ);
    // Serial.print(" AngleRoll [deg]= ");
    // Serial.print(AngleRoll);
    // Serial.print(" AnglePitch [deg]= ");
    // Serial.println(AnglePitch);
    // delay(500);
  }
}
void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");

    while (1);
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");

  for(RateCaliNum=0;  //Peform the calibration measurements
      RateCaliNum<2000;
      RateCaliNum++){
        IMU.readGyroscope(RateRoll, RatePitch, RateYaw);
        RateCaliRoll+=RateRoll;
        RateCaliPitch+=RatePitch;
        RateCaliYaw+=RateYaw;
        delay(1);
  }
  RateCaliRoll/=2000; // calcurate the calibration values
  RateCaliPitch/=2000;
  RateCaliYaw/=2000;
  LoopTimer=micros();

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");

}

void loop() {  
  gyro_signals();
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(RateRoll, RatePitch, RateYaw);
    RateRoll-=RateCaliRoll;
    RatePitch-=RateCaliPitch;
    RateYaw-=RateCaliYaw;
  }
  // Start of the iteration for the Kalman filter with the roll and pitch angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
    Serial.print("Roll Angle [deg] ");
    Serial.print(KalmanAngleRoll);
    Serial.print(" Pitch Angle [deg] ");
    Serial.println(KalmanAnglePitch);
    while(micros() - LoopTimer < 4000);
    LoopTimer=micros();
}