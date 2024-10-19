// Sub Function
void Rate_calibration() {
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
}

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // Define and adjust these parameters
  float dt = 0.004;  // Time step (in seconds), keep this if your loop runs at 250Hz
  float Q = 0.01;    // Process noise covariance, adjust this value. Increase this value if the filter responds too slowly to changes.
  float R = 0.1;     // Measurement noise covariance, adjust this value Increase this value if the output is too noisy.

  // Predict
  KalmanState = KalmanState + dt * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + dt * dt * Q;

  // Update
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + R);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  // Store the results
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void read_accel(){
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(AccX, AccY, AccZ);
    // Calibrate the Acceleration
    AccX=AccX+0.02;
    AccY=-AccY-0.01;  // add minus to change right-handed system
    AccZ=AccZ-0.02;
    // Serial.print("AccX [g] ");
    // Serial.print(AccX);
    // Serial.print('\t');
    // Serial.print("AccY [g] ");
    // Serial.print(AccY);
    // Serial.print('\t');
    // Serial.print("AccZ [g] ");
    // Serial.println(AccZ);
  }
}
void angle_by_accel(){
  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*180/3.142;
  AnglePitch = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*180/3.142; 
  // Serial.print("AngleRoll [deg] ");
  // Serial.print(AngleRoll);
  // Serial.print('\t');
  // Serial.print("AnglePitch [deg] ");
  // Serial.println(AnglePitch);
}

void calibrated_rate(){
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(RateRoll, RatePitch, RateYaw);
    RateRoll = - (RateRoll - RateCaliRoll);
    RatePitch-=RateCaliPitch;
    RateYaw = - (RateYaw - RateCaliYaw);
    // Serial.print("RateROll [deg/s]");
    // Serial.print(RateRoll);
    // Serial.print('\t');
    // Serial.print("RatePitch [deg/s]");
    // Serial.print(RatePitch);
    // Serial.print('\t');
    // Serial.print("RateYaw [deg/s]");
    // Serial.println(RateYaw);
  }
}



