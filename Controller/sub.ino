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
    // Serial.print("AccX [g] ");
    // Serial.print(AccX);
    // Serial.print("AccY [g] ");
    // Serial.print(AccY);
    // Serial.print("AccZ [g] ");
    // Serial.print(AccZ);
    AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*180/3.142;
    AnglePitch = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*180/3.142; 
  }
}

void calibrated_rate(){
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(RateRoll, RatePitch, RateYaw);
    RateRoll-=RateCaliRoll;
    RatePitch-=RateCaliPitch;
    RateYaw-=RateCaliYaw;
  }
}


