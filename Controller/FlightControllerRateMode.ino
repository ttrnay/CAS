/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics. 
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence. 
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics 
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity 
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by 
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware, 
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury. 
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  5 October 2022 -  initial release
*/


#include <Wire.h>  // Include the Wire library for I2C communication.
#include <Arduino_LSM6DS3.h>
#include <Arduino.h>
#include  <sam.h> //to get access to the correct register definitions for the SAMD21 microcontroller.
float RatePitch, RateRoll, RateYaw;  // Variables to store the gyroscope rates for pitch, roll, and yaw.
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;  // Variables for storing calibration offsets.
int RateCalibrationNumber;  // Counter for calibration iterations.
// Original PulsePosition include and object (commented out)
// #include <PulsePosition.h>
// PulsePositionInput ReceiverInput(RISING);
// New PPMReader include and object
#include <PPMReader.h>  // Include the PPMReader library instead of PulsePosition
#define PPM_INPUT_PIN 14
#define CHANNEL_AMOUNT 8
PPMReader ppm(PPM_INPUT_PIN, CHANNEL_AMOUNT);

float ReceiverValue[]={0, 0, 0, 0, 0, 0, 0, 0};  // Array to store receiver channel values.
int ChannelNumber=0;  // Variable to store the number of available channels.
float Voltage, Current, BatteryRemaining, BatteryAtStart;  // Variables for battery monitoring.
float CurrentConsumed=0;  // Track the total current consumed.
float BatteryDefault=1300;  // Default battery capacity in mAh.
uint32_t LoopTimer;  // Timer for controlling loop execution timing.
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;  // Desired rates for roll, pitch, and yaw.
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;  // Error between desired and actual rates.
float InputRoll, InputThrottle, InputPitch, InputYaw;  // Control inputs for motors.
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;  // Previous errors for PID control.
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;  // Previous integral terms for PID control.
float PIDReturn[]={0, 0, 0};  // Array to store PID output and state.
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;  // PID proportional gains.
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;  // PID integral gains.
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;  // PID derivative gains.
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;  // Motor control signals.
void battery_voltage(void) {
  Voltage=(float)analogRead(15)/62;  // Read and convert battery voltage.
  Current=(float)analogRead(21)*0.089;  // Read and convert current.
}
// Original read_receiver function (commented out)
/*
void read_receiver(void){
  ChannelNumber = ReceiverInput.available();
  if (ChannelNumber > 0) {
    for (int i=1; i<=ChannelNumber;i++){
      ReceiverValue[i-1]=ReceiverInput.read(i);
    }
  }
}
*/
void read_receiver(void){
  // Read all channels from the PPM signal
  for (int i = 0; i < CHANNEL_AMOUNT; i++) {
    ReceiverValue[i] = ppm.latestValidChannelValue(i + 1, 1500);  // Default to 1500 if no valid signal
  }
}
void gyro_signals(void) {
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(RateRoll, RatePitch, RateYaw);
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
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;  // Calculate proportional term.
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;  // Calculate integral term.
  if (Iterm > 400) Iterm=400;  // Limit integral term to prevent windup.
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;  // Calculate derivative term.
  float PIDOutput= Pterm+Iterm+Dterm;  // Sum PID terms to get output.
  if (PIDOutput>400) PIDOutput=400;  // Limit PID output.
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;  // Store PID output.
  PIDReturn[1]=Error;  // Store current error.
  PIDReturn[2]=Iterm;  // Store current integral term.
}
void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;  // Reset previous errors.
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;  // Reset previous integral terms.
}
void setup() {
  // pinMode(5, OUTPUT);  // Set pin 5 as output.
  // digitalWrite(5, HIGH);  // Turn on LED or other device connected to pin 5.
  // pinMode(13, OUTPUT);  // Set pin 13 as output.
  // digitalWrite(13, HIGH);  // Turn on LED or other device connected to pin 13.
  // Wire.setClock(400000);  // Set I2C clock speed to 400kHz.
  // Wire.begin();  // Initialize I2C communication.
  // delay(250);  // Wait for 250 milliseconds.
  // Wire.beginTransmission(0x68);  // Start communication with the gyroscope.
  // Wire.write(0x6B);  // Point to the power management register.
  // Wire.write(0x00);  // Wake up the gyroscope.
  // Wire.endTransmission();  // End transmission.
  Serial.begin(9600);
  while (!Serial);
  if (!IMU.begin()) { // Initialize the IMU
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    IMU.readGyroscope(RateRoll, RatePitch, RateYaw);
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;  // Calculate average roll rate for calibration.
  RateCalibrationPitch/=2000;  // Calculate average pitch rate for calibration.
  RateCalibrationYaw/=2000;  // Calculate average yaw rate for calibration.
  // analogWriteFrequency(1, 250);  // Set PWM frequency for motor 1. this is for Teensy boards
  // analogWriteFrequency(2, 250);  // Set PWM frequency for motor 2.
  // analogWriteFrequency(3, 250);  // Set PWM frequency for motor 3.
  // analogWriteFrequency(4, 250);  // Set PWM frequency for motor 4.
  analogWriteResolution(12);  // Set PWM resolution to 12 bits.
  // Set PWM frequency to about 732Hz (48MHz/64/1024)
  // REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_NPWM;
  // while (TCC0->SYNCBUSY.bit.WAVE);  // Wait for synchronization
 
  // // Set prescaler to 64
  // TCC0->CTRLA.reg &= ~TCC_CTRLA_PRESCALER_Msk;
  // TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV64;
  // while (TCC0->SYNCBUSY.bit.ENABLE);  // Wait for synchronization
  
  // // Set PER (period) register to 1024
  // TCC0->PER.reg = 1024;
  // while (TCC0->SYNCBUSY.bit.PER);  // Wait for synchronization
  
  // // Enable TCC0
  // TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
  // while (TCC0->SYNCBUSY.bit.ENABLE);  // Wait for synchronization

  pinMode(6, OUTPUT);  // Set pin 6 as output.
  digitalWrite(6, HIGH);  // Turn on LED or other device connected to pin 6.
  battery_voltage();  // Read and calculate battery voltage.
  if (Voltage > 8.3) { digitalWrite(5, LOW); BatteryAtStart=BatteryDefault; }  // Set initial battery capacity based on voltage.
  else if (Voltage < 7.5) {BatteryAtStart=30/100*BatteryDefault ;}  // Adjust battery capacity for low voltage.
  else { digitalWrite(5, LOW);
  BatteryAtStart=(82*Voltage-580)/100*BatteryDefault; }  // Calculate battery capacity for intermediate voltage.
  // PPMReader doesn't need explicit initialization, so we can remove the ReceiverInput.begin(14) line
  // ReceiverInput.begin(14);
  // Original throttle check (commented out)
  /*
  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050) {
    read_receiver();
    delay(4);
  }
  */
  // Wait for the throttle to be in a safe position
  // while (ppm.latestValidChannelValue(3, 0) < 1020 || ppm.latestValidChannelValue(3, 0) > 1050) {
  //   delay(4);
  // }
  LoopTimer=micros();  // Initialize loop timer.
}
void loop() {
  gyro_signals();  // Read gyroscope signals.
  IMU.readGyroscope(RateRoll, RatePitch, RateYaw);
  RateRoll = - (RateRoll - RateCalibrationRoll);
  RatePitch-=RateCalibrationPitch;
  RateYaw = - (RateYaw - RateCalibrationYaw);
  // Serial.print("RateROll [deg/s]");
  // Serial.print(RateRoll);
  // Serial.print('\t');
  // Serial.print("RatePitch [deg/s]");
  // Serial.print(RatePitch);
  // Serial.print('\t');
  // Serial.print("RateYaw [deg/s]");
  // Serial.println(RateYaw);
  read_receiver();  // Read receiver inputs.
  DesiredRateRoll=0.15*(ReceiverValue[0]-1500);  // Calculate desired roll rate.
  DesiredRatePitch=0.15*(ReceiverValue[1]-1500);  // Calculate desired pitch rate.
  InputThrottle=ReceiverValue[2];  // Get throttle input.
  DesiredRateYaw=0.15*(ReceiverValue[3]-1500);  // Calculate desired yaw rate.
  ErrorRateRoll=DesiredRateRoll-RateRoll;  // Calculate roll error.
  ErrorRatePitch=DesiredRatePitch-RatePitch;  // Calculate pitch error.
  ErrorRateYaw=DesiredRateYaw-RateYaw;  // Calculate yaw error.
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
       InputPitch=PIDReturn[0]; 
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw,
       IRateYaw, DRateYaw, PrevErrorRateYaw,
       PrevItermRateYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];
  if (InputThrottle > 1800) InputThrottle = 1800;
  MotorInput1= 1.024*(InputThrottle-InputRoll-InputPitch-InputYaw);
  MotorInput2= 1.024*(InputThrottle-InputRoll+InputPitch+InputYaw);
  MotorInput3= 1.024*(InputThrottle+InputRoll+InputPitch-InputYaw);
  MotorInput4= 1.024*(InputThrottle+InputRoll-InputPitch+InputYaw);
if (MotorInput1 > 2000)MotorInput1 = 1999;
  if (MotorInput2 > 2000)MotorInput2 = 1999; 
  if (MotorInput3 > 2000)MotorInput3 = 1999; 
  if (MotorInput4 > 2000)MotorInput4 = 1999;
  int ThrottleIdle=1180;
  if (MotorInput1 < ThrottleIdle) MotorInput1 =  ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 =  ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 =  ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 =  ThrottleIdle;
  int ThrottleCutOff=1000;
  if (ReceiverValue[2]<1050) {
    MotorInput1=ThrottleCutOff; 
    MotorInput2=ThrottleCutOff;
    MotorInput3=ThrottleCutOff; 
    MotorInput4=ThrottleCutOff;
    reset_pid();
  }
  analogWrite(1,MotorInput1);
  analogWrite(2,MotorInput2);
  analogWrite(3,MotorInput3); 
  analogWrite(4,MotorInput4);
  battery_voltage();
  CurrentConsumed=Current*1000*0.004/3600+CurrentConsumed;
  BatteryRemaining=(BatteryAtStart-CurrentConsumed)/BatteryDefault*100;
  if (BatteryRemaining<=30) digitalWrite(5, HIGH);
  else digitalWrite(5, LOW);
  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}