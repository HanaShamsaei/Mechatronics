//////////////////////////////libraries
#include <Wire.h>
#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <SoftwareSerial.h>
SoftwareSerial MySerial(53,51);

/// app
float BTvar = 0, Setpoint=0;
int a = 0, Flag = 3, FlagOnOff = 1,Flag_Weight = 0, Setpoint1 = 90;
char c1;
//////////////////////////////// MPU sensor variables
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

//////////////////////////////////// Motor variables
//motor1
#define ENA 2 // Encoder signal A
#define ENB 22 // Encoder signal B
#define pulse 4 // PWM signal for motor speed control
#define IN1 8 // Motor direction control pin 1
#define IN2 7 // Motor direction control pin 2
//motor2
#define ENA2 3 // Encoder signal A
#define ENB2 5 // Encoder signal B
#define pulse2 6 // PWM signal for motor speed control
#define IN12 10 // Motor direction control pin 1
#define IN22 9 // Motor direction control pin 2


#define LEFT_SENSOR_PIN A0
#define RIGHT_SENSOR_PIN A1
// Steering PID constants (tune these)
float Kp_line = 425.0;
float Ki_line = 0.0;
float Kd_line = 0.0;

float lineError = 0;
float lastLineError = 0;
float lineIntegral = 0;


bool direction = true; // Motor direction: true = CCW, false = CW
bool direction2 = true; // Motor direction: true = CCW, false = CW
long int T = 4000; // Timer period in microseconds (1 second)
long int N = 0, N2 = 0; // Counter for encoder pulses
float v, v2, x = 0, x1 = 0, x2 = 0, delta_v = 0, x_des = 0; // Motor speed in RPM
int pulse_value, pulse_value2;
float output, output2, output_v1 = 0, output_v2 = 0;
int d1 = 1, d12=0;

///////////////////////////////////// PID variables
float Kp = 25;                    //P Gain; Mine was 30
float Ki = 0.4;                  //I Gain; Mine was 0.61
float Kd =23;                     //D Gain; Mine was 9
float Moving_Speed = 20;          //Moving speed with Bluetooth Control; Mine was 20
float Max_Speed = 160;            //Max mooving speed; Mine was 160
float Temp_Error, PID_I, gyro_input, PID_Value, Last_D_Error, PID_Output;
float output_t;



///////////////////////////////////// PID Syncronized Motors variables
float Kp1 = 30;                    //P Gain; Mine was 30
float Ki1 = 0.5;                  //I Gain; Mine was 0.61
float Kd1 = 33;                     //D Gain; Mine was 9
float Moving_Speed1 = 20;          //Moving speed with Bluetooth Control; Mine was 20
float Max_Speed1 = 160;            //Max mooving speed; Mine was 160
float Temp_Error1, PID_I1, gyro_input1, PID_Value1, Last_D_Error1, PID_Output1;
float output_t1;



/////////////////////////////////////Ultrasonic sensor variables
const int trigPin = 25;
const int echoPin = 24;
float duration, distance;
const int trigPin1 = 27;
const int echoPin1 = 26;
float duration1, distance1;


/////////////////////////////////////TCRT5000 sensor variables
const int pinIRd = 23;
const int pinIRa = A0;
int IRvalueA = 0;
int IRvalueD = 0;
const int pinIRd1 = 30;
const int pinIRa1 = A1;
int IRvalueA1 = 0;
int IRvalueD1 = 0;


void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();     
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096-0.03;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096-0.02;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}
float getLineError() {
  int leftValue = analogRead(LEFT_SENSOR_PIN);
  int rightValue = analogRead(RIGHT_SENSOR_PIN);

  float error_line = (float)(rightValue - leftValue) / 1023.0;

  return error_line;
}
float computeLinePID(float error_line) {
  lineIntegral += error_line;  // With only two sensors, be careful with integral windup
  float derivative = error_line - lastLineError;
  lastLineError = error_line;
  
  float output_line = (Kp_line * error_line) + (Ki_line * lineIntegral) + (Kd_line * derivative);
  
  // Limit turn output
  if (output_line > 255)  output_line = 255;
  if (output_line < -255) output_line = -255;
  
  return output_line;
}
void setup() {
  Serial.begin(57600);
  // ---- (NEW) Start Bluetooth on MySerial for Arduino Mega ----
  MySerial.begin(9600); 
  // ----------------------------------------------------------
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
  pinMode(ENA, INPUT_PULLUP);
  pinMode(ENB, INPUT);
  pinMode(pulse, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA2, INPUT_PULLUP);
  pinMode(ENB2, INPUT);
  pinMode(pulse2, OUTPUT);
  pinMode(IN12, OUTPUT);
  pinMode(IN22, OUTPUT);
  // ultrasonic1
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // ultrasonic2
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  // TCRT5000 1
  pinMode(pinIRd,INPUT);
  pinMode(pinIRa,INPUT);
  // TCRT5000 2
  pinMode(pinIRd1,INPUT);
  pinMode(pinIRa1,INPUT);
  // analogWrite(pulse, 255); // Set maximum speed for the motor (PWM = 255)
  Timer1.initialize(T); // Initialize Timer1 with period T
  Timer1.attachInterrupt(velocity); // Attach interrupt function for velocity calculation
  attachInterrupt(digitalPinToInterrupt(ENA), pulseCounter, RISING); // Attach encoder interrupt on rising edge
  attachInterrupt(digitalPinToInterrupt(ENA2), pulseCounter2, RISING); // Attach encoder interrupt on rising edge
  digitalWrite(IN1, HIGH); // Set initial direction to CCW
  digitalWrite(IN2, LOW);
}
void loop() {
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  
  while (micros() - LoopTimer < 16);
  LoopTimer=micros();
  if (direction) {
    //Serial.println("CCW");
    d1 = -1;
  } else {
    //Serial.println("CW");
    d1 = 1;
  }
  if (direction) {
    //Serial.println("CCW");
    d12 = -1;
  } else {
    //Serial.println("CW");
    d12 = 1;
  }

  if(PID_Value <0){
    d1 = -1;
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN12, LOW);
    digitalWrite(IN22, HIGH);
  }
  else if(PID_Value > 0){
    d1 = 1;
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN12, HIGH);
    digitalWrite(IN22, LOW);
  }
  float lineErr = getLineError();
  float turnOutput = computeLinePID(lineErr);


  if (FlagOnOff){
    if(Flag == 3){
         output_v1 = output-delta_v*0.64+a;
         output_v2 = output+delta_v*0.64-a;
    }
    else{
        output_v1 = output-delta_v*0.64-turnOutput;
        output_v2 = output+delta_v*0.64+turnOutput/2;

    }
    output_v1 = output-delta_v*0.64+a;
    output_v2 = output+delta_v*0.64-a;

  }
  else{
    output_v1 = 0;
    output_v2 = 0;

  }

  analogWrite(pulse, output_v1);
  analogWrite(pulse2, output_v2);


  // ------------------ READ BLUETOOTH ------------------
  readBluetoothCommands(); 
  // ----------------------------------------------------
  if((Flag_Weight == 1) && (c1 == '0')){
    Setpoint = 1.5;
    Kp = 30;
  }
  else if((Flag_Weight == 1) && (c1 == '2')){
    Setpoint = -1.2;
    Kp = 30;
  }

}



void readBluetoothCommands() {
  // Check if there is data from the Bluetooth module
  while (MySerial.available() > 0) {
    char c = MySerial.read();
    c1 = c;
    if (c < 32 || c > 126) {
      return;  // Skip processing this character
    }

    // Ignore newline ('\n') and carriage return ('\r')
    if (c == '\n' || c == '\r') {
      return;  
    }
    // Example: Suppose you send single-character commands from your phone app:
    // 'f' = forward, 'b' = backward, 'l' = left, 'r' = right, etc.
    switch (c) {
      case '0':
        Serial.println("BT: UP");
        Setpoint = 1.9;
        break;

      case '1':
        Serial.println("BT: STOP");
        Setpoint = 0;
        a = 0;
        break;

      case '2':
        Serial.println("BT: DOWN");
        Setpoint = -1.8;
        break;

      case '3':
        Serial.println("BT: CONTROL");
        Flag = 3;
        break;

      case '4':
        Serial.println("BT: LINE TRAKING");
        Flag = 4;
        break;

      case '5':
        Serial.println("BT: LEFT");
        a += 10;
        break;

      case '6':
        Serial.println("BT: RIGHT");
        a -= 10;
        break;
      
      case '7':
        Serial.println("BT: ON");
        FlagOnOff = 1;

        break;

      case '8':
        // Decrease Kp or speed
        Serial.println("BT: OFF");
        FlagOnOff = 0;
        break;
      case '9':
        //Weight
        Serial.println("Weight");
        Flag_Weight = 1;
        break;
      case '#':
        //No Weight
        Serial.println("No Weight");
        Flag_Weight = 0;
        break;

      case '\n':
        break;

      case ' ':
        break;

      case '\r':
        break;  

      case 'H':
        break; 

      case '�':
        break; 

      case '$':
        break;   

      default:
        Serial.println("BT: NO MESSAGE: ");
        break;
    }
  }
}


//PID funcrion
float calcPID(){
  Temp_Error = KalmanAngleRoll - Setpoint;

  if (PID_Value > 10 || PID_Value < -10) {
    Temp_Error += PID_Value * 0.015 ;
  }

  //I value
  PID_I *= 1 ;
  PID_I += Ki * Temp_Error;                                                 //Calculate the "I" value
  if (PID_I > 200)PID_I = 200;                                              //We limit the "I" to the maximum output
  else if (PID_I < -200)PID_I = -200;


  //Calculate the PID output value
  PID_Value = Kp * Temp_Error + PID_I + Kd * (Temp_Error - Last_D_Error);
  if (PID_Value > 400)PID_Value = 400;                                      //Limit the P+I to the maximum output
  else if (PID_Value < -400)PID_Value = -400;

  Last_D_Error = Temp_Error;                                                //Store the error for the next loop

  if (PID_Value < 6 && PID_Value > - 6)PID_Value = 0;                       //Dead-band where the robot is more or less balanced

  if (KalmanAngleRoll > 30 || KalmanAngleRoll < -30) {              //If the robot falls or the "Activated" is 0
    PID_Value = 0;                                                          //Set the PID output to 0 so the motors are stopped
    PID_I = 0;                                                              //Reset the I-controller memory
  }
  return PID_Value;
}
// Interrupt service routine to count encoder pulses
void pulseCounter() {
  N++;
  // Check the state of ENB to determine motor direction
  if (digitalRead(ENB) == 1) {
  direction = true; // CCW (Counter-Clockwise)
  } else {
  direction = false; // CW (Clockwise)
}
}
void pulseCounter2() {
  N2++;
  // Check the state of ENB to determine motor direction
  if (digitalRead(ENB2) == 1) {
  direction2 = true; // CCW (Counter-Clockwise)
  d1=1;
  } else {
  direction2 = false; // CW (Clockwise)
  d1=-1;
}
}
float syncMotors(float delta_v1){
  //v2 = delta_v * kp;
  Temp_Error1 = delta_v1 - Setpoint1;

  if (PID_Value1 > 10 || PID_Value1 < -10) {
    Temp_Error1 += PID_Value1 * 0.015 ;
  }

  //I value
  PID_I1 += Ki1 * Temp_Error1;                                                 //Calculate the "I" value
  if (PID_I1 > 400)PID_I1 = 400;                                              //We limit the "I" to the maximum output
  else if (PID_I1 < -400)PID_I1 = -400;


  //Calculate the PID output value
  PID_Value1 = Kp1 * Temp_Error1 + PID_I1 + Kd1 * (Temp_Error1 - Last_D_Error1);
  if (PID_Value1 > 400)PID_Value1 = 400;                                      //Limit the P+I to the maximum output
  else if (PID_Value1 < -400)PID_Value1 = -400;

  Last_D_Error1 = Temp_Error1;                                                //Store the error for the next loop

  if (PID_Value1 < 6 && PID_Value1 > - 6)PID_Value1 = 0;                       //Dead-band where the robot is more or less balanced

  // if (KalmanAngleRoll > 30 || KalmanAngleRoll < -30) {              //If the robot falls or the "Activated" is 0
  //   PID_Value1 = 0;                                                          //Set the PID output to 0 so the motors are stopped
  //   PID_I1 = 0;                                                              //Reset the I-controller memory
  // }
  return PID_Value1;
}
// Timer interrupt function to calculate motor speed
void velocity() {
  PID_Value = calcPID();
  PID_Output = PID_Value;
  output = 0.63 * abs(PID_Output);
  v = (60.00 * N) / (1.00); // Convert pulse count to RPM
  v2 = (60.00 * N2) / (1.00); // Convert pulse count to RPM
  delta_v = v2 + v;
  delta_v*=1;
  delta_v = syncMotors(delta_v)*-1;
  N = 0; // Reset pulse count for next calculation
  N2 =0;
}
