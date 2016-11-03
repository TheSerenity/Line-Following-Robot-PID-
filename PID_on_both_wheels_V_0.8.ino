/*
 * Line-Following Robot with PID
 * 
 * Erhvervsakademiet Lillebaelt
 * 
 * Credits:
 * - Daniel Larsen
 * - Martin Salling
 * - Jonas Thomsen
 */

#include <SoftwareSerial.h> // For Bluetooth communication

#define SETPOINT 27500
#define SOFTTURN 27150
#define HARDTURN 26800 
#define GOLEFT 0
#define GORIGHT 1


// Function prototypes
int PIDFunctionLeft(int PWMLeft);
int PIDFunctionRight(int PWMRight);

// Pin numbers
const int lSensor = 2; // Left sensor
const int mSensor = 4; // mid sensor
const int rSensor = 7; // Right sensor
const int lMotorPinForth = 5; // Left motor forth (when HIGH and opposite of lMotorPinBack)
const int lMotorPinBack = 6; // Left motor back (when HIGH and opposite of lMotorPinForth)
const int rMotorPinForth = 3; // Right motor (when HIGH and opposite of rMotorPinBack)
const int rMotorPinBack = 11; // Right motor back (when HIGH and opposite of rMotorPinForth)
const int tacoSensorLeft = 12; // Left tacosensor
const int tacoSensorRight = 9; // Right tacosensor
const int bluetoothTx = 8;  // TX-O pin of bluetooth mate, Arduino D2
const int bluetoothRx = 10;  // RX-I pin of bluetooth mate, Arduino D3

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// Vars
const bool hvid = 0;
const bool sort = 1;
bool right, mid, left;
bool Operating = 0;
bool State;
float NewDUTYLeft;
float NewDUTYRight;
float PWMLeft = 70; // Preset PWM to start out with on left motor
float PWMRight = 70; // Preset PWM to start out with on right motor
int TacoValLeft;
int TacoValRight;
int timesleft;
int timesright;
unsigned long DurationLeftHigh, DurationRightHigh, DurationLeftLow, DurationRightLow, TotalDurationLeft, TotalDurationRight;
char BTInput;


void setup() 
{
  Serial.begin(250000);
  pinMode(lMotorPinForth, OUTPUT);
  pinMode(lMotorPinBack, OUTPUT);
  pinMode(rMotorPinForth, OUTPUT);
  pinMode(rMotorPinBack, OUTPUT);
  pinMode(tacoSensorLeft, INPUT);
  pinMode(tacoSensorRight, INPUT);
  pinMode(lSensor, INPUT);
  pinMode(mSensor, INPUT);
  pinMode(rSensor, INPUT);
  digitalWrite(rMotorPinBack, LOW);
  digitalWrite(lMotorPinBack, LOW);  
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps, independant of Serial bps
}
   
void loop() 
{
  // Read incoming bluetooth
  if(bluetooth.available()) BTInput = (char)bluetooth.read();
  
  // If input is 83 (S)tart, start robot
  if(BTInput == 83) Operating = 1;

  // Else if input is 80 (P)ause robot
  else if(BTInput == 80 && Operating) 
  {
    Operating = 0;
    analogWrite(rMotorPinForth, 0);
    analogWrite(lMotorPinForth, 0);  
  }
  
  // If robot is running...
  if(Operating)
  {   
    DurationLeftHigh = pulseIn(tacoSensorLeft, HIGH); // Take the time of a high pulse 
    DurationLeftLow = pulseIn(tacoSensorLeft, LOW); // Take the time of a low pulse
    DurationRightHigh = pulseIn(tacoSensorRight, HIGH);
    DurationRightLow = pulseIn(tacoSensorRight, LOW);
    
    TotalDurationLeft = DurationLeftHigh + DurationLeftLow; // Sum up the high and low pulse to get the full duty cycle
    TotalDurationRight = DurationRightHigh + DurationRightLow;
    bluetooth.print("*L\n\nDur = ");
    bluetooth.print(TotalDurationLeft);
    bluetooth.print("*");
    bluetooth.print("*R\n\nDur = ");
    bluetooth.print(TotalDurationRight);
    bluetooth.print("*");
    
    right = digitalRead(rSensor);
    mid = digitalRead(mSensor);
    left = digitalRead(lSensor);
    PWMLeft = PIDFunctionLeft(PWMLeft);
    PWMRight = PIDFunctionRight(PWMRight);
    int L_PMW_procent = (PWMRight/255*100);
    int R_PMW_procent = (PWMLeft/255*100);
    bluetooth.print("*G");
    bluetooth.print(L_PMW_procent);
    bluetooth.print(",");
    bluetooth.print(R_PMW_procent);
    bluetooth.print("*");
  }
}

int PIDFunctionLeft(int PWMLeft)
{
  float Kp_value = 0.008;
  float Ki_value = 0.00008;
  float Kd_value = 0.0008;

  long Error; //error value
  float P;          // Holds the calculated Proportional value
  float I;
  float I_error;    // Holds the calculated Integral value
  float D;          // Holds the calculated Differential value
  float D_error;
  int SetPoint = SETPOINT;
  unsigned int old_Duty = PWMLeft;
  
  if(left==0 && mid==1 && right==1) 
  {
    SetPoint = SOFTTURN;
    State = GORIGHT;
  }
  else if(left==0 && mid==0 && right==1) 
  {
    SetPoint = HARDTURN;
    State = GORIGHT;
  }
  else if((left==1 && mid==1 && right==1) ||
          (left==0 && mid==0 && right==0) ||
          (left==1 && mid==0 && right==1))
  {
    if(State == GORIGHT) SetPoint = HARDTURN - 2000;
  }
  
  //  Error of Set point vs FB
  Error = (SetPoint - TotalDurationLeft);
  
  // P factor and error
  P = Kp_value * Error;

  // I_erro over time factor of the I term
  I_error += Error;

  // Set up a Integral windup protection. Set a max and a min level for the I error

  // I factor over time. Remember I is error over time. Which means if error is 1 over two iteration error is now 2
  I = Ki_value * I_error;

  // D factor of the PID
  D = Kd_value * (D_error - Error);

  // The New error becomes the error the D expects and if not the same it will adjust for it.
  D_error = Error;

  // the PID SUM Logic
  NewDUTYLeft = old_Duty - (P + I +D);
  bluetooth.print("*L");
  bluetooth.println(" ");
  bluetooth.print("P = ");
  bluetooth.println(P);
  bluetooth.print("I = ");
  bluetooth.println(I);
  bluetooth.print("D = ");
  bluetooth.println(D);
  bluetooth.print("\n*");
  
  //load the new duty cycle to the variable of the old so you can adjust for it.
  if(NewDUTYLeft>=256) NewDUTYLeft = 255; // If the PWM is above 255 it will be set to 255 PWM
  else if(NewDUTYLeft<=4) NewDUTYLeft = 5; // If the PWM is below 5 it will be set to 5 PWM 
  
  old_Duty = NewDUTYLeft;
  
  // You can also implement a min max for your PWM value before you send it further.    
  analogWrite(lMotorPinForth, old_Duty); // Set left motor to the new duty cycle
  
  return old_Duty;
}

int PIDFunctionRight(int PWMRight)
{
  float Kp_value = 0.008;
  float Ki_value = 0.0008;
  float Kd_value = 0.0008;
  
  long Error; //error value
  float P;      // Holds the calculated Proportional value
  float I;
  float I_error;    // Holds the calculated Integral value
  float D;    // Holds the calculated Differential value
  float D_error;
  int SetPoint = SETPOINT;
  unsigned int old_Duty = PWMRight;
  
  if(left==1 && mid==1 && right==0) 
  {
    SetPoint = SOFTTURN;
    State = GOLEFT;
  }
  else if(left==1 && mid==0 && right==0) 
  {
    SetPoint = HARDTURN;
    State = GOLEFT;
  }
  else if((left==1 && mid==1 && right==1) ||
          (left==0 && mid==0 && right==0) ||
          (left==1 && mid==0 && right==1))
  {
    if(State == GOLEFT) SetPoint = HARDTURN - 2000;
  }
  
  // Error of Set point vs FB
  Error = (SetPoint - TotalDurationRight);
  
  // P factor and error
  P = Kp_value * Error;
  
  // I_erro over time factor of the I term
  I_error += Error;
  
  // Set up a Integral windup protection. Set a max and a min level for the I error
  
  // I factor over time. Remember I is error over time. Which means if error is 1 over two iteration error is now 2
  I = Ki_value * I_error;
  
  // D factor of the PID
  D = Kd_value * (D_error - Error);
  
  // The New error becomes the error the D expects and if not the same it will adjust for it.
  D_error = Error;
  
  // the PID SUM Logic
  NewDUTYRight = old_Duty - (P + I +D);     
  bluetooth.print("*R\n");
  bluetooth.print("P = ");
  bluetooth.println(P);
  bluetooth.print("I = ");
  bluetooth.println(I);
  bluetooth.print("D = ");
  bluetooth.println(D);
  bluetooth.print("\n*");
  
  //load the new duty cycle to the variable of the old so you can adjust for it.
  if(NewDUTYRight>=256) NewDUTYRight = 255; // If the PWM is above 255 it will be set to 255 PWM
  else if(NewDUTYRight<=4) NewDUTYRight = 5; // If the PWM is below 5 it will be set to 5 PWM
  
  old_Duty = NewDUTYRight;
  
  // You can also implement a min max for your PWM value before you send it further.
  analogWrite(rMotorPinForth, old_Duty); // Set right motor to the new duty cycle
  return old_Duty;
}

