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
 
//contractor so you can call a function that runs after the Main

#define SOFTTURN 22250
#define HARDTURN 22000
#define DATA_TOTAL 59 // Number taken from the Receiver 

// Function prototypes
int PIDFunctionLeft(int PWMLeft);
int PIDFunctionRight(int PWMRight);
void CRC_6_DARC(bool data);

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

//Varibles
const bool hvid = 0;
const bool sort = 1;
float NewDUTYLeft;
float NewDUTYRight;
float PWMLeft = 100; // Preset PWM to start out with on left motor
float PWMRight = 100; // Preset PWM to start out with on right motor
int timesleft;
int timesright;
bool right, mid, left;
unsigned long DurationLeftHigh, DurationRightHigh, DurationLeftLow, DurationRightLow, TotalDurationLeft, TotalDurationRight;

// CRC-6-DARC vars, polynomial 0x19 (1 011001, x^6 + x^4 + x^3 + 1)
bool Polynomial[7] = {1, 0, 1, 1, 0, 0, 1};
bool Checksum[6];
bool XOR[DATA_TOTAL];
bool Tmp[DATA_TOTAL];

void setup() 
{
  Serial.begin(115200);
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
}
   
void loop() 
{
  DurationLeftHigh = pulseIn(tacoSensorLeft, HIGH); // Take the time of a high pulse 
  DurationLeftLow = pulseIn(tacoSensorLeft, LOW); // Take the time of a low pulse
  DurationRightHigh = pulseIn(tacoSensorRight, HIGH);
  DurationRightLow = pulseIn(tacoSensorRight, LOW);

  TotalDurationLeft = DurationLeftHigh + DurationLeftLow; // Sum up the high and low pulse to get the full duty cycle
  TotalDurationRight = DurationRightHigh + DurationRightLow;
  
  right = digitalRead(rSensor);
  mid = digitalRead(mSensor);
  left = digitalRead(lSensor);
  PWMLeft = PIDFunctionLeft(PWMLeft);
  PWMRight = PIDFunctionRight(PWMRight);
}

int PIDFunctionLeft(int PWMLeft)
{
    float Kp_value = 0.008;
    float Ki_value = 0.00008;
    float Kd_value = 0;

    long Error; //error value
    float P;          // Holds the calculated Proportional value
    float I;
    float I_error;    // Holds the calculated Integral value
    float D;          // Holds the calculated Differential value
    float D_error;
    int SetPoint = 22500;
    unsigned int old_Duty = PWMLeft;

    if(left==0 && mid==1 && right==1) SetPoint = SOFTTURN;
    else if(left==0 && mid==0 && right==1) SetPoint = HARDTURN;
    
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
    
    //load the new duty cycle to the variable of the old so you can adjust for it.
    if(NewDUTYLeft>=256)      // If the PWM is above 256 it will be set to 255 PWM
    {
      NewDUTYLeft = 255;
    }

    else if(NewDUTYLeft<=-69)  // If the PWM is below 69 it will be set to 70 PWM 
    {
      NewDUTYLeft = 70;
    }

    old_Duty = NewDUTYLeft;

     // You can also implement a min max for your PWM value before you send it further.
    
     analogWrite(lMotorPinForth, old_Duty); // Set left motor to the new duty cycle

    /*
     * Radio transmission
     * Send Duty, P, I and D
     */
     // Assign data to Data
    String Data = String(NewDUTYLeft) + String(P*100) + String(I*10000) + String(D*100);
    
    // Print NewDUTYLeft, P, I, D and checksum
    CRC_6_DARC(Data); // Calculate checksum of Data
    Serial.print(NewDUTYLeft, BIN);
    Serial.print(P*10, BIN);
    Serial.print(I*10000, BIN);
    Serial.print(D*100, BIN);
    for(int i=0; i<sizeof(Checksum); i++) Serial.print(Checksum[i]);
    Serial.print("\n");
     
     return old_Duty;
}

int PIDFunctionRight(int PWMRight)
{
    float Kp_value = 0.008;
    float Ki_value = 0.0008;
    float Kd_value = 0;

    long Error; //error value
    float P;      // Holds the calculated Proportional value
    float I;
    float I_error;    // Holds the calculated Integral value
    float D;    // Holds the calculated Differential value
    float D_error;
    int SetPoint = 22500;
    unsigned int old_Duty = PWMRight;

    if(left==1 && mid==1 && right==0) SetPoint = SOFTTURN;
    else if(left==1 && mid==0 && right==0) SetPoint = HARDTURN;

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
            
    //load the new duty cycle to the variable of the old so you can adjust for it.
    if(NewDUTYRight>=256)     // If the PWM is above 256 it will be set to 255 PWM
    {
      NewDUTYRight = 255;
    }

    else if(NewDUTYRight<=69) // If the PWM is below 69 it will be set to 70 PWM 
    {
      NewDUTYRight = 70;
    }
    
    old_Duty = NewDUTYRight;
    
     // You can also implement a min max for your PWM value before you send it further.
     analogWrite(rMotorPinForth, old_Duty); // Set right motor to the new duty cycle
     
     return old_Duty;
}

void CRC_6_DARC(String data) {

  // Assign data to XOR[]
  memset(XOR, 0, sizeof(XOR));
  for(int d=0; d<sizeof(data); d++) XOR[d] = data[d];

  // For each number in var data
  for(int i=0; i<sizeof(data); i++)
  {
    // If the number is 1
    if(XOR[i]) {

      // Clear Tmp and set it to Polynomial at the first 1 in XOR from left to right
      memset(Tmp, 0, sizeof(Tmp));
      for(int d=0; d<sizeof(Polynomial); d++) Tmp[d+i] = Polynomial[d];

      // XOR Polynomial with XOR
      for(int d=0; d<sizeof(XOR); d++) XOR[d] = Tmp[d] ^ XOR[d];
    }
  }
  
  // Assign last 6 bits of XOR to Checksum
  for(int i=0; i<sizeof(Checksum); i++) Checksum[i] = XOR[i + sizeof(data)];
}
