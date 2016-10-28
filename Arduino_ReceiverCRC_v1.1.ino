/*
 * PWM[8], P[15], I[15], D[15]
 * Valid test data "10101010000000011001000000000000010000000000000000000110011";
 *                 "10111010000000011001100000000100010000000000000100000100111";
 */
// Definitions
#define CHECKSUM_SIZE 6
#define DATA_MIN 0
#define DATA_MAX 53
#define DATA_TOTAL 59 // DATA_MAX + CHECKSUM_SIZE
#define DATA_PWM_MIN 0
#define DATA_PWM_MAX 7
#define DATA_P_MIN 8
#define DATA_P_MAX 22
#define DATA_I_MIN 23
#define DATA_I_MAX 37
#define DATA_D_MIN 38
#define DATA_D_MAX 52

// Vars
String inputString = "";
String ChecksumInput = "";
String Checksum = "";
long pwm, P, I, D;
bool stringComplete = 0;
bool Polynomial[7] = {1, 0, 1, 1, 0, 0, 1}; // CRC-6-DARC, polynomial 0x19 (1 011001, x^6 + x^4 + x^3 + 1)
bool XOR[DATA_TOTAL];
bool Tmp[DATA_TOTAL];

// Function prototype
void CRC_6_DARC(String data);

void setup() {
  Serial.begin(115200);
  while(!Serial);
}

void loop() {
  
  // If received info passed basic validation
  if(stringComplete) {

    // Calculate checksum of inputString
    CRC_6_DARC(inputString.substring(DATA_MIN, DATA_MAX));

    // Read the checksum of inputString
    for(int i = 0; i < CHECKSUM_SIZE; i++) ChecksumInput += inputString[i + DATA_MAX];

    // Compare calculated checksum with read checksum from inputString
    if(Checksum == ChecksumInput)
    {
      // Assign values to pwm, P, I and D, converted from binary string to decimal
      for(int i = DATA_PWM_MIN; i<= DATA_PWM_MAX; i++) pwm += (inputString[i] == '1' ?round(pow(2, DATA_PWM_MAX - i)) :0);
      for(int i = DATA_P_MIN;   i<=  DATA_P_MAX;  i++) P   += (inputString[i] == '1' ?round(pow(2, DATA_P_MAX - i))   :0);
      for(int i = DATA_I_MIN;   i<=  DATA_I_MAX;  i++) I   += (inputString[i] == '1' ?round(pow(2, DATA_I_MAX - i))   :0);
      for(int i = DATA_D_MIN;   i<=  DATA_D_MAX;  i++) D   += (inputString[i] == '1' ?round(pow(2, DATA_D_MAX - i))   :0);
      
      Serial.print("Received: " + inputString);         // Print info received (binary)
      Serial.print("PWM: " + String((float)(pwm*100)/255) + "% ");  // Print PWM
      Serial.print("P: "   + String((float)P/10, 1) + " "); // Print P
      Serial.print("I: "   + String((float)I/10000, 4) + " "); // Print I
      Serial.println("D: "   + String((float)D/100, 2) + "\n"); // Print D
    }
    // Checksums do not match
    else Serial.println("CRC-6-DARC failed: Invalid data\n" + Checksum + " is NOT equal to " + ChecksumInput + "\n");
  
    // Reset vars for new string
    inputString = "", Checksum = "", ChecksumInput = "";
    pwm = 0, P = 0, I = 0, D = 0, stringComplete = 0;
  }
}

void serialEvent() {
  
  // Wait for serial data
  while(Serial.available())
  {
    // Read a byte
    char Received = (char)Serial.read();
    
    // Add byte to string
    // Must be boolean or '\n'
    if ((Received == '0') || (Received == '1') || (Received == '\n')); // This char is OK
    else return;
    
    inputString += String(Received);
    
    //if(inputString.length() > DATA_TOTAL) return;

    // If byte is a newline '\n', set stringComplete to 1
    if(Received == '\n') stringComplete = 1;
    //if(inputString.length() == DATA_TOTAL) stringComplete = 1;
  }
}

void CRC_6_DARC(String data) {

  // Assign data to XOR[]
  memset(XOR, 0, sizeof(XOR));
  for(int d = DATA_MIN; d < DATA_MAX; d++) XOR[d] = (data[d] == '0' ?0:1);

  // For each character in data string
  for(int c = DATA_MIN; c < DATA_MAX; c++)
  {
    if(XOR[c])
    {
      // Clear Tmp and set it to Polynomial at the first 1 in XOR from left to right
      memset(Tmp, 0, sizeof(Tmp));
      for(int p = 0; p < sizeof(Polynomial); p++) Tmp[p + c] = Polynomial[p];

      // XOR Polynomial with XOR at each 1's
      for(int x = 0; x < sizeof(XOR); x++) XOR[x] = Tmp[x] ^ XOR[x];
    }
  }

  // Assign checksum of XOR to Checksum
  for(int i = 0; i < CHECKSUM_SIZE; i++) Checksum += (XOR[i + data.length()] == 1 ?1:0);
}
