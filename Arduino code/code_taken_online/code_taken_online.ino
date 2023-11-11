#define ENCR 2 // WHITE
#define ENR 4 // BLACK
#define PWMR 10
#define INR2 6
#define INR1 7

#define ENCL 3 // GREEN
#define ENL 5 // BLUE
#define PWML 11
#define INL2 8
#define INL1 9
#define PI 3.1415926535897932384626433832795

// PARAMETER RODA
const float driveX = 100;   // cm
const float driveY = 100;
const int wheel_d = 80;           // Wheel diameter (mm)
const float wheel_c = PI * wheel_d; // Wheel circumference (mm) (KR)
const int counts_per_rev = 104;   // (7 pairs N-S) * (16 : 1 gearbox) = 224 (RR)
int power = 75;
int moff = 5;
int pwR = 20 ;
int pwL = 20 ;
//---------------------------------------------------------------------------------------------------------------------------------------------------------//

volatile int posR = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevTR = 0;
float eprevR = 0;
float eintegralR = 0;

volatile int posL = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevTL = 0;
float eprevL = 0;
float eintegralL = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------//

//PARAMETER ODOMETRY
const float wheelDistance = 220;
float meanDistance = 0;
float theta = 0;
float posX = 0;
float posY = 0;
int x = 0;
int y = 0;
int deg = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------//

/*
  // KEC 50
  // PID constants 1/10 = 0.1
  float kpR = 1.1;
  float kiR = 0.0008;
  float kdR = 0.016;

  float kpL = 1.1;
  float kiL = 0.0008;
  float kdL = 0.016;
*/

// KEC 75
// PID constants 1/10 = 0.1
float kpR = 0.68;
float kiR = 0.0008;
float kdR = 0.08;

float kpL = 0.7;
float kiL = 0.0008;
float kdL = 0.08;

void setup() {
  Serial.begin(9600);
  pinMode(ENCR, INPUT);
  pinMode(ENR, INPUT);
  pinMode(ENCL, INPUT);
  pinMode(ENL, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCR), readEncoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCL), readEncoderL, RISING);

  pinMode(PWMR, OUTPUT);
  pinMode(INR1, OUTPUT);
  pinMode(INR2, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(INL1, OUTPUT);
  pinMode(INL2, OUTPUT);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------//

void loop() {
  int drive = driveX + driveY;
    if (pwR && pwL <= power) {
      pwR += 5;
      pwL += 5;
    }

    // Calculate target number of ticks
    float num_rev = (drive * 10) / wheel_c;  // Convert to mm (RPSE)
    unsigned long target = num_rev * counts_per_rev;
    unsigned long tmm = (num_rev * wheel_c) / 10;

    setPID(target);
    Odometry();
  }
  //---------------------------------------------------------------------------------------------------------------------------------------------------------//

  void setPID(int target) {


    // RIGHT WHEEL
    long currT = micros();  // time difference
    float deltaTR = ((float) (currT - prevTR)) / ( 1.0e6 ); prevTR = currT;
    float deltaTL = ((float) (currT - prevTL)) / ( 1.0e6 ); prevTL = currT;

    int posiR = 0; noInterrupts(); // disable interrupts temporarily while reading
    posiR = posR; interrupts(); // turn interrupts back on
    int posiL = 0; noInterrupts(); // disable interrupts temporarily while reading
    posiL = posL; interrupts(); // turn interrupts back on

    int eR = posR - target; // error
    int eL = posL - target; // error

    float dedtR = (eR - eprevR) / (deltaTR); // derivative
    eintegralR = eintegralR + eR * deltaTR; // integral
    float dedtL = (eL - eprevL) / (deltaTL); // derivative
    eintegralL = eintegralL + eL * deltaTL; // integral

    float uR = (kpR * eR) + (kiR * eintegralR) + (kdR * dedtR);  // control signal
    float uL = (kpL * eL) + (kiL * eintegralL) + (kdL * dedtL);  // control signal

    eprevR = eR; // store previous error
    eprevL = eL; // store previous error
    //---------------------------------------------------------------------------------------------------------------------------------------------------------//
    unsigned long num_ticks_r;
    unsigned long num_ticks_l;

    unsigned long diff_r;
    unsigned long diff_l;

    unsigned long enc_r_prev = posR;
    unsigned long enc_l_prev = posL;

    int pwrR = fabs(uR); // motor power
    int pwrL = fabs(uL); // motor power

    if ( pwrR && pwrL >= power ) {
      pwrR = pwR;
      pwrL = pwL;
    }

    int dirR = 1; // motor direction
    i