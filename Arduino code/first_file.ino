//This code is used to control a vacuum cleaner prototype developed by Bruface program students in Belgium
//




motor 1 PWM
motor 1 in1
motor 1 in2

motor 2 PWM
motor 2 in1
motor 2 in2

encoder 1
encoder 2

trig 1
echo 1
trig 2
echo 2
trig 3
echo 3
trig 4
echo 4

cleaning system 

//                                               +-----+
//                  +----[PWR]-------------------| USB |--+
//                  |                            +-----+  |
//                  |         GND/RST2  [ ][ ]            |
//                  |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |    
//                  |          5V/MISO2 [ ][ ]  A4/SDA[ ] |    
//                  |                             AREF[ ] |
//                  |                              GND[ ] |
//                  | [ ]N/C                    SCK/13[ ] |   
//                  | [ ]IOREF                 MISO/12[ ] |   
//                  | [ ]RST                   MOSI/11[ ]~|   Motor Left PWM
//                  | [ ]3V3    +---+               10[ ]~|   Motor Right PWM
//                  | [ ]5v    -| A |-               9[ ]~|   Motor Left int2
//                  | [ ]GND   -| R |-               8[ ] |   Motor Left int1
//                  | [ ]GND   -| D |-                    |
//                  | [ ]Vin   -| U |-               7[ ] |   Motor Right int2
//                  |          -| I |-               6[ ]~|   Motor Right int1
//      Wall Dist   | [ ]A0    -| N |-               5[ ]~|   Encoder Left2
//      Front Obs   | [ ]A1    -| O |-               4[ ] |   Encoder Left1
//     Floor Dist   | [ ]A2     +---+           INT1/3[ ]~|   Encoder Right2
//                  | [ ]A3                     INT0/2[ ] |   Encoder Right1                       
//                  | [ ]A4/SDA  RST SCK MISO     TX>1[ ] |   Cleanning system
//                  | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |   D0
//                  |            [ ] [ ] [ ]              |
//                  |  UNO_R3    GND MOSI 5V  ____________/
//                   \_______________________/ 

#define CLN 1

#define ENCR1 2 
#define ENCR2 3 
#define PWMR 10
#define INR1 6
#define INR2 7

#define ENCL1 3 
#define ENCL2 4 
#define PWML 11
#define INL1 8
#define INL2 9
#define PI 3.1415926535897932384626433832795

#define WDIST A0
#define ODIST A1
#define FDIST A2

///////////////////////////////////////////
volatile int posR = 0;
volatile int posL = 0;

void setup() {
  Serial.begin(9600);
  pinMode(CLN, INPUT);
  pinMode(WDIST, INPUT);
  pinMode(ODIST, INPUT);
  pinMode(FDIST, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCR1), readEncoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCL1), readEncoderL, RISING);

  pinMode(CLN, OUTPUT);
  pinMode(PWMR, OUTPUT);
  pinMode(INR1, OUTPUT);
  pinMode(INR2, OUTPUT);
  pinMode(PWML, OUTPUT);
  pinMode(INL1, OUTPUT);
  pinMode(INL2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void readEncoderR() {
    int E = digitalRead(ENCR1);
    if (E > 0) {
      posR++;
    }
    else {
      posR--;
    }
  }

void readEncoderL() {
    int E = digitalRead(ENCL1);
    if (E > 0) {
      posR++;
    }
    else {
      posR--;
    }
  }
