//This code is used to control a vacuum cleaner prototype developed by Bruface program students in Belgium
//                                               +-----+
//                  +----[PWR]-------------------| USB |--+
//                  |                            +-----+  |
//                  |         GND/RST2  [ ][ ]            |
//                  |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |    
//                  |          5V/MISO2 [ ][ ]  A4/SDA[ ] |    
//                  |                             AREF[ ] |
//                  |                              GND[ ] |
//                  | [ ]N/C                    SCK/13[ ] |   Motor left int4
//                  | [ ]IOREF                 MISO/12[ ] |   Motor left int3
//                  | [ ]RST                   MOSI/11[ ]~|   Motor left PWM ENB
//                  | [ ]3V3    +---+               10[ ]~|   Motor right PWM ENA
//                  | [ ]5v    -| A |-               9[ ]~|   Motor right int2
//                  | [ ]GND   -| R |-               8[ ] |   Motor right int1
//                  | [ ]GND   -| D |-                    |
//                  | [ ]Vin   -| U |-               7[ ] |   TRIG 3
//                  |          -| I |-               6[ ]~|   ECHO 3
//                  | [ ]A0    -| N |-               5[ ]~|   TRIG 2
//                  | [ ]A1    -| O |-               4[ ] |   ECHO 2
//                  | [ ]A2     +---+           INT1/3[ ]~|   ENCODER L
// cleanning syst   | [ ]A3                     INT0/2[ ] |   ENCODER R
//          ECHO 4  | [ ]A4/SDA  RST SCK MISO     TX>1[ ] |   TRIG 1
//          TRIG 4  | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |   ECHO 1
//                  |            [ ] [ ] [ ]              |
//                  |  UNO_R3    GND MOSI 5V  ____________/
//                   \_______________________/ 
//
// use delayMicroseconds() instead of millis() as interruptions are used for the encoders
// Encoders pins and variables
  const int rightEncoder = 2;
  const int leftEncoder = 3;
  volatile int rightPulses = 0;
  volatile int leftPulses = 0;
  int rightPulsesc = 0;
  int leftPulsesc = 0;
// ultrasonic sensors output and input 
  const int triggerPin1 = 1;
  const int echoPin1 = 0; 
  const int triggerPin2 = 5;
  const int echoPin2 = 4; 
  const int triggerPin3 = 7;
  const int echoPin3 = 6;
  const int triggerPin4 = 7;
  const int echoPin4 = 6;
// cleaning system
  const int cleaning = 17;
//RIGHT motor input voltage: IN1 = HIGH IN2 = LOW to go forward
  const int IN1 = 8; 
  const int IN2 = 9;
  const int ENA = 10;
//LEFT motor input voltage
  const int IN3 = 12; 
  const int IN4 = 13; 
  const int ENB = 11; //PWM of left motor

//robot geometric constants
  const int r = 0.035 //35 mm
  const int l = 0.0435 //43.5 mm distance from the center of mass to the wheels
        //red = 7.423 mm // from the x axis to the center to the wheel axis
        //blue 43.5 mm // from the center of the robot to the first wheel 
        
//timevariables for each sensors measuring the time the signal takes to come back
  //sensor 1 is left, 2 is front, 3 is right, 4 is bottom
  long sensorTime1;
  long sensorTime2;
  long sensorTime3;
  long sensorTime4;
  int distanceLeft;
  int distanceFront;
  int distanceRight;
  int distanceBottom;
  int a=0;

//controller variables
  const float KpR = 0.8
  float kpR = 0.8;
  float kiR = 0.005;

  float kpL = 08;
  float kiL = 0.005;
  volatile float re = 0;
  volatile float re1 = 0;
  volatile float re2 = 0;
  volatile float re3 = 0;
  volatile float rei = 0;
  
  volatile float le = 0;
  volatile float le1 = 0;
  volatile float le2 = 0;
  volatile float le3 = 0;
  volatile float lei = 0;

  long currT = micros();
  long prevTr = 0;
  long prevTl = 0;

  float deltaTR = ((float) (currT - prevTR)) / ( 1.0e6 ); prevTR = currT;
  float deltaTL = ((float) (currT - prevTL)) / ( 1.0e6 ); prevTL = currT;

//inverse kinematics variables
  float v = 0; //linear velocity of the robot
  float w = 0; //angular velocity of the robot
  float phir = 0; //desired motor speed
  float phil = 0; //desired motor speed
  float phirR = 0; //real motor speed
  float philR = 0; //real motor speed
  float A = 0.025; //gear ratio of wheels and motor
void setup() {
  //set the pins as output and input
  pinMode(triggerPin1, OUTPUT);
  pinMode(triggerPin2, OUTPUT);
  pinMode(triggerPin3, OUTPUT);
  pinMode(triggerPin4, OUTPUT);
  pinMode(echoPin1, INPUT); 
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(echoPin4, INPUT);

  pinMode(cleaning, OUTPUT);

  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  //pinMode(irPin, INPUT);
  //define the encoders interruption attachinterrupt(interruptPin,functionCalled, rising/falling/change) 
  attachInterrupt(digitalPinToInterrupt(rightEncoder), rightEncoderIncrease, RISING);
  attachInterrupt(digitalPinToInterrupt(leftEncoder), leftEncoderIncrease, RISING);
}

//Encoder functions triggered every time a rising or falling edge caused by the rotation changes, used to calculate speed, speed=(n*pulses)/time
void rightEncoderIncrease(){
  rightPulses += 1;}

void leftEncoderIncrease(){
  leftPulses += 1;}

void calculatemotorSpeed(){
  phirR = rightpulsesc*0.1*PI/deltaTr
  philR = leftpulsesc*0.1*PI/deltaTl
}
  
void inversekinematics(){
  phi1 = -(v+l*w)/r
  phi2 =  (v-l*w)/r
}
void controller(){
  currT = micros(); //set current time
  deltaTr = float((currT - prevTr)) / ( 1.0e6 ); prevTR = currT);
  deltaTl = float((currT - prevTl)) / ( 1.0e6 ); prevTL = currT);
  
  noInterrupts(); // disable interrupts temporarily while reading
  rightPulsesc = rightPulses; interrupts();
  noInterrupts(); // disable interrupts temporarily while reading
  leftPulsesc = leftPulses; interrupts(); // turn interrupts back on

  calculatemotorspeed()
  re = phi1 - phi1R; // error right
  le = phi2 - phi2R; // error left

  rei = re1 + re2 + re * deltaTr -re3; // integral
  lei = le1 + le2 + le * deltaTl -le3; // integral

  uR = (kpR * eR) + (kiR * rei));  // control signal
  uL = (kpL * eL) + (kiL * lei);  // control signal


}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
