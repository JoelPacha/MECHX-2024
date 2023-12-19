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

//pins for encoders
#define CLK_PINA 2
#define CLK_PINB 3 

//pins for motor voltage
#define ENA 10 //right motor input voltage IN1 = HIGH IN2 = LOW to go forward
#define IN1 8
#define IN2 9
#define ENB 11
#define IN3 12
#define IN4 13

//globals for loop control
#define DIRECTION_CW 0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction
#define KP 11     //1.03/(2*0.135 )
#define PI 3.14159265358979323846
#define E 0.175
#define RADIUS 0.038

//variables used in the interrupts should be "volatile" type to avoid unexpected behavior
volatile int rotatingCounterA = 0;
volatile int rotatingCounterB = 0; 
volatile int numberOfRotationsA = 0; 
volatile int numberOfRotationsB = 0; 
volatile int directionA = DIRECTION_CW;
volatile int directionB = DIRECTION_CW; 
const int numberOfHoles = 20; 

//dutycycles of PWM signals 
int dcA = 0; 
int dcB = 0; 

//ultrasonic sensors output and input 
const int triggerPin1 = 2;
const int echoPin1 = 0; 
const int triggerPin2 = 4;
const int echoPin2 = 1; 
const int triggerPin3 = 7;
const int echoPin3 = 2;
const int triggerPin4 = 19;
const int echoPin4 = 18;

//timevariables for each sensors measuring the time the sound signal takes to come back
//sensor 1 is left, 2 is below, 3 is right, 4 is front
long sensorTime1;
long sensorTime2;
long sensorTime3;
long sensorTime4; 

//distances between sensors and obstacles
int distanceLeft;
int distanceFront;
int distanceRight;
int distanceBelow; 
int minimalDistance = 20; 
////////////////////////variables for loop control
float Kp = 0; 
float totalAngle = 90;  
float distanceTotal = 0; 

//length of the rotation displacement spend in acceleration, continous speed and deceleration of motors
float distanceAccel = 0.25;             //const because acc= 1
float distanceDecel = 0.25;             //const because dec= -1
float distanceContinuous = 0;         

//time spent in each phase
float accelTime = 1 ; 
float decelTime = 1;
float continuousTime = 0; 
float totalTime = 0; 

float t = 0; 
float t_continuous = 0; 
float t_decel = 0; 
float ref = 0; 
float tolerance = 0.01; 

//positions on the wheels in m and degrees
float posA = 0; 
float posB = 0; 
float posADegree = 0;
float posBDegree = 0; 

//errors on the position towards the ref and the total distance
float errorA = 0; 
float errorB = 0;
float errorTotalA = 0; 
float errorTotalB = 0;

bool stopRotation = 1; 

void calcul(){
  distanceTotal = totalAngle * PI/180 * E/2;
  Kp = KP;
  if(distanceTotal>0.5){
  distanceContinuous = distanceTotal - distanceAccel - distanceDecel; 
  continuousTime = distanceContinuous/0.5; 
  totalTime = accelTime + decelTime + continuousTime;
  }
  else{
  distanceContinuous = 0; 
  continuousTime = 0; 
  distanceAccel = distanceTotal/2;
  distanceDecel = distanceTotal/2;
  accelTime  = sqrt(2 * distanceAccel/0.5); //acceleration of motors is 0.5 s/m^2
  decelTime = sqrt(2 * distanceDecel/(0.5)); 
  totalTime = accelTime + decelTime ;
  }
}

//calculates the distances in meters done by prototype
void calculate_posA(){
    posADegree = numberOfRotationsA*360 + (rotatingCounterA+1)/1440;      
    posA = posADegree*2*PI/360*RADIUS;                    
}

void calculate_posB(){
    posBDegree = numberOfRotationsB*360 + rotatingCounterB+1;
    posB = posBDegree*2*PI/360*RADIUS;
}


void regulate(float ref){
    calculate_posA(); 
    calculate_posB(); 
    
    errorA = ref-posA; 
    errorB = ref-posB;  
    dcA = Kp * errorA; 
    dcB = Kp * errorB; 
}

//defines the input signal
void accel(){
    ref = 0.5*pow(t,2)/2;  ///2.5000e-07
    regulate(ref);  
}

void continuous(){
    t_continuous = t-accelTime; 
    ref = 0.5*t_continuous + distanceAccel; 
    regulate(ref); 
}

void decel(){
    t_decel = t-accelTime-continuousTime; 
    ref = -0.5*pow(t_decel,2)/2 + distanceAccel + distanceContinuous + (0.5*accelTime)*t_decel; 
    regulate(ref);
}

void stop(){
    /*calculate_posA(); 
    calculate_posB(); 
    ref = distanceTotal; 
    errorA = ref-posA; 
    errorA = ref-posB;  
    dcA = Kp * errorA; 
    dcB = Kp * errorB; */
    stopRotation = 1; 
    dcA = 0.5; 
    dcB = 0.5; 
}

void sample_time(){
  while(!stopRotation){
    t = t+0.001; 
    errorTotalA = distanceTotal-posA; 
    errorTotalB = distanceTotal-posB;
    if ( (errorTotalA > tolerance) || (errorTotalB > tolerance) ){
        if (t<accelTime && t>0){
          accel();
        }
        else if(t<(continuousTime+ accelTime)){
          continuous(); 
        }
        else if(t>(accelTime +continuousTime)){
          decel(); 
        }
        Serial.println(dcA); 
        Serial.println(dcB); 
        analogWrite(ENA, dcA*255); 
        analogWrite(ENB, dcB*255); 
    }
    else{
        stop(); 
    }
    delay(1); 
  }
}


//interrupt functions called on rising edge of pin 2 and 3 (each time a hole passes in front of the encoder)
void ISR_encoderA() {
  rotatingCounterA++;
  if(rotatingCounterA == 800){
    numberOfRotationsA = numberOfRotationsA + 1;
    rotatingCounterA = 0; 
  }
}

void ISR_encoderB() {
  rotatingCounterB++;  
  if(rotatingCounterB == 800){
    numberOfRotationsB = numberOfRotationsB + 1;
    rotatingCounterB = 0;  
  }
}





////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);

  // configure encoder pins as inputs
  pinMode(CLK_PINA, INPUT);
  pinMode(CLK_PINB, INPUT);

  // use interrupt for CLK pin is enough
  // call ISR_encoderChange() when CLK pin changes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(CLK_PINA), ISR_encoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(CLK_PINB), ISR_encoderB, RISING);
  /////putting the change of directions 


  //set the sensors' pins as output and input
  pinMode(triggerPin1, OUTPUT);
  pinMode(triggerPin2, OUTPUT);
  pinMode(triggerPin3, OUTPUT);
  pinMode(triggerPin4, OUTPUT); 
  pinMode(echoPin1, INPUT); 
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(echoPin4, INPUT); 

  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  calcul(); 

}

void turn_right(){
  directionA = DIRECTION_CCW; 
  directionB = DIRECTION_CW;            //clock wise = forward      
  stopRotation = 0; 
  //right wheel goes backward 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  //left wheel goes forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  sample_time(); 
}

void turn_left(){
  directionA = DIRECTION_CW; 
  directionB = DIRECTION_CCW; 
  stopRotation = 0; 
  //right wheel goes forward 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  //left wheel goes  backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  sample_time(); 
}




void check_sensors() {

  //1. continuously checks the sensors for obstacle

  //sensor1 
  digitalWrite(triggerPin1, LOW);    //first restart the trigger pin
  delayMicroseconds(2);              //wait of 2µs
  digitalWrite(triggerPin1, HIGH);   
  delayMicroseconds(10);             //sends the ultrasonic wave during 10µs
  digitalWrite(triggerPin1, LOW);
  sensorTime1 = pulseIn(echoPin1, HIGH);
  distanceLeft = sensorTime1 * 0.034 / 2;   // transform into centimers 

  //sensor2
  digitalWrite(triggerPin2, LOW);    //first restart the trigger pin
  delayMicroseconds(2);              //wait of 2µs
  digitalWrite(triggerPin2, HIGH);   
  delayMicroseconds(10);             //sends the ultrasonic wave during 10µs
  digitalWrite(triggerPin2, LOW);
  sensorTime2 = pulseIn(echoPin2, HIGH);
  distanceBelow = sensorTime2 * 0.034 / 2;

  //sensor3
  digitalWrite(triggerPin3, LOW);    //first restart the trigger pin
  delayMicroseconds(2);              //wait of 2µs
  digitalWrite(triggerPin3, HIGH);   
  delayMicroseconds(10);             //sends the ultrasonic wave during 10µs
  digitalWrite(triggerPin3, LOW);
  sensorTime3 = pulseIn(echoPin3, HIGH);
  distanceRight = sensorTime3 * 0.034 / 2;

   //sensor4
  digitalWrite(triggerPin4, LOW);    //first restart the trigger pin
  delayMicroseconds(2);              //wait of 2µs
  digitalWrite(triggerPin4, HIGH);   
  delayMicroseconds(10);             //sends the ultrasonic wave during 10µs
  digitalWrite(triggerPin4, LOW);
  sensorTime4 = pulseIn(echoPin4, HIGH);
  distanceFront = sensorTime4 * 0.034 / 2;

  if (distanceFront <= minimalDistance ){
    //obstacle in front: we're gonna start spinning
    //if obstacle is on the left and front turn right
    if (distanceLeft <= minimalDistance && distanceRight > minimalDistance){
      turn_right(); 
    }
    //if obstacle is on the right and front turn left
    else if (distanceLeft > minimalDistance && distanceRight <= minimalDistance){
      turn_left(); 
    }
  }
  else{
    //wheels go forward
    dcA = 0.5; 
    dcA = 0.5; 
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, dcA*255); 
    analogWrite(ENB, dcB*255); 
  }
}


void loop() {
    Serial.print("DIRECTION: ");
    if (directionA == DIRECTION_CW)
      Serial.print("Clockwise");
    else
      Serial.print("Counter-clockwise");

    Serial.print(" | COUNTER: ");
    Serial.println(rotatingCounterA);
    
    check_sensors(); 
}















