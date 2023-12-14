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
#define CLK_PIN1 2
#define CLK_PIN2 3 

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
#define E 0.135
#define RAYON 0.041

//variables used in the interrupts should be "volatile" type to avoid unexpected behavior
volatile int rotatingCounter = 0;
volatile int numberOfRotations1 = 0; 
volatile int numberOfRotations2 = 0; 
volatile int direction = DIRECTION_CW;
volatile unsigned long lastTime;  // for debouncing
int prevCounter ;
//dutycycles of PWM signals 
int dcA = 0; 
int dcB = 0; 

// ultrasonic sensors output and input 
const int triggerPin1 = 2;
const int echoPin1 = 0; 
const int triggerPin2 = 4;
const int echoPin2 = 1; 
const int triggerPin3 = 7;
const int echoPin3 = 2;
const int triggerPin4 = 7;
const int echoPin4 = 2;

//timevariables for each sensors measuring the time the sound signal takes to come back
//sensor 1 is left, 2 is front, 3 is right
long sensorTime1;
long sensorTime2;
long sensorTime3;
long sensorTime4; 

int distanceLeft;
int distanceFront;
int distanceRight;
int distanceBelow; 
int a=0;

//variables for loop control
float Kp = 0; 
float totalAngle = 90;  
float distanceTotal = 0 ;

float distanceAccel = 0.25;             //const because acc= 1
float distanceDecel = 0.25; 
float distanceContinuous = 0;         //const because decc= -1

float accelTime = 1 ; 
float decelTime = 1;
float continuousTime = 0; 
float totalTime = 0; 

float t = 0; 
float ref = 0; 
float tolerance = 0.01; 

float numberOfRotations1 = 0; 
float numberOfRotations2 = 0; 

float pos1 = 0; 
float pos2 = 0; 
float pos1Degree = 0;
float pos2Degree = 0; 

float error1 = 0; 
float error2= 0;
float errorTotal1 = 0; 
float errorTotal2 = 0;

void calcul(
  float* Kp, float* continuousTime, float* accelTime, float* decelTime, float* totalTime,
  float* distanceContinuous, float* distanceAccel, float* distanceDecel, float* distanceTotal){

  *distanceTotal = totalAngle * PI/180 * E/2;
  *Kp = KP;
  if(*distanceTotal>0.5){
  *distanceContinuous = *distanceTotal - *distanceAccel - *distanceDecel; 
  *continuousTime = *distanceContinuous/0.5; 
  *totalTime = *accelTime + *decelTime + *continuousTime;
  }
  else{
  *distanceContinuous = 0; 
  *continuousTime = 0; 
  *distanceAccel = *distanceTotal/2;
  *distanceDecel = *distanceTotal/2;
  *accelTime  = sqrt(2 * *distanceAccel/0.5); //acceleration of motors is 0.5 s/m^2
  *decelTime = sqrt(2 * *distanceDecel/(0.5)); 
  *totalTime = *accelTime + *decelTime ;
  }
}

//calculates the distances in meters done by prototype
void calculate_pos1(){
    float pos1_degree = nb_tours1*360 + (POS1CNT+1)/4;      //POSCNT from 0 to 1439 because 4x360 mode 
    pos1 = pos1_degree*2*PI/360*RAYON;                   // 0.041 radius of the wheels 
}

void calculate_pos2(){
    pos2_degree = nb_tours2*360 + (POS2CNT+1)/4;
    pos2 = pos2_degree*2*PI/360*RAYON;
}


void reguler(float csg){
    calculate_pos1(); 
    calculate_pos2(); 
    
    error1 = csg-pos1; 
    error2 = csg-pos2;  
    dc1 = Kp * error1; 
    dc2 = Kp * error2; 
    
}

//defines the input signal
void accel(){
    csg = 0.5*pow(t,2)/2;  ///2.5000e-07
    reguler(csg);  
}

void continu(){
    float t_continue = t-duree_accel; 
    csg = 0.5*t_continue + distance_accel; 
    reguler(csg); 
}

void deccel(){
    float t_deccel = t-duree_accel-duree_continue; 
    if (distance_totale<=0.5){
        csg = -0.5*pow(t_deccel,2)/2 + distance_accel + (0.5*duree_accel)*t_deccel; 
        reguler(csg);
    }
    else{
        csg = -0.5*pow(t_deccel,2)/2 + distance_accel + distance_continue + 0.5*t_deccel; 
        reguler(csg);
    }
     
}

void arrete(){
    //dc1 = 0; 
    //dc2 = 0; 
     /*for (int i = 0; i < 10; i++) { /// Remise à 0 de data
      donne[i] = 0 ;      
     }   
      param = 0 ;
      order = 0 ;
      compteur = 0 ;*/ 
    calculate_pos1(); 
    calculate_pos2(); 
    csg = distance_totale; 
    error1 = csg-pos1; 
    error2 = csg-pos2;  
    dc1 = Kp * error1; 
    dc2 = Kp * error2; 
}



void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
t = t+0.001; 
error_totale1 = distance_totale-pos1; 
error_totale2 = distance_totale-pos2; 
if ( (error_totale1 > seuil) || (error_totale2 > seuil) ){
    if (t<duree_accel && t>0){
       accel();
    }
    else if(t<duree_continue + duree_accel){
      continu(); 
    }

    else if(t>duree_accel +duree_continue){
        deccel(); 
    }
}
else{
    
    arrete(); 
    
}
IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
}


void setup() {

  Serial.begin(9600);

  // configure encoder pins as inputs
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN, INPUT);

  // use interrupt for CLK pin is enough
  // call ISR_encoderChange() when CLK pin changes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), ISR_encoder, RISING);

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
}

void turn_right(){
  //right wheel goes backward 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  //left wheel goes forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turn_left(){
  //right wheel goes forward 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  //left wheel goes  backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}


void move() {

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
  distanceFront = sensorTime2 * 0.034 / 2;

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
  distanceBelow = sensorTime4 * 0.034 / 2;

  //wheels go forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(1000);

  if (distanceFront <= 15 ){
    //if obstacle is on the left and front turn right
    if (distanceLeft <= 15 && distanceRight > 15){
      turn_right(); 
    }
    //if obstacle is on the right and front turn left
    else if (distanceLeft > 15 && distanceRight <= 15){
      turn_left(); 
    }
  }
}


void loop() {
  
 
    Serial.print("DIRECTION: ");
    if (direction == DIRECTION_CW)
      Serial.print("Clockwise");
    else
      Serial.print("Counter-clockwise");

    Serial.print(" | COUNTER: ");
    Serial.println(rotatingCounter);
  move(); 
}

void ISR_encoder() {
  if (digitalRead(DT_PIN) == HIGH) {
    //encoder is rotating counter-clockwise because B high before A:decrease the counter
    rotatingCounter--;
    direction = DIRECTION_CCW;
    
  } else {
    //encoder is rotating clockwise: increase the counter
    rotatingCounter++;
    direction = DIRECTION_CW;
  }
  if(rotatingCounter == 1440){
    numberOfRotations = numberOfRotations + 1; 
  }

}















