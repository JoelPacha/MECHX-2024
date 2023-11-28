//This code is used to control a vacuum cleaner prototype developed by Bruface program students in Belgium
//                                               +-----+
//                  +----[PWR]-------------------| USB |--+
//                  |                            +-----+  |
//                  |         GND/RST2  [ ][ ]            |
//                  |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |    
//                  |          5V/MISO2 [ ][ ]  A4/SDA[ ] |    
//                  |                             AREF[ ] |
//                  |                              GND[ ] |
//                  | [ ]N/C                    SCK/13[ ] |   Motor Right int4
//                  | [ ]IOREF                 MISO/12[ ] |   Motor Right int3
//                  | [ ]RST                   MOSI/11[ ]~|   Motor Right PWM
//                  | [ ]3V3    +---+               10[ ]~|   Motor Left PWM
//                  | [ ]5v    -| A |-               9[ ]~|   Motor Left int2
//                  | [ ]GND   -| R |-               8[ ] |   Motor Left int1
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

// ultrasonic sensors output and input 
  const int triggerPin1 = 2;
  const int echoPin1 = 0; 
  const int triggerPin2 = 4;
  const int echoPin2 = 1; 
  const int triggerPin3 = 7;
  const int echoPin3 = 2;
  //int irpin =2;   infrarated sensor for stairs

  //timevariables for each sensors measuring the time the signal takes to come back
  //sensor 1 is left, 2 is front, 3 is right
  long sensorTime1;
  long sensorTime2;
  long sensorTime3;
  int distanceLeft;
  int distanceFront;
  int distanceRight;
  int a=0;

  //right motor input voltage: IN1 = HIGH IN2 = LOW to go forward
  const int IN1 = 8; 
  const int IN2 = 9;
  const int ENA = ;  //PWM of right motor

  //left motor input voltage
  const int IN3 = 12; 
  const int IN4 = 13; 
  const int ENB = 10; //PWM of left motor

void setup() {
  //set the pins as output and input
  pinMode(triggerPin1, OUTPUT);
  pinMode(triggerPin2, OUTPUT);
  pinMode(triggerPin3, OUTPUT);
  pinMode(echoPin1, INPUT); 
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);

  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  //pinMode(irPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}



void loop() {
  int encoder1()
  //continuously checks the sensors for obstacle

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
  sensorTime1 = pulseIn(echoPin2, HIGH);
  distanceFront = sensorTime2 * 0.034 / 2;

  //sensor3
  digitalWrite(triggerPin3, LOW);    //first restart the trigger pin
  delayMicroseconds(2);              //wait of 2µs
  digitalWrite(triggerPin3, HIGH);   
  delayMicroseconds(10);             //sends the ultrasonic wave during 10µs
  digitalWrite(triggerPin3, LOW);
  sensorTime1 = pulseIn(echoPin3, HIGH);
  distanceRight = sensorTime3 * 0.034 / 2;

  //wheels go forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(1000);

  if (distanceFront <= 15 ){
    //if obstacle is on the left and front turn right
    if (distanceLeft <= 15 && distanceRight > 15){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      delay(1000);
    }
    //if obstacle is on the right and front turn left
    else if (distanceLeft > 15 && distanceRight <= 15){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      delay(1000);
    }
    else if (distanceLeft <= 15 && distanceRight <= 15){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      delay(1000);
    }
  }
}
