
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
  const int ENA = 9;  //PWM of right motor

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
}

void loop() {

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

















