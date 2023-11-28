int analogPin = 3;     
int data = 0;           
char userInput;
int a = 0;
void setup(){

  Serial.begin(9600);                        //  setup serial
}

void loop(){

if(Serial.available()> 0){ 
    
    userInput = Serial.read();               // read user input
      
      if(userInput == 'g'){                  // if we get expected value 
            if(a == 1000){
                a = 0;
            }
            else{
                a = a+1;  
            }
            data = analogRead(analogPin);    // read the input pin
            Serial.println(a);            
            
      } // if user input 'g' 
  } // Serial.available
} // Void Loop
