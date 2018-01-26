/* This is a simple project using RGB LED as blinkeing Led,
a Potentimeter
a Servo that will be rotating at a given rate 
a PhotoSensor that controls the power of the lights
Two switches to control which light should blink

*/

#include<Servo.h> // Loading the servo Library

int pos = 0; // declare and initialize the position variable
int servoPin = 3; // connecting the servo to pin 9
int servoDelay = 25;
int maxDegree  = 180 ; // maximum angle 
int minDegree  = 0; // minimum angle

Servo myPointer; // Create a sevo object called myPointer

int redLEDpin = 6; // Set red LED pin to 6
int greenLEDpin = 9; // Set green LED pin to 9
int blueLEDpin = 10; // Set blue LED pin to 10

int s1Pin = 12; // Switch 1 connect to pin 11
int s2Pin = 13; // Switch 2 connect to pin 3

int switch1Value; // Value of switch 1
int switch2Value; // Value of switch 2

int potPin  = A0; // potentiometer pin set to A0
int potValue ;  // value of the potentiometer

int SensorTempPin = A1; // Sensor connected to pin A1
int SensorValue ;  // Value the sensor
int PhotoSensorPin  = A3; // Photo Sensor connected to A3
int PhotoSensorValue;

float TempVoltage; // Voltage for the temperature
float degreesC ; // Temperature in degree Celcius
float degreesF ; // Temperature in degree Farenheit
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(redLEDpin, OUTPUT); // setting red redLEDpin as output
  pinMode(greenLEDpin, OUTPUT); // setting greenLEDpin as ouput  
  pinMode(blueLEDpin, OUTPUT); // setting blueLEDpin as output 
  
  pinMode(s1Pin, INPUT);  // setting s1Pin as input
  pinMode(s2Pin, INPUT);  // setting s2Pin as input
  pinMode(potPin, INPUT); // setting potentiometer pin as input

  pinMode(SensorTempPin , INPUT); // Setting SensortempPin as input

  pinMode(PhotoSensorPin , INPUT); // Setting PhotoSensorPin as input

  myPointer.attach(servoPin);

  myPointer.write(pos);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Serial.println(" Where would you like to position the servo");
  // while(Serial.available()==0){};
  // pos = Serial.parseInt();
  /// myPointer.write(pos);
  // servoRotation();

   
   readSwitches();
   
  // Information from the potentiometer
   readPotentiometer();
  // Information for the temperature sensor
   readTempSensor();
  
  // Information from the photo Sensor
   readPhotoSensor();
  
  //Serial.println("");
  //Serial.println(PhotoSensorValue);
  
 
  
  //SensorValue = map(SensorValue , 0,1023, 0, 255);
  //SensorValue = constrain(SensorValue, 0, 255);
 // Serial.println(SensorValue);
  Serial.println("");
 // Serial.print(degrees
  if( switch1Value == LOW && switch2Value == LOW) // Both switches are off
  {
     analogWrite(greenLEDpin, LOW); // Turn green LED off
     analogWrite(blueLEDpin, LOW); // Turn blue LED off
     analogWrite(redLEDpin, potValue); // Send a signal to turn the red LED on
     delay(1000);

  }

  if( switch1Value == HIGH && switch2Value == LOW ) // Switch 1 is on and Switch 2 is off
  {
     analogWrite(redLEDpin, LOW); // Turn red LED off
     analogWrite(blueLEDpin, LOW); // Turn blue LED off
     analogWrite(greenLEDpin, PhotoSensorValue); // Send a signal to turn the green LED on
     delay(1000);

  }

  if( switch1Value == LOW && switch2Value == HIGH) // Switch 1 is off and Switch 2 is on
  {
     analogWrite(redLEDpin, LOW); // Turn red LED off
     analogWrite(greenLEDpin, LOW); // Turn green LED off
     analogWrite(blueLEDpin, PhotoSensorValue); // Send a signal to turn the blue LED on
     delay(1000);

  }
  if( switch1Value == HIGH && switch2Value == HIGH && (SensorValue)) // Both switches are on
  {
     // Turn all the LEDs on
     analogWrite(greenLEDpin, HIGH); // Turn green LED on
     analogWrite(blueLEDpin, HIGH); // Turn blue LED on
     analogWrite(redLEDpin, HIGH); // Turn red LED on
     delay(1000);

  }

  delay(1000);
  
}
void servoRotation(){

 // potValue = analogRead(potPin);
 // potValue =(int) (potValue/1023.)*maxDegree;
  while(pos < maxDegree) {
    pos +=5;
    myPointer.write(pos);
    delay(servoDelay);
  }
  while(pos>minDegree){
    myPointer.write(pos);
    pos -=5;
    delay(servoDelay);
  }
    
  }
void readPotentiometer(){
  
  potValue     = analogRead(potPin); // reading the voltage at potPin 
  potValue     = map(potValue, 0 , 1023, 0 , 255);
  potValue     = constrain(potValue, 0, 255);

  }
void readTempSensor(){
  
  SensorValue  = analogRead(SensorTempPin); // reading information provide by the sensor
  //SensorValue = map(potValue, 0 , 1023, 0 , 255);
  //SensorValue = constrain(SensorValue, 0, 255);

 TempVoltage = SensorValue * 5.0;
 TempVoltage /= 1024.0; 
 
 // print out the voltage
 Serial.print(TempVoltage); Serial.println(" volts");
 
 // now print out the temperature
 degreesC = (TempVoltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                               //to degrees ((voltage - 500mV) times 100)
 Serial.print(degreesC); Serial.println(" degrees C");
 
 // now convert to Fahrenheit
 degreesF = (degreesC * 9.0/5.0) + 32.0;
 Serial.print(degreesF); Serial.println(" degrees F");
 
 delay(1000); 
  }
void readPhotoSensor(){
  PhotoSensorValue = analogRead(PhotoSensorPin); // reading information provided by the photo sensor
  PhotoSensorValue = map(PhotoSensorValue, 0 , 1023, 0 , 255);
  PhotoSensorValue = constrain(PhotoSensorValue, 0, 255);
  }
void readSwitches(){
  switch1Value = digitalRead(s1Pin); // reading the value at the switch 1
  switch2Value = digitalRead(s2Pin); // reading the value at the switch 2

  }
 void TurnOnRed(){}
 void TurnOnGreen(){}
 void TurnOnBlue(){} 
