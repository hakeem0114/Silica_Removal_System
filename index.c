//Capstone Design

#include <LiquidCrystal.h>
#include <PID_v1.h>
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to

//Ultrasonic sensor
const int trigPin = 0;  echoPin = 1;
float duration;
float distanceCM, distanceINCH ;
float  distanceM;

//PID controller
#define PIN_INPUT 0
#define PIN_OUTPUT 1 
//It is connected to the tx/rx pins so the code is uploaded before the wire is connected

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=98, Ki=58.5, Kd=28.2 //Variables modified to achieve the best calibration
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// Motor Wheel connections (rear)
int enA = A0;// Initialize pin A0 for Enable pin
int in1 = A1; // Initialize pin A1 for Input1
int in2 = A2;// Initialize pin A2 for Input2

// Motor Wheel connections (rear)
int enB = A0;
int in3 = A1; 
int in4 = A2;

// Motor Wheel connections 
int enC = A3;
int in5 = A4;
int in6 = A5;

// Motor Wheel connections
int enD = A3;
int in7 = A4; 
int in8 = A5;

// Motor Wheel connections (front)
int enE = 2;
int in9 = 3; 
int in10 = 4;

// Motor Wheel connections (front)
int enF = 5;
int in11 = 6;
int in12 = 7;


// Motor Brush connections
int enG = 8;
int in13 = 9; 
int in14 = 10;

// Motor Brush connections
int enH = 11;
int in15 = 12;
int in16 = 13;

int i;
int numBlinks; // variable to store RPM


void setup() {

  Serial.begin(9600);
  Serial.println(" DC Motor simulation");
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
      pinMode(enC, OUTPUT);
        pinMode(enD, OUTPUT);
          pinMode(enE, OUTPUT);
            pinMode(enF, OUTPUT);
              pinMode(enG, OUTPUT);
                pinMode(enH, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);
  pinMode(in9, OUTPUT);
  pinMode(in10, OUTPUT);
  pinMode(in11, OUTPUT);
  pinMode(in12, OUTPUT);
  pinMode(in13, OUTPUT);
  pinMode(in14, OUTPUT);
  pinMode(in15, OUTPUT);
  pinMode(in16, OUTPUT);


  
  //Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
    digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
    digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
    digitalWrite(in9, LOW);
  digitalWrite(in10, LOW);
    digitalWrite(in11, LOW);
  digitalWrite(in12, LOW);
    digitalWrite(in13, LOW);
  digitalWrite(in14, LOW);
    digitalWrite(in15, LOW);
  digitalWrite(in16, LOW);
  
  lcd.begin(16, 2);
  // clear old screen data
  lcd.clear();
  // text to be displayed on the screen
  lcd.print(" Starting!! ");
  delay(2000);

  //PID variable initialization
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;

  //turns the PID on
  myPID.SetMode(AUTOMATIC); //Auto-calibrates


  Serial.begin(9600); //sets serial port for communication
}

void loop() {
  //directionControl();
  //delay(1000);

  int speed = map(analogRead(A0), 0, 1024, 0, 255);
  analogWrite (enA, speed);
   analogWrite (enB, speed);
    analogWrite (enC, speed);
     analogWrite (enD, speed);
      analogWrite (enE, speed);
       analogWrite (enF, speed);
        analogWrite (enG, speed);
         analogWrite (enH, speed);
  lcd.begin(16, 2);
  // clear old screen data
  lcd.clear();

//Distance measurements
  distanceCM = duration*0.034/2;
  distanceM = distanceCM/ 100; 
  distanceINCH = duration*0.0133/2;

     //Calibrates the analog values
    Input = analogRead(PIN_INPUT);
    myPID.Compute();
    analogWrite(PIN_OUTPUT, Output);

      
  // text to be displayed on the screen
  lcd.print("DC MOTOR SPEED ");
  delay(1000);
  speedControl();
  brushSpeed();
  delay(1000);

  //Moves around obstruction / stops an accident
  if (distanceCM < 4 ) //If distance = 4cm
   {
    
      /////////////////////////////////////////////////////////////////////
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Device is stopping!!");
      lcd.print("Recalibrating!!");
      speedControl(); /
      delay(10);
      ///////////////////////////////////////////////////////////////////
       
   }
   
}

void speedControl() {
  // Turn on motors
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, HIGH);
    digitalWrite(in5, HIGH);
  digitalWrite(in6, HIGH);
    digitalWrite(in7, HIGH);
  digitalWrite(in8,HIGH);
    digitalWrite(in9, HIGH);
  digitalWrite(in10, HIGH);
    digitalWrite(in11, HIGH);
  digitalWrite(in12, HIGH);
  int speed = lcd.print("");
  // Accelerate from zero to maximum speed
  for (i = 0; i < 255; i++) {
    analogWrite(enA, i);
    lcd.clear();
    lcd.print("DC MOTOR SPEED ");
    lcd.setCursor(4, 2);
    lcd.print("Speed=");
    lcd.print(i);
    delay(200);
  }

  // Decelerate from maximum speed to zero
  for ( i = 255; i >= 0; --i) {
    analogWrite(enA, i);
    lcd.clear();
    lcd.print("DC MOTOR SPEED ");
    lcd.setCursor(4, 2);
    lcd.print("Speed=");
    lcd.print(i);
    delay(200);
  }

  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
    digitalWrite(in5, LOW);
  digitalWrite(in6, LOW);
    digitalWrite(in7, LOW);
  digitalWrite(in8, LOW);
    digitalWrite(in9, LOW);
  digitalWrite(in10, LOW);
    digitalWrite(in11, LOW);
  digitalWrite(in12, LOW);
}

void brushSpeed() //Speed of brush motors
{
  Serial.println("Enter speed of brush motors"); //Prompt User for Input
  while (Serial.available() == 0) {
    // Wait for User to Input Data
  }
  numBlinks = Serial.parseInt(); //Read the data the user has input
  for (int counter = 1; counter <= numBlinks; counter++) {
    
  }
  Serial.print("Chosen speed is = ");
  Serial.println(numBlinks);
  Serial.println(" ");

    // Turn on motors
      digitalWrite(in13, LOW);
            digitalWrite(in13, HIGH);
  digitalWrite(in14, LOW);
        digitalWrite(in14, HIGH);
    digitalWrite(in15, LOW);
            digitalWrite(in15, HIGH);
  digitalWrite(in16, LOW);
          digitalWrite(in16, HIGH);
  
  int speed = lcd.print("");
  // Accelerate from zero to maximum speed
  for (i = 0; i < numBlinks; i++) {
    analogWrite(enA, i);
    lcd.clear();
    lcd.print("DC MOTOR SPEED ");
    lcd.setCursor(4, 2);
    lcd.print("Speed=");
    lcd.print(i);
    delay(200);
  }

  // Decelerate from maximum speed to zero
  for ( i = numBlinks; i >= 0; --i) {
    analogWrite(enA, i);
    lcd.clear();
    lcd.print("DC MOTOR SPEED ");
    lcd.setCursor(4, 2);
    lcd.print("Speed=");
    lcd.print(i);
    delay(200);
  }

  // Now turn off motors
    digitalWrite(in13, LOW);
  digitalWrite(in14, LOW);
    digitalWrite(in15, LOW);
  digitalWrite(in16, LOW);
}
