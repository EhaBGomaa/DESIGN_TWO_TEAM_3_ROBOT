// Motor A connections
int enA = 10;
int in1 = 9;
int in2 = 8;

// Motor B connections
int enB = 5;
int in3 = 7;
int in4 = 6;

// IR line follower pins
int leftIR = 12;
//int middleIR = 3;
int rightIR = 11;
int infraTable = 13;

//Ultrasonic 1 pins
int trig1 = A1;
int echo1 = A2;

//Ultrasonic 2 pins
int trig2 = A3;
int echo2 = A4;

int buzzer = 4; 
void setup() {
  // Set motor control pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set IR line follower pins as inputs
  pinMode(leftIR, INPUT);
 // pinMode(middleIR, INPUT);
  pinMode(rightIR, INPUT);

  pinMode(infraTable, INPUT);
 // Ultrasonic 1 pins 
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  // Ultrasonic 2 pins 
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  
  pinMode(buzzer, OUTPUT);
}

void loop() {
  int leftSensor = digitalRead(leftIR);
  //int middleSensor = digitalRead(middleIR);
  int rightSensor = digitalRead(rightIR);

  int tableIR = digitalRead(infraTable);

 if (checkdistance1() > 20 && checkdistance2() > 20  ) {
  // All sensors on white surface
  if (leftSensor == HIGH && rightSensor == HIGH) {
    // Move forward
    moveForward();
  }

  // Left sensor on black line
  else if (leftSensor == LOW && rightSensor == HIGH && tableIR == HIGH ) {
    // Turn left
    turnLeft();
  }
  // Right sensor on black line
  
  else if (rightSensor == LOW && leftSensor == HIGH && tableIR == HIGH ) {
    // Turn right
    turnRight();
  }
else if (rightSensor == LOW && leftSensor == LOW && tableIR == HIGH )
 {
     digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(enA, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(enB,LOW);
    }
  else if (tableIR == LOW && rightSensor == LOW && leftSensor == LOW)
{
  moveForward();
}
}

else {
 stop();
 tone(buzzer, 1000); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(1000);        // ...for 1sec
 }

}

// Function to move the robot forward
void moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 230);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 230);
}

// Function to turn the robot left
void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 0);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 230);
}

// Function to turn the robot right
void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 230);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 0);
} 

// Check Distance of Ultrasonic 1
float checkdistance1() {
  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  float distance = pulseIn(echo1, HIGH) / 58.00;
  delay(10);
  return distance;
}

// Check Distnace of Ultrasonic 2
float checkdistance2() {
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);
  float distance = pulseIn(echo2, HIGH) / 58.00;
  delay(10);
  return distance;
}


void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}
