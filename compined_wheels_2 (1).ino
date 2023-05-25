
// Pins
#define ENCA 2
#define ENCB 3
#define PWM 10
#define IN1 9
#define IN2 8
#define PWMB 5
#define IN3 7
#define IN4 6
#define ENCA2 19
#define ENCB2 18
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

long prevT = 0;
int posPrev = 0;

// used in an interrupt
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;

float eintegral = 0;

volatile int pos_i_2 = 0;
volatile float velocity_i_2 = 0;
volatile long prevT_i_2 = 0;

float v1Filt_2 = 0;
float v1Prev_2 = 0;

float eintegral_2 = 0;

void setup() {
  Serial.begin(9600);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(ENCA2,INPUT);
  pinMode(ENCB2,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
   pinMode(PWMB,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2),
                  readEncoder_2,RISING);
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
  Serial.begin(9600);
}

void loop() {
 
  // read the position and velocity
  int pos = 0;
  float velocity2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos = pos_i;
  velocity2 = velocity_i;
  interrupts(); // turn interrupts back on
  

  // Compute velocity with method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  int pos_2 = 0;
  int posPrev_2 = 0;
  int prevT_2 = 0;
  float velocity2_2 = 0;
  noInterrupts(); // disable interrupts temporarily while reading
  pos_2 = pos_i_2;
  velocity2_2 = velocity_i_2;
  interrupts(); // turn interrupts back on

  // Compute velocity with method 1
  long currT_2 = micros();
  float deltaT2 = ((float) (currT_2-prevT_2))/1.0e6;
  float velocity1_2 = (pos_2 - posPrev_2)/deltaT2;
  posPrev_2 = pos_2;
  prevT_2 = currT_2;

  // Convert count/s to RPM
  float v1 = velocity1/770.0*60.0;


  // Low-pass filter (25 Hz cutoff)
  v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  v1Prev = v1;
  
  // Convert count/s to RPM
  float v1_2 = velocity1_2/770.0*60.0;


  // Low-pass filter (25 Hz cutoff)
  v1Filt_2 = 0.854*v1Filt_2 + 0.0728*v1_2 + 0.0728*v1Prev_2;
  v1Prev_2 = v1_2;

  // Set a target
  float vt = 60;
  float vt_2 = 60;
  // Compute the control signal u
  float kp = 50;
  float ki = 70;
  float e = vt-v1Filt;
  eintegral = eintegral + e*deltaT;
  
  // Compute the control signal u
  float kp_2 = 50;
  float ki_2 = 70;
  float e_2 = vt_2-v1Filt_2;
  eintegral_2 = eintegral_2 + e_2*deltaT2;
  float u = kp*e + ki*eintegral;
  float u_2 = kp_2*e_2 + ki_2*eintegral_2;
  // Set the motor speed and direction
  int dir = 1;
 if (u<0){
    dir =1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  int pwr_2 = (int) fabs(u_2);
  if(pwr_2 > 255){
    pwr_2 = 255;
  }
  setMotor(dir,pwr,PWM,IN1,IN2);
  setMotor(dir,pwr_2,PWMB,IN3,IN4);

  Serial.print(vt);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
  Serial.print(vt_2);
  Serial.print(" ");
  Serial.print(v1Filt_2);
  Serial.println();
  delay(1);
  
   int leftSensor = digitalRead(leftIR);
  //int middleSensor = digitalRead(middleIR);
  int rightSensor = digitalRead(rightIR);
  int tableIR = digitalRead(infraTable);
 if (Serial.available()) {
    char commandpre = Serial.read();
    char commandcurr = '0';
  if (commandcurr != commandpre)
{
  commandcurr = commandpre;
}
if(commandcurr == '0')
{
 if (checkdistance1() > 20 && checkdistance2() > 20  ) {
  // All sensors on white surface
  if (leftSensor == HIGH && rightSensor == HIGH) {
    // Move forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, pwr);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(PWMB, pwr_2);
  }

  // Left sensor on black line
  else if (leftSensor == LOW && rightSensor == HIGH && tableIR == HIGH) {
    // Turn left
    digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, pwr);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(PWMB, 0);
  }
  // Right sensor on black line
  
  else if (rightSensor == LOW && leftSensor == HIGH && tableIR == HIGH) {
    // Turn right
       digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, 0);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(PWMB, pwr_2);
  }
else if (rightSensor == LOW && leftSensor == LOW && tableIR == HIGH)
 {
     digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(PWM, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(PWMB,LOW);
    }
   else if (tableIR == LOW && rightSensor == LOW && leftSensor == LOW)
{
      // Move forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, pwr);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(PWMB, pwr_2);
}
}
}
else if (commandcurr == '1' || commandcurr == '2' || commandcurr == '3')
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(PWM, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(PWMB,LOW);
}
else {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(PWM, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(PWMB,LOW);

 tone(buzzer, 1000); // Send 1KHz sound signal...
  delay(1000);        // ...for 1 sec
  noTone(buzzer);     // Stop sound...
  delay(1000);        // ...for 1sec
 }
 }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == -1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == 1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i = pos_i + increment;
}

void readEncoder_2(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB2);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  pos_i_2 = pos_i_2 + increment;
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
