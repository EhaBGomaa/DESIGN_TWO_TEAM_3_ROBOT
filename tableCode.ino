#define dirPin 4
#define stepPin 2
#define stepsPerRevolution 200
//#define infrared_table_1 7
#define infrared_table_2 7
#define switchPin 6
//Pin connected to the switch
bool switchState = HIGH;

bool motorActivated = false;  // Variable to track if motor is activated

void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  //pinMode(infrared_table_1, INPUT);
  pinMode(infrared_table_2, INPUT);
   // Set the switch pin as an input
  pinMode(switchPin, INPUT);
  // Enable the internal pull-up resistor for the switch pin
  digitalWrite(switchPin, HIGH);

}

void loop() {
  // Set the spinning direction clockwise:
  int infrared_2 = digitalRead(infrared_table_2);
  //int infrared_1 = digitalRead(infrared_table_1);
  // Read the state of the switch

   switchState = digitalRead(switchPin);

  // If the switch is pressed, turn on the device
  if (switchState == LOW) {
    digitalWrite(dirPin, HIGH);
    
    // Spin the stepper motor 200 steps:
    for (int i = 0; i < stepsPerRevolution; i++) {
      if (infrared_2 == LOW) {
        // Stop motor rotation
        break;
      }
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(2000);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(2000);
    }
  }
    if (switchState == HIGH) {
    digitalWrite(stepPin, LOW);
    }
}
