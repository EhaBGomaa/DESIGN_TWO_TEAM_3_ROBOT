// Pin connected to the IR sensor
const int irPin = 7;
int value = 0;
void setup() {
  Serial.begin(9600);
  pinMode(irPin, INPUT);
}

void loop() {
  // Read the state of the IR sensor
  int irState = digitalRead(irPin);

  // Check if the IR sensor reads LOW
  if (irState == LOW) {
    value = 2;
    Serial.println(value);
  }

  delay(100); // Delay for stability
}
