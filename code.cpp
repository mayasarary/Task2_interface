#include <LiquidCrystal.h>

const int motorPin1 = 9;    // Motor driver input 1
const int motorPin2 = 10;   // Motor driver input 2
const int encoderPinA = 2;  // Encoder output A
const int encoderPinB = 3;  // Encoder output B
const int potX = A0;        // Potentiometer X
const int ledPin = 11;      // LED indicator

// LCD pin definitions
 const int rs = 13;   // Register select pin
 const int en = 12;   // Enable pin
 const int d4 = 4;   // Data pin 4
 const int d5 = 6;  // Data pin 5
 const int d6 = 7;  // Data pin 6
 const int d7 = 8;  // Data pin 7


volatile long encoderPosition = 0;
int motorSpeed = 0;
int motorDirection = 1; // 1 for forward (CW), -1 for reverse (CCW)
volatile int lastEncoded = 0; // Last encoded value for encoder

LiquidCrystal lcd(13, 12, 4, 6, 7, 8);

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT); // Configure pin 11 as an output for the LED

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  lcd.begin(16, 2);
  lcd.print("Initializing...");
  delay(1000);
}

void loop() {
  int potValX = analogRead(potX);

  if (potValX < 512) {
    motorDirection = 1; // Clockwise (CW)
    motorSpeed = map(potValX, 0, 512, 255, 0);
  } else if (potValX > 512) {
    motorDirection = -1; // Counter-clockwise (CCW)
    motorSpeed = map(potValX, 512, 1023, 0, 255);
  } else {
    motorSpeed = 0; // Stop
  }

  if (motorSpeed > 0) {
    digitalWrite(ledPin, HIGH); // Turn on the LED when motor is active
    if (motorDirection == 1) {
      analogWrite(motorPin1, motorSpeed);
      analogWrite(motorPin2, 0);
    } else {
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, motorSpeed);
    }
  } else {
    digitalWrite(ledPin, LOW); // Turn off the LED when motor is inactive
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
  }

  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(motorSpeed);
  lcd.print("   "); // Clear the rest of the line

  lcd.setCursor(0, 1);
  lcd.print("Pos: ");
  lcd.print(encoderPosition);
  lcd.print(" ");
  
  lcd.print((motorDirection == 1) ? "CW" : "CCW"); // Print direction
  
  lcd.print("   "); // Clear the rest of the line

  delay(100); // Add a small delay to avoid flickering
}

void updateEncoder() {
  int MSB = digitalRead(encoderPinA); // MSB = most significant bit
  int LSB = digitalRead(encoderPinB); // LSB = least significant bit

  int encoded = (MSB << 1) | LSB; // Converting the 2 pin value to a single number
  int sum = (lastEncoded << 2) | encoded; // Adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPosition++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPosition--;

  lastEncoded = encoded; // Store this value for next time
}
