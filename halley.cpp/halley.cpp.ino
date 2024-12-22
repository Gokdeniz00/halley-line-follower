#include <QTRSensors.h>

// QTR-8A sensor setup
#define NUM_SENSORS 8         // Number of sensors in QTR-8A
#define TIMEOUT 2500          // Timeout for sensor reading
#define EMITTER_PIN 2         // Pin to control IR emitter (optional)

QTRSensorsAnalog qtrrc((unsigned char[]) {A8, A9, A10, A11, A12, A13, A14, A15}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Motor pins
#define ENA A1     // Speed control for left motor
#define IN1 6      // Direction control for left motor
#define IN2 5
#define ENB A2     // Speed control for right motor
#define IN3 4      // Direction control for right motor
#define IN4 3

// Obstacle detection sensor pin
#define IR_SENSOR A0

// Motor control function
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  analogWrite(ENA, abs(leftSpeed));
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  // Right motor
  analogWrite(ENB, abs(rightSpeed));
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 400; i++) {  // Make 400 readings (adjust based on need)
  qtrrc.calibrate();              // Call calibration method
  delay(20); 
  }
  Serial.println("Calibrated");
  // Motor pin setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Obstacle sensor pin setup
  pinMode(IR_SENSOR, INPUT);

  // Optional emitter pin setup for QTR sensor (if using it)
  pinMode(EMITTER_PIN, OUTPUT);
}

void loop() {
  // Read QTR sensor values
  qtrrc.readCalibrated(sensorValues);
  int position = qtrrc.readLine(sensorValues);  // Returns position of line (2000 to 7000)
  
  // Obstacle detection
  int irValue = analogRead(IR_SENSOR);
  if (!irValue) {   // Adjust threshold based on testing
    // Obstacle detected, stop motors
    setMotorSpeed(0, 0);
    return;
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
  Serial.print(sensorValues[i]);
  Serial.print(" ");
 }
 Serial.println(); 
  // Line following logic
  int error = position - 3500;   // Error calculation (middle position is 3500)

  // Motor speed calculation based on error
  int baseSpeed = 60;           // Adjust base speed as needed
  int Kp = 0.2;                  // Proportional control constant
  int correction = Kp * error;   // Simple proportional controller

  // Set motor speeds
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  setMotorSpeed(leftSpeed, rightSpeed);

  // Optional: Debugging sensor values and motor speeds
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);

  delay(550);   // Delay to slow down loop, adjust as needed
}
