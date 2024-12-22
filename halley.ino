#include <QTRSensors.h>

int ENA = 1;
int IN1 = 6;
int IN2 = 5;
int IN3 = 4;
int IN4 = 3;
int ENB = 2;

QTRSensorsRC qtrrc((unsigned char[]) {A8, A9, A10, A11, A12, A13, A14, A15}, 8);
bool s9;

void setup() {
  Serial.begin(9600);
  qtrrc.calibrate();
  delay(500);
  
  pinMode(A0, INPUT);   // Extra sensor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); // Motor control
  pinMode(ENB, OUTPUT);
}

void loop() {
  unsigned int sensorValues[8];
  qtrrc.read(sensorValues);
  s9 = digitalRead(A0); // Check for extra sensor
  
  // Check if the center sensors detect the line
  bool forward = (sensorValues[3] <= 204) && (sensorValues[4] <= 204);

  // Direction logic based on sensor values
  int direction = (sensorValues[0] <= 200) * 1+(sensorValues[1] <= 200) * 1 + (sensorValues[2] <= 200) * 1 
                - (sensorValues[5] <= 200) * 1 - (sensorValues[6] <= 200) * 1-(sensorValues[7] <= 200) * 1;

  // Stop condition if all sensors are off the line
  if (allSensorsOff(sensorValues)) {
    stop();
    Serial.println("Stop");
  } 
  // Stop if external sensor is triggered
  else if (s9 == 0) {
    stop();
    Serial.println("Stop Goz");
  } 
  // Move forward if center sensors detect the line
  else if (sensorValues[3] <= 200 && sensorValues[4] <= 200) {
    Serial.println("Yardirrrr!!");
    Forward();
  } 
  // Turn left if direction is positive
  else if (direction >= 1) {
    left();
    Serial.println("Left");
  } 
  // Turn right if direction is negative
  else if (direction <= -1) {
    right();
    //Serial.println("Right");
  } 
  // Default to forward movement
  else if (allSensorsOn(sensorValues)){
    Forward();
    Serial.println("DÜMDÜK!");
  }
  delay(100); // Shorter delay for better response
}

void Forward() {
 digitalWrite(IN1, LOW);
  digitalWrite(IN3, 1);
  digitalWrite(IN2, 1);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 80);  // Speed control using PWM
  analogWrite(ENB, 80);  
} 

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);   // Slow down left motor
  analogWrite(ENB, 80);  // Normal speed on the right motor
}

void right() {
  Serial.println("Right");
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);  // Speed control using PWM
  analogWrite(ENB, 80);  
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);    // Stop the motors
  analogWrite(ENB, 0);
}

// Function to check if all sensors are off the line
bool allSensorsOff(unsigned int sensorValues[]) {
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] <= 250) {
      return false;  // Line detected
    }
  }
  return true;  // No line detected
}
bool allSensorsOn(unsigned int sensorValues[]) {
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > 250) {
      return false;  // Line detected
    }
  }
  return true;  // No line detected
}