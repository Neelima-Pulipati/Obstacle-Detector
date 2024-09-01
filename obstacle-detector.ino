#define TRIGGER_PIN_FORWARD 10
#define ECHO_PIN_FORWARD 10
#define TRIGGER_PIN_LEFT 12
#define ECHO_PIN_LEFT 12
#define TRIGGER_PIN_BACKWARD 13
#define ECHO_PIN_BACKWARD 13
#define TRIGGER_PIN_RIGHT 11
#define ECHO_PIN_RIGHT 11

#define MOTOR_SPEED_PIN 3
#define MOTOR_DIR_PIN1 A0
#define MOTOR_DIR_PIN2 A1
#define MOTOR_DIR_PIN3 A2

#define SAFE_DISTANCE 20 // cm

long readUltrasonicDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

void setup()
{
  Serial.begin(9600);
  pinMode(MOTOR_SPEED_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN1, OUTPUT);
  pinMode(MOTOR_DIR_PIN2, OUTPUT);
  pinMode(MOTOR_DIR_PIN3, OUTPUT);
}

void loop()
{
  int distances[4];
  
  // Measure distances in cm
  distances[0] = 0.017 * readUltrasonicDistance(TRIGGER_PIN_FORWARD, ECHO_PIN_FORWARD);
  distances[1] = 0.017 * readUltrasonicDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);
  distances[2] = 0.017 * readUltrasonicDistance(TRIGGER_PIN_BACKWARD, ECHO_PIN_BACKWARD);
  distances[3] = 0.017 * readUltrasonicDistance(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);

  // Print distances
  Serial.println("Distances: Forward: " + String(distances[0]) + 
                 " Left: " + String(distances[1]) + 
                 " Backward: " + String(distances[2]) + 
                 " Right: " + String(distances[3]));

  if (distances[0] > SAFE_DISTANCE) {
    moveForward(distances[0]);
  } else if (distances[1] > SAFE_DISTANCE) {
    turnLeft(distances[1]);
  } else if (distances[3] > SAFE_DISTANCE) {
    turnRight(distances[3]);
  } else if (distances[2] > SAFE_DISTANCE) {
    moveBackward(distances[2]);
  } else {
    stop();
  }

  delay(100);
}

void moveForward(int distance) {
  Serial.println("Moving Forward");
  setMotorDirection(LOW, LOW, HIGH);
  setMotorSpeed(distance);
}

void turnLeft(int distance) {
  Serial.println("Turning Left");
  setMotorDirection(HIGH, LOW, LOW);
  setMotorSpeed(distance);
}

void turnRight(int distance) {
  Serial.println("Turning Right");
  setMotorDirection(LOW, HIGH, HIGH);
  setMotorSpeed(distance);
}

void moveBackward(int distance) {
  Serial.println("Moving Backward");
  setMotorDirection(LOW, HIGH, LOW);
  setMotorSpeed(distance);
}

void stop() {
  Serial.println("Stopped");
  setMotorDirection(LOW, LOW, LOW);
  analogWrite(MOTOR_SPEED_PIN, 0);
}

void setMotorDirection(int dir1, int dir2, int dir3) {
  digitalWrite(MOTOR_DIR_PIN1, dir1);
  digitalWrite(MOTOR_DIR_PIN2, dir2);
  digitalWrite(MOTOR_DIR_PIN3, dir3);
}

void setMotorSpeed(int distance) {
  int speed = map(distance, 0, 1000, 0, 255);
  analogWrite(MOTOR_SPEED_PIN, speed);
}