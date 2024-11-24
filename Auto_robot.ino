// Motor control pins
#define motor1Pin1 3
#define motor1Pin2 4
#define motor2Pin1 5
#define motor2Pin2 6

// IR sensor pins
#define leftSensor A0
#define rightSensor A1

// Motor speed (PWM)
#define maxSpeed 255
#define minSpeed 100

// PID control variables
float Kp = 2.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 1.0;  // Derivative gain

float previousError = 0;
float integral = 0;
float error = 0;
float pidValue = 0;

// Setpoint (center of the line)
float setPoint = 0;

// Last sensor readings
int leftReading = 0;
int rightReading = 0;

void setup() {
  // Set motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Set IR sensor pins as inputs
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  Serial.begin(9600);  // For debugging
}

void loop() {
  // Read the sensors
  leftReading = analogRead(leftSensor);
  rightReading = analogRead(rightSensor);

  // Convert the readings to either 0 (no line) or 1 (line detected)
  int left = (leftReading < 500) ? 1 : 0;  // Line detected on left
  int right = (rightReading < 500) ? 1 : 0; // Line detected on right

  // Calculate error based on sensor readings
  // Setpoint is the middle of the robot (center of the line)
  error = left - right;  // 1 if the left sensor detects the line, -1 if right does

  // Calculate integral and derivative for PID
  integral += error;
  float derivative = error - previousError;

  // Calculate PID output
  pidValue = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speeds based on PID value
  int leftSpeed = maxSpeed - pidValue;
  int rightSpeed = maxSpeed + pidValue;

  // Ensure speeds are within range
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  // Apply motor speeds
  moveMotors(leftSpeed, rightSpeed);

  // Store previous error for next iteration
  previousError = error;

  // Debugging (optional)
  Serial.print("Left Sensor: ");
  Serial.print(leftReading);
  Serial.print(" Right Sensor: ");
  Serial.print(rightReading);
  Serial.print(" PID: ");
  Serial.println(pidValue);

  delay(50);  // Small delay for stability
}

// Function to control the motors based on speeds
void moveMotors(int leftSpeed, int rightSpeed) {
  // Left motor control
  if (leftSpeed > 0) {
    analogWrite(motor1Pin1, leftSpeed);
    analogWrite(motor1Pin2, 0);
  } else {
    analogWrite(motor1Pin1, 0);
    analogWrite(motor1Pin2, -leftSpeed);
  }

  // Right motor control
  if (rightSpeed > 0) {
    analogWrite(motor2Pin1, rightSpeed);
    analogWrite(motor2Pin2, 0);
  } else {
    analogWrite(motor2Pin1, 0);
    analogWrite(motor2Pin2, -rightSpeed);
  }
}
