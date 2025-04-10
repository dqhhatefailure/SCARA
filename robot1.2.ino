#include <AccelStepper.h>
#include <Servo.h>

#define X_STEP     2
#define X_DIR      5
#define Y_STEP     3
#define Y_DIR      6
#define Z_STEP     4
#define Z_DIR      7
#define ENABLE_PIN 8
#define SERVO_PIN  11
#define BUTTON_PIN 12

const float STEPS_PER_DEGREE = 8.89;
const float STEPS_PER_CM = 500.0;
unsigned long previousMillis = 0;  
const long interval = 100; 

AccelStepper stepper_theta1(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepper_theta2(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper stepper_height(AccelStepper::DRIVER, Z_STEP, Z_DIR);
Servo base_servo;

enum SystemState { 
  IDLE,
  MOVING_GRASP,
  GRASPING,
  HOLDING_GRASP,
  MOVING_RELEASE,
  RELEASING,
  RETURNING
};
SystemState currentState = IDLE;

const int GRASP_ANGLE = 60;
const int GRASP_STEP_DELAY = 30;
unsigned long lastStepTime = 0;
int currentServoAngle = 0;
int commandIndex = 0;
float coordinates[6][3];  // 存储六个坐标点

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  stepper_theta1.setMaxSpeed(100);
  stepper_theta1.setAcceleration(50);
  stepper_theta2.setMaxSpeed(100);
  stepper_theta2.setAcceleration(50);
  stepper_height.setMaxSpeed(300);
  stepper_height.setAcceleration(150);

  base_servo.attach(SERVO_PIN);
  base_servo.write(0);
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  
    int rawValue = analogRead(A0);
    float angle = map(rawValue, 0, 1023, -180, 180);
    
    Serial.print("Angle: ");
    Serial.print(angle, 1);
    Serial.println("°");
  }
  switch(currentState){
    case IDLE:
      if(commandIndex >= 6 && allMotorsStopped()){
        returnToOrigin();
        currentState = RETURNING;
      }
      else if(Serial.available()){
        processCommand(Serial.readStringUntil('\n'));
      }
      break;

    case MOVING_GRASP:
      updateMotors();
      if(allMotorsStopped()){
        currentServoAngle = 0;
        currentState = GRASPING;
        lastStepTime = millis();
      }
      break;

    case GRASPING:
      handleGrasping();
      break;

    case HOLDING_GRASP:
      if(millis() - lastStepTime > 1000){
        currentState = IDLE;
        Serial.println("GRASP_COMPLETE");
      }
      break;

    case MOVING_RELEASE:
      updateMotors();
      if(allMotorsStopped()){
        currentServoAngle = GRASP_ANGLE;
        currentState = RELEASING;
        lastStepTime = millis();
      }
      break;

    case RELEASING:
      handleReleasing();
      break;

    case RETURNING:
      updateMotors();
      if(allMotorsStopped()){
        Serial.println("RETURN_COMPLETE");
        commandIndex = 0;
        currentState = IDLE;
      }
      break;
  }
}

void handleGrasping() {
  if(digitalRead(BUTTON_PIN) == LOW){
    currentState = HOLDING_GRASP;
    Serial.println("GRASP_INTERRUPTED");
    return;
  }

  if(millis() - lastStepTime > GRASP_STEP_DELAY){
    if(currentServoAngle < GRASP_ANGLE){
      base_servo.write(++currentServoAngle);
      lastStepTime = millis();
    } else {
      currentState = HOLDING_GRASP;
      lastStepTime = millis();
    }
  }
}

void handleReleasing() {
  if(millis() - lastStepTime > GRASP_STEP_DELAY){
    if(currentServoAngle > 0){
      base_servo.write(--currentServoAngle);
      lastStepTime = millis();
    } else {
      Serial.println("RELEASE_COMPLETE");
      currentState = IDLE;
    }
  }
}

void processCommand(String cmd) {
  int comma1 = cmd.indexOf(',');
  int comma2 = cmd.indexOf(',', comma1 + 1);

  if(comma1 != -1 && comma2 != -1 && commandIndex < 6){
    // 存储坐标点
    coordinates[commandIndex][0] = cmd.substring(0, comma1).toFloat();
    coordinates[commandIndex][1] = cmd.substring(comma1+1, comma2).toFloat();
    coordinates[commandIndex][2] = cmd.substring(comma2+1).toFloat();

    // 根据命令序号确定状态
    if(commandIndex % 2 == 0){
      moveTo(coordinates[commandIndex][0], 
            coordinates[commandIndex][1], 
            coordinates[commandIndex][2]);
      currentState = MOVING_GRASP;
    } else {
      moveTo(coordinates[commandIndex][0], 
            coordinates[commandIndex][1], 
            coordinates[commandIndex][2]);
      currentState = MOVING_RELEASE;
    }
    commandIndex++;
  }
}

void moveTo(float theta1, float height, float theta2) {
  stepper_theta1.moveTo(theta1 * STEPS_PER_DEGREE * 0.66);
  stepper_height.moveTo(height * STEPS_PER_CM * 1.2);
  stepper_theta2.moveTo(theta2 * STEPS_PER_DEGREE * 0.2);
}

void returnToOrigin() {
  stepper_theta1.moveTo(0);
  stepper_height.moveTo(0);
  stepper_theta2.moveTo(0);
}

void updateMotors() {
  stepper_theta1.run();
  stepper_height.run();
  stepper_theta2.run();
}

bool allMotorsStopped() {
  return !stepper_theta1.isRunning() && 
         !stepper_height.isRunning() && 
         !stepper_theta2.isRunning();
}