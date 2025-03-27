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

const float STEPS_PER_DEGREE = 8.89;
const float STEPS_PER_CM = 500.0;

AccelStepper stepper_theta1(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepper_theta2(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper stepper_height(AccelStepper::DRIVER, Z_STEP, Z_DIR);
Servo base_servo;

enum SystemState { IDLE, MOVING, SERVO_MID, RETURNING, SERVO_HOME };
SystemState currentState = IDLE;
unsigned long servoStartTime = 0;
float initial_theta1 = 0, initial_theta2 = 0, initial_height = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

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
  switch(currentState){
    case IDLE:
      if(Serial.available()){
        String cmd = Serial.readStringUntil('\n');
        processCommand(cmd);
        currentState = MOVING;
      }
      break;
      
    case MOVING:
      updateMotors();
      if(allMotorsStopped()){
        servoStartTime = millis();
        base_servo.write(45);
        currentState = SERVO_MID;
      }
      break;
      
    case SERVO_MID:
      if(millis() - servoStartTime > 2000){
        returnToOrigin();
        currentState = RETURNING;
      }
      break;
      
    case RETURNING:
      updateMotors();
      if(allMotorsStopped()){
        servoStartTime = millis();
        base_servo.write(0);
        currentState = SERVO_HOME;
      }
      break;
      
    case SERVO_HOME:
      if(millis() - servoStartTime > 2000){
        currentState = IDLE;
        Serial.println("Cycle Complete");
      }
      break;
  }
}

// 修复后的运动控制函数
void moveTo(float theta1, float height, float theta2) {
  // 应用校准系数到步数计算
  stepper_theta1.move(theta1 * STEPS_PER_DEGREE * 0.66);
  stepper_height.move(height * STEPS_PER_CM * 1.5);
  stepper_theta2.move(theta2 * STEPS_PER_DEGREE * 0.2);
}

void processCommand(String cmd) {
  int comma1 = cmd.indexOf(',');
  int comma2 = cmd.indexOf(',', comma1 + 1);

  if(comma1 != -1 && comma2 != -1){
    initial_theta1 = cmd.substring(0, comma1).toFloat();
    initial_height = cmd.substring(comma1+1, comma2).toFloat();
    initial_theta2 = cmd.substring(comma2+1).toFloat();

    moveTo(initial_theta1, initial_height, initial_theta2);
  }
}

void returnToOrigin() {
  stepper_theta1.move(-stepper_theta1.currentPosition());
  stepper_height.move(-stepper_height.currentPosition());
  stepper_theta2.move(-stepper_theta2.currentPosition());
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