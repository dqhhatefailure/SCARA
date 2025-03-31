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
#define BUTTON_PIN 12  // 新增安全按钮引脚

const float STEPS_PER_DEGREE = 8.89;
const float STEPS_PER_CM = 500.0;

AccelStepper stepper_theta1(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepper_theta2(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper stepper_height(AccelStepper::DRIVER, Z_STEP, Z_DIR);
Servo base_servo;

// 状态机增强
enum SystemState { 
  IDLE, 
  MOVING,
  SERVO_GRASPING,  // 新增抓取状态
  SERVO_HOLDING,    // 新增保持状态
  RETURNING, 
  SERVO_RELEASING   // 新增释放状态
};
SystemState currentState = IDLE;

// 舵机控制参数
const int GRASP_ANGLE = 45;
const int GRASP_STEP_DELAY = 30;  // 每步30ms
unsigned long lastStepTime = 0;
int currentServoAngle = 0;
float target_theta1 = 0, target_theta2 = 0, target_height = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // 初始化安全按钮

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
        currentServoAngle = 0;
        currentState = SERVO_GRASPING;  // 进入抓取阶段
      }
      break;
      
    case SERVO_GRASPING:
      handleGrasping();
      break;
      
    case SERVO_HOLDING:
      if(millis() - lastStepTime > 1000){ // 保持1秒
        returnToOrigin();
        currentState = RETURNING;
      }
      break;
      
    case RETURNING:
      updateMotors();
      if(allMotorsStopped()){
        currentState = SERVO_RELEASING;  // 进入释放阶段
      }
      break;
      
    case SERVO_RELEASING:
      handleReleasing();
      break;
  }
}

// 新增抓取控制函数
void handleGrasping() {
  if(digitalRead(BUTTON_PIN) == LOW){  // 安全检测
    emergencyStop();
    return;
  }
  
  if(millis() - lastStepTime > GRASP_STEP_DELAY){
    if(currentServoAngle < GRASP_ANGLE){
      currentServoAngle++;
      base_servo.write(currentServoAngle);
      lastStepTime = millis();
    } else {
      currentState = SERVO_HOLDING;
      lastStepTime = millis();
    }
  }
}

// 新增释放控制函数
void handleReleasing() {
  if(millis() - lastStepTime > GRASP_STEP_DELAY){
    if(currentServoAngle > 0){
      currentServoAngle--;
      base_servo.write(currentServoAngle);
      lastStepTime = millis();
    } else {
      currentState = IDLE;
      Serial.println("CYCLE_COMPLETE");
    }
  }
}

// 新增急停函数
void emergencyStop() {
  base_servo.detach();
  Serial.println("EMERGENCY_STOP");
  currentState = IDLE;
}

// 保持原有函数不变
void processCommand(String cmd) {
  int comma1 = cmd.indexOf(',');
  int comma2 = cmd.indexOf(',', comma1 + 1);

  if(comma1 != -1 && comma2 != -1){
    target_theta1 = cmd.substring(0, comma1).toFloat();
    target_height = cmd.substring(comma1+1, comma2).toFloat();
    target_theta2 = cmd.substring(comma2+1).toFloat();

    moveTo(target_theta1, target_height, target_theta2);
  }
}

void moveTo(float theta1, float height, float theta2) {
  stepper_theta1.move(theta1 * STEPS_PER_DEGREE * 0.66);
  stepper_height.move(height * STEPS_PER_CM * 1.5);
  stepper_theta2.move(theta2 * STEPS_PER_DEGREE * 0.2);
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