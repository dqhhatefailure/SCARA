#include <AccelStepper.h>
#include <Servo.h>

// ================= 硬件引脚定义 =================
#define X_STEP     2
#define X_DIR      5
#define Y_STEP     3
#define Y_DIR      6
#define Z_STEP     4
#define Z_DIR      7
#define ENABLE_PIN 8
#define SERVO_PIN  11

// ================= 机械参数校准 =================
const float STEPS_PER_DEGREE = 8.89;
const float STEPS_PER_CM = 500.0;

// ================= 全局对象初始化 =================
AccelStepper stepper_theta1(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepper_theta2(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper stepper_height(AccelStepper::DRIVER, Z_STEP, Z_DIR);
Servo base_servo;

// ================= 运动控制变量 =================
float current_theta1 = 0.0;
float current_theta2 = 0.0;
float current_height = 0.0;
bool motorsRunning = false;
bool servoTriggered = false;

// ================= 初始化设置 =================
void setup() {
  Serial.begin(115200);
  
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  stepper_theta1.setMaxSpeed(250);
  stepper_theta1.setAcceleration(50);
  stepper_theta2.setMaxSpeed(250);
  stepper_theta2.setAcceleration(50);
  stepper_height.setMaxSpeed(500);
  stepper_height.setAcceleration(100);

  base_servo.attach(SERVO_PIN);
  base_servo.write(0);  // 初始化为0度
}

// ================= 主循环 =================
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
    servoTriggered = false;  // 新指令重置舵机触发状态
    motorsRunning = true;    // 标记电机开始运行
  }

  updateSteppers();
  
  // 检测电机状态并触发舵机
  if (motorsRunning && allMotorsStopped() && !servoTriggered) {
    controlServo();
    motorsRunning = false;
    servoTriggered = true;
  }
}

// ================= 指令解析函数 =================
void parseCommand(String cmd) {
  int comma1 = cmd.indexOf(',');
  int comma2 = cmd.indexOf(',', comma1 + 1);

  if (comma1 != -1 && comma2 != -1) {
    float theta1 = cmd.substring(0, comma1).toFloat();
    float height = cmd.substring(comma1 + 1, comma2).toFloat();
    float theta2 = cmd.substring(comma2 + 1).toFloat();

    setTheta1(theta1);
    setHeight(height);
    setTheta2(theta2);
    
    Serial.print("ACK:");
    Serial.print(theta1); Serial.print(",");
    Serial.print(height); Serial.print(",");
    Serial.pri  ntln(theta2);
  }
}

// ================= 运动控制函数 =================
void setTheta1(float theta) {
  long steps = (theta - current_theta1) * STEPS_PER_DEGREE * 0.66;
  stepper_theta1.move(steps);
  current_theta1 = theta;
}

void setTheta2(float theta) {
  long steps = (theta - current_theta2) * STEPS_PER_DEGREE * 0.066;
  stepper_theta2.move(steps);
  current_theta2 = theta;
}

void setHeight(float height) {
  long steps = (height - current_height) * STEPS_PER_CM * 2;
  stepper_height.move(steps);
  current_height = height;
}

// ================= 状态检测函数 =================
bool allMotorsStopped() {
  return (stepper_theta1.distanceToGo() == 0) && 
         (stepper_theta2.distanceToGo() == 0) && 
         (stepper_height.distanceToGo() == 0);
}

// ================= 舵机控制函数 =================
void controlServo() {
  base_servo.write(45);      // 转动到45度
  Serial.println("SERVO:45");// 发送舵机状态
  delay(500);                // 等待舵机到位（根据实际舵机速度调整）
}

// ================= 电机更新函数 =================
void updateSteppers() {
  stepper_theta1.run();
  stepper_theta2.run();
  stepper_height.run();
}