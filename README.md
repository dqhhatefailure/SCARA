dqh:

这是一个SCARA三自由度机械臂项目，由我和@1020204共同完成，我负责机械臂的所有代码，他负责机械结构建模，材料采买和3D打印，最后共同组装调试完成。

本项目组成包含机械结构、代码编写、材料清单。

代码部分分为arduino端和python端，arduino端负责驱动电机和舵机，python端进行逆运动学计算并向arduino端传送信号

本项目机械臂工作流程介绍

1.上位机输入坐标点1和2及其高度1和2

2.arduino端接收坐标，换算为脉冲发射信号输出到电机（人工Kp调节）

3.两个力臂上的MPU6050测算目标角度和实际角度之差，回调电机（2025.3.26尚未完成）

4.机械臂抵达目标点1后，夹爪执行抓取

5.抓取后运动至目标点2，松开夹爪（2025.3.26尚未完成）

6.回到起始点（2025.3.26尚未完成）

## 当机械臂意外超出设定工作空间时自动强行制动（2025.3.26尚未完成）

3.27日志

上传机械臂1.0版本（robot1.0.ino）将上述目标5完成，其它未完善（MPU6050运行十几秒后总是自动卡死？？？）闭环控制希望渺茫

今日提交时时突然想到机械臂运动时夹爪中心点未触碰至目标点时夹爪侧边可能碰倒目标，今后需针对此修改代码（先移动至目标点前一段距离后再往前伸展机械臂）

mpu6050或可以采集机械臂平面运行角度变化数据，帮助完成更精确的开环控制。
