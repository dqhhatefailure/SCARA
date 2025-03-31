import numpy as np
import math
import matplotlib.pyplot as plt
import serial

ser = serial.Serial('COM6', 115200, timeout=1)

l1 = 15.5  # 大臂长度 (cm)
l2 = 24    # 小臂长度 (cm)
prev_state = {
    'theta1': 0.0,
    'theta2': 0.0,
    'height': 0.0
}

def adjust_angle(radians):
    """将弧度转换为角度并规范到[-180, 180]范围"""
    degrees = math.degrees(radians)
    normalized = degrees % 360
    if normalized > 180:
        return normalized - 360
    return normalized

def inverse_kinematics(x, y):
    """逆运动学解算（返回角度范围：-180°到180°）"""
    d_sq = x ** 2 + y ** 2
    max_reach = (l1 + l2) ** 2
    min_reach = (l1 - l2) ** 2

    if not (min_reach <= d_sq <= max_reach):
        raise ValueError(f"坐标({x:.1f},{y:.1f})不可达")

    # 计算关节角度
    cos_theta2 = (d_sq - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
    theta2 = math.acos(cos_theta2)

    angle = math.atan2(y, x)
    gamma = math.acos((d_sq + l1 ** 2 - l2 ** 2) / (2 * l1 * math.sqrt(d_sq)))
    theta1 = angle - gamma

    return (
        adjust_angle(theta1),
        adjust_angle(theta2)
    )

def plot_workspace():
    theta1_range = np.radians(np.linspace(-160, 160, 360))
    theta2_range = np.radians(np.linspace(-90, 90, 360))
    THETA1, THETA2 = np.meshgrid(theta1_range, theta2_range)

    X = l1 * np.cos(THETA1) + l2 * np.cos(THETA1 + THETA2)
    Y = l1 * np.sin(THETA1) + l2 * np.sin(THETA1 + THETA2)

    plt.figure(figsize=(8, 8))
    plt.scatter(X.ravel(), Y.ravel(), s=1, c='blue', alpha=0.1)
    plt.title('Robot Arm Workspace')
    plt.xlabel('X Position (cm)')
    plt.ylabel('Y Position (cm)')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def send_control_command(theta1, height, theta2):
    """发送绝对位置指令"""
    command = f"{theta1:.2f},{height:.2f},{theta2:.2f}\n"
    ser.write(command.encode())
    print(f"[指令已发送] θ1={theta1:.1f}° | 高度={height:.1f}cm | θ2={theta2:.1f}°")

def main_control_system():
    print("1. 输入目标坐标(X,Y)和基座高度")
    print("2. 系统直接发送绝对位置指令\n")
    plot_workspace()

    try:
        while True:
            try:
                # 用户输入
                x = float(input("\n目标X坐标 (cm) > "))
                y = float(input("目标Y坐标 (cm) > "))
                new_h = float(input("基座高度 (0-15cm) > "))

                if not (0 <= new_h <= 15):
                    raise ValueError("高度超出安全范围(0-15cm)")

                # 逆运动学解算
                theta1, theta2 = inverse_kinematics(x, y)

                # 显示绝对位置
                print("\n目标位置：")
                print(f"θ1: {theta1:7.2f}°")
                print(f"θ2: {theta2:7.2f}°")
                print(f"基座高度: {new_h:5.2f}cm")

                # 发送控制指令
                send_control_command(theta1, new_h, theta2)

                # 更新状态（仅记录，不用于计算差值）
                prev_state.update({
                    'theta1': theta1,
                    'theta2': theta2,
                    'height': new_h
                })

            except ValueError as e:
                print(f"\n输入错误: {e}")
            except Exception as e:
                print(f"\n系统异常: {str(e)}")

    except KeyboardInterrupt:
        print("\n=== 系统安全关闭 ===")
        print("最终状态：")
        print(f"θ1: {prev_state['theta1']:.2f}°")
        print(f"θ2: {prev_state['theta2']:.2f}°")
        print(f"基座高度: {prev_state['height']:.2f}cm")
    finally:
        ser.close()

if __name__ == "__main__":
    main_control_system()