import numpy as np
import math
import matplotlib.pyplot as plt
import serial

# 串口配置
ser = serial.Serial('COM6', 115200, timeout=1)

# 机械臂参数
l1 = 16.1  # 大臂长度 (cm)
l2 = 11.7  # 小臂长度 (cm)
prev_state = {
	'theta1': 0.0,  # 大臂角度 (度)
	'theta2': 0.0,  # 小臂角度 (度)
	'height': 0.0  # 基座高度 (cm)
}


# 运动学函数
def angle_diff(current, previous):
	"""角度差值计算（考虑360°环绕）"""
	diff = (current - previous + 180) % 360 - 180
	return diff if abs(diff) > 0.001 else 0.0


def inverse_kinematics(x, y):
	"""逆运动学解算"""
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
		math.degrees(theta1) % 360,
		math.degrees(theta2) % 360
	)


#  可视化工作空间
def plot_workspace():
	theta1_range = np.radians(np.linspace(-160, 160, 360))
	theta2_range = np.radians(np.linspace(-60, 60, 360))
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


# ================ 控制指令发送 =================
def send_control_command(theta1, height, theta2):
	"""发送三轴控制指令"""
	command = f"{theta1:.2f},{height:.2f},{theta2:.2f}\n"
	ser.write(command.encode())
	print(f"[指令已发送] θ1={theta1:.1f}° | 高度={height:.1f}cm | θ2={theta2:.1f}°")


# ================ 主交互系统 ===================
def main_control_system():
	print("1. 输入目标坐标(X,Y)和基座高度")
	print("2. 系统自动计算关节角度")
	print("3. 发送控制指令至执行机构\n")

	plot_workspace()  # 显示工作空间

	try:
		while True:
			try:
				# 用户输入
				x = float(input("\n目标X坐标 (cm) > "))
				y = float(input("目标Y坐标 (cm) > "))
				new_h = float(input("基座高度 (0-15cm) > "))

				# 输入验证
				if not (0 <= new_h <= 15):
					raise ValueError("高度超出安全范围(0-15cm)")

				# 逆运动学
				theta1, theta2 = inverse_kinematics(x, y)

				# 计算相对变化量
				delta_theta1 = angle_diff(theta1, prev_state['theta1'])
				delta_theta2 = angle_diff(theta2, prev_state['theta2'])
				delta_h = new_h - prev_state['height']

				# 显示状态
				print("\n当前运动参数:")
				print(f"θ1: {theta1:7.2f}° [变化量: {delta_theta1:+7.2f}°]")
				print(f"θ2: {theta2:7.2f}° [变化量: {delta_theta2:+7.2f}°]")
				print(f"基座高度: {new_h:5.2f}cm [变化量: {delta_h:+5.2f}cm]")

				# 发送指令
				send_control_command(delta_theta1, new_h, delta_theta2)

				# 更新状态
				prev_state.update({
					'theta1': theta1,
					'theta2': theta2,
					'height': new_h
				})

			except ValueError as e:
				print(f"\n错误: {e}")
			except Exception as e:
				print(f"\n系统异常: {str(e)}")

	except KeyboardInterrupt:
		print("最终状态:")
		print(f"θ1: {prev_state['theta1']:.2f}°")
		print(f"θ2: {prev_state['theta2']:.2f}°")
		print(f"基座高度: {prev_state['height']:.2f}cm")
	finally:
		ser.close()


if __name__ == "__main__":
	main_control_system()