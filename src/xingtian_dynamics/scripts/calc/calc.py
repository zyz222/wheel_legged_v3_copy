import math
import numpy as np
 # 杆长 l1 l2
l1 = 0.140
l2 = 0.140
x1, y1 = -0.269,-0.084
# 判断是否有解
a1 = (x1*x1 + y1*y1 - l1*l1 - l2*l2) / (2*l1*l2)

b = math.sqrt(x1*x1 + y1*y1)
print(a1)
print(b)
# 判断是否有解
if np.abs(a1) > 1:
    print("无法到达该位置")
    exit()

# 求解 theta3
theta13 = np.zeros((2, 1))
theta13[0, 0] = np.arccos(a1)
theta13[1, 0] = -np.arccos(a1)   #取这个解


# 求解 theta2

# 左前腿
k11 = l1 + l2 * np.cos(theta13[0, 0])
k12 = l2 * np.sin(theta13[0, 0])
gama1 = np.arctan2(k12, k11)
theta12_1 = np.arctan2(y1, x1) - gama1

k11 = l1 + l2 * np.cos(theta13[1, 0])
k12 = l2 * np.sin(theta13[1, 0])
gama1 = np.arctan2(k12, k11)
theta12_2 = np.arctan2(y1, x1) - gama1

# 输出角度 theta2大腿

ftheta = np.zeros((2, 4))
ftheta[0, 0] = np.degrees(theta12_1) 
ftheta[1, 0] = np.degrees(theta12_2) 
ftheta[0, 1] = np.degrees(theta13[0, 0])
ftheta[1, 1] = np.degrees(theta13[1, 0])
print(ftheta)

