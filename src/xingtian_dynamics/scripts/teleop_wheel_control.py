#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select

class KeyboardTeleop:
    def __init__(self):
        rospy.init_node('keyboard_teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
        # 终端设置
        self.fd = sys.stdin.fileno()     #获取标准输入的文件描述符
        self.old_settings = termios.tcgetattr(self.fd)   #保存终端设置
        tty.setcbreak(sys.stdin.fileno())     #确保用户按下按键立即返回
        
    def reset_speeds(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        try:
            while not rospy.is_shutdown():
                # 检测按键输入
                if select.select([sys.stdin], [], [], 0)[0]:     #用来检查是否有按键输入，如果有，则继续读取
                    key = sys.stdin.read(1)       #每次读取一个字符
                    
                    if key == 'w':
                        self.linear_x += 0.1
                    elif key == 's':
                        self.linear_x -= 0.1
                    elif key == 'a':
                        self.linear_y += 0.1
                    elif key == 'd':
                        self.linear_y -= 0.1
                    elif key == 'z':
                        self.reset_speeds()
                    elif key == 'i':
                        self.angular_z += 0.1
                    elif key == 'k':
                        self.angular_z -= 0.1
                    # elif key.isdigit():      #如果按下的是数字，就转换为角速度
                    #     self.angular_z = float(key)
                    
                    # 四舍五入保留一位小数
                    self.linear_x = round(self.linear_x, 2)
                    self.linear_y = round(self.linear_y, 2)
                
                # 构造并发布Twist消息
                twist = Twist()
                twist.linear.x = self.linear_x
                twist.linear.y = self.linear_y
                twist.angular.z = self.angular_z
                print("linear_x: {}, linear_y: {}, angular_z: {}".format(self.linear_x, self.linear_y, self.angular_z))
                self.pub.publish(twist)
                rate.sleep()
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)   #恢复终端的原始设置

if __name__ == '__main__':
    try:
        kt = KeyboardTeleop()
        kt.run()
    except rospy.ROSInterruptException:
        pass