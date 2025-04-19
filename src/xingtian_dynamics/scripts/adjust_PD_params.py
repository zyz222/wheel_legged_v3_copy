#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from std_msgs.msg import Float32MultiArray
from PyQt5.QtWidgets import QApplication, QWidget, QSlider, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt

class MotorControlWidget(QWidget):
    def __init__(self):
        super().__init__()

        # 初始化ROS发布者
        self.pub = rospy.Publisher('/xingtian/PD_params', Float32MultiArray, queue_size=10)

        # 初始化参数默认值
        self.kp = 100.0
        self.kd = 3.0
        self._kp = 55.0
        self._kd = 10.0

        # 创建界面组件
        self.kp_slider = QSlider(Qt.Horizontal)
        self.kd_slider = QSlider(Qt.Horizontal)
        self._kp_slider = QSlider(Qt.Horizontal)
        self._kd_slider = QSlider(Qt.Horizontal)


        # 配置滑块范围
        self.kp_slider.setRange(0, 200)
        self.kd_slider.setRange(0, 100)
        self._kp_slider.setRange(0, 200)
        self._kd_slider.setRange(0, 100)

        # 设置初始值
        self.kp_slider.setValue(int(self.kp))
        self.kd_slider.setValue(int(self.kd))
        self._kp_slider.setValue(int(self._kp))
        self._kd_slider.setValue(int(self._kd))

        # 创建数值显示标签
        self.kp_label = QLabel(f"Kp: {self.kp:.1f}")
        self.kd_label = QLabel(f"Kd: {self.kd:.1f}")
        self._kp_label = QLabel(f"Kp: {self._kp:.1f}")
        self._kd_label = QLabel(f"Kd: {self._kd:.1f}")

        # 连接信号与槽
        self.kp_slider.valueChanged.connect(self.update_kp)
        self.kd_slider.valueChanged.connect(self.update_kd)
        self._kp_slider.valueChanged.connect(self.update_kp_)
        self._kd_slider.valueChanged.connect(self.update_kd_)

        # 设置界面布局
        layout = QVBoxLayout()
        layout.addWidget(self.kp_label)
        layout.addWidget(self.kp_slider)
        layout.addWidget(self.kd_label)
        layout.addWidget(self.kd_slider)
        layout.addWidget(self._kp_label)
        layout.addWidget(self._kp_slider)
        layout.addWidget(self._kd_label)
        layout.addWidget(self._kd_slider)

        self.setLayout(layout)

    def update_kp(self, value):
        self.kp = float(value)
        self.kp_label.setText(f"Kp: {self.kp:.1f}")
        self.publish_params()

    def update_kd(self, value):
        self.kd = float(value)
        self.kd_label.setText(f"Kd: {self.kd:.1f}")
        self.publish_params()
    def update_kp_(self, value):
        self._kp = float(value)
        self._kp_label.setText(f"_Kp: {self._kp:.1f}")
        self.publish_params()

    def update_kd_(self, value):
        self._kd = float(value)
        self._kd_label.setText(f"_Kd: {self._kd:.1f}")
        self.publish_params()
    def publish_params(self):
        msg = Float32MultiArray()
        msg.data = [self.kp, self.kd,self._kp, self._kd]
        self.pub.publish(msg)
        # rospy.loginfo(f"Published parameters: Kp={self.kp}, Kd={self.kd}")

def main():
    # 初始化ROS节点
    rospy.init_node('motor_control_gui', anonymous=True)
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    window = MotorControlWidget()
    window.setWindowTitle('PD Parameters Controller')
    window.show()
    # rospy.spin()
    # 保证ROS节点在应用退出时正确关闭
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()