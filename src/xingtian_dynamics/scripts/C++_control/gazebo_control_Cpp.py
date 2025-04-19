#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64,Float64MultiArray
import time

# 控制话题名称列表
control_topics = [
    "LF_hip_controller/command", "LF_knee_controller/command", "LF_wheel_controller/command",
    "LR_hip_controller/command", "LR_knee_controller/command", "LR_wheel_controller/command",
    "RR_hip_controller/command", "RR_knee_controller/command", "RR_wheel_controller/command",
    "RF_hip_controller/command", "RF_knee_controller/command", "RF_wheel_controller/command"
]

# 全局变量：最后接收消息的时间戳
last_received_time = [time.time()] * len(control_topics)
global _pub_idx
# 订阅回调函数：更新接收时间戳
def callback(msg, index):
    global last_received_time
    last_received_time[index] = time.time()
    # 如果接收到消息，则这里不做处理（消息正常通过）
    global _if_received
    _if_received = True
    global _pub_idx
    _pub_idx = False
    # rospy.loginfo(f"Topic {control_topics[index]} received message: {msg.data}")

def main():
    rospy.init_node("default_controller", anonymous=True)
    rate = rospy.Rate(10)  # 设置10Hz的循环频率
    global _if_received
    _if_received = False
    _pub_idx = True
    # 发布者和订阅者
    publishers = []
    for topic in control_topics:
        publishers.append(rospy.Publisher('/xingtian/'+topic, Float64, queue_size=10))

    subscribers = []
    for i, topic in enumerate(control_topics):
        subscribers.append(rospy.Subscriber('/xingtian/'+topic, Float64, callback, callback_args=i))

    # 默认的控制指令
    default_msg = Float64MultiArray()
    default_msg.data = [0.0] * len(control_topics)
    default_msg.data[0] = 5.0  # 默认值为0，可以根据需要修改
    default_msg.data[1] = -5.0
    default_msg.data[2] = 0.0
    default_msg.data[3] = 5.0
    default_msg.data[4] = -5.0
    default_msg.data[5] = 0.0
    default_msg.data[6] = 5.0
    default_msg.data[7] = -5.0
    default_msg.data[8] = 0.0
    default_msg.data[9] = 5.0
    default_msg.data[10] = -5.0
    default_msg.data[11] = 0.0
    # 超时时间设定
    timeout = 10.0  # 30秒超时

    rospy.loginfo("Default controller node started. Monitoring control topics...")

    while not rospy.is_shutdown():
        current_time = time.time()
        # 检查每个话题的最后接收时间
        for i, pub in enumerate(publishers):
            if not _if_received and current_time - last_received_time[i] > timeout:
                _pub_idx = True
                # 超时，发布默认控制指令
            if _pub_idx :
                for j in range(len(publishers)):
                    rospy.logwarn(f"Topic {control_topics[j]} timed out. Publishing default message.")
                    pub.publish(default_msg.data[j])
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
