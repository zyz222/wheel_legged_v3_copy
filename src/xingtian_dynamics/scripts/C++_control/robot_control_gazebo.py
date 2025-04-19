#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Twist

class ModelSetter:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('model_setter', anonymous=True)

        # 创建服务代理
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # 等待服务可用
        rospy.wait_for_service('/gazebo/set_model_state')

    def set_model_pose(self, model_name, position, orientation):
        # 创建 ModelState 请求
        model_state = SetModelStateRequest()
        model_state.model_state.model_name = model_name

        # 设置位置和姿态
        model_state.model_state.pose.position.x = position[0]
        model_state.model_state.pose.position.y = position[1]
        model_state.model_state.pose.position.z = position[2]

        model_state.model_state.pose.orientation.x = orientation[0]
        model_state.model_state.pose.orientation.y = orientation[1]
        model_state.model_state.pose.orientation.z = orientation[2]
        model_state.model_state.pose.orientation.w = orientation[3]

        # 设置线速度和角速度为零（可选）
        model_state.model_state.twist = Twist()

        try:
            # 调用服务
            response = self.set_model_state(model_state)
            if response.success:
                rospy.loginfo('Model state set successfully.')
            else:
                rospy.logerr('Failed to set model state: {}'.format(response.status_message))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    def start_timer(self, model_name, position, orientation):
        self.model_name = model_name
        self.position = position
        self.orientation = orientation

        # 启动一个定时器，每 0.1 秒重置一次模型
        self.timer = rospy.Timer(rospy.Duration(10), self.timer_callback)

    def timer_callback(self, event):
        self.set_model_pose(self.model_name, self.position, self.orientation)


def main():
    model_setter = ModelSetter()

    # 设置初始位置和姿态
    position = [0.0, 0.0, 1.0]  # x, y, z
    orientation = [0.0, 0.0, 0.0, 1.0]  # 四元数 (x, y, z, w)
    model_name = 'xingtian'

    # 使用定时器持续保持模型悬挂
    model_setter.start_timer(model_name, position, orientation)
    rospy.spin()



if __name__ == '__main__':
    main()


# #!/usr/bin/env python

# import rospy
# from gazebo_msgs.srv import ApplyBodyWrench
# from geometry_msgs.msg import Wrench

# class PartHanger:
#     def __init__(self):
#         rospy.init_node('part_hanger', anonymous=True)
        
#         # 等待 Gazebo 的 /gazebo/apply_body_wrench 服务可用
#         rospy.wait_for_service('/gazebo/apply_body_wrench')
#         self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

#     def hang_part(self, model_name, part_name, force_z, duration):
#         """
#         给指定零部件施加一个向上的力，使其悬挂
#         :param model_name: 模型名称（机器人名称）
#         :param part_name: 零部件名称（机器人中的具体 link 名称）
#         :param force_z: 向上的力大小（与重力平衡）
#         :param duration: 力的作用时间
#         """
#         # 创建一个向上的 Wrench（力矩）
#         wrench = Wrench()
#         wrench.force.z = force_z  # 设置向上的力，抵消重力
        
#         try:
#             # 调用服务，施加力
#             response = self.apply_wrench(
#                 body_name=f"{model_name}::{part_name}",  # 模型名称 + 零部件名称
#                 reference_frame="world",
#                 wrench=wrench,
#                 start_time=rospy.Time.now(),
#                 duration=rospy.Duration(duration)  # 持续时间
#             )
#             if response.success:
#                 rospy.loginfo(f"Successfully applied upward force to {model_name}::{part_name}.")
#             else:
#                 rospy.logerr(f"Failed to apply force: {response.status_message}")
#         except rospy.ServiceException as e:
#             rospy.logerr(f"Service call failed: {e}")

# def main():
#     # 创建 PartHanger 实例
#     hanger = PartHanger()
    
#     # 模型名称和零部件名称
#     model_name = "xingtian"  # 替换为你的机器人模型名
#     part_name = "dummy"  # 替换为悬挂的零部件名（如 base_link 或 arm_link 等）
    
#     # 力大小和作用时间
#     force_z = 200  # 与零部件重力平衡的向上力
#     duration = -1  # 持续时间（-1 表示无限施加力）
    
#     # 施加悬挂力
#     hanger.hang_part(model_name, part_name, force_z, duration)

#     rospy.spin()

# if __name__ == "__main__":
#     main()
