#! /usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import numpy as np
import time
from sensor_msgs.msg import PointCloud
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker


class HandController():
    def __init__(self):
        # ハンドと爪先の間の寸法、単位はメートル
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.arm.set_max_velocity_scaling_factor(0.7)
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        self.finger_pub = rospy.Publisher("finger_pub", Marker, queue_size=5)
        self.finger_size_vector = np.array([[0.0],
                                            [0.0], 
                                            [0.06 + 0.024]])
        self.finger_pose_vector = np.array([[0.0],
                                            [0.0],
                                            [0.0]])

    def transform_3Darray_to_3Dvector(self, pos):
        pos_vec = np.array([[pos[0]], 
                            [pos[1]], 
                            [pos[2]]])
        return pos_vec

    def rotate_euler_angle_x_axis(self, position_vec, theta):
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(theta), -np.sin(theta)],
                                    [0, np.sin(theta), np.cos(theta)]])
        return np.dot(rotation_matrix, position_vec)

    def rotate_euler_angle_y_axis(self, position_vec, theta):
        rotation_matrix = np.array([[np.cos(theta), 0, np.sin(theta)],
                                    [0, 1, 0],
                                    [-np.sin(theta), 0, np.cos(theta)]])
        return np.dot(rotation_matrix, position_vec)

    def rotate_euler_angle_z_axis(self, position_vec, theta):
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                    [np.sin(theta), np.cos(theta), 0],
                                    [0, 0, 1]])
        return np.dot(rotation_matrix, position_vec)

    def get_current_hand_pose_as_vector(self):
        pos = self.arm.get_current_pose().pose.position
        return self.transform_3Darray_to_3Dvector([pos.x, pos.y, pos.z])

    def get_current_hand_orientation_euler(self):
        q = self.arm.get_current_pose().pose.orientation
        return euler_from_quaternion((q.x, q.y, q.z, q.w))

    def calculate_finger_pose_from_hand_pose(self):
        hand_current_pos_vec = self.get_current_hand_pose_as_vector()
        hand_ori = self.get_current_hand_orientation_euler()
        # 指先をハンドと同じ姿勢に回転移動させる
        # XYZ軸を順番的に回転する
        finger_pose = self.rotate_euler_angle_x_axis(self.finger_size_vector, hand_ori[0])
        finger_pose = self.rotate_euler_angle_y_axis(finger_pose, hand_ori[1])
        finger_pose = self.rotate_euler_angle_z_axis(finger_pose, hand_ori[2])

        # ハンドと同じ姿勢になったハンドを平行移動させる
        finger_pose += hand_current_pos_vec

        return finger_pose

    def update_finger_pose(self):
        self.finger_pose_vector = self.calculate_finger_pose_from_hand_pose() 

    def publish_finger_marker(self):
        finger_marker = Marker()
        finger_marker.header.frame_id = "base_link"
        finger_marker.id = 1000
        finger_marker.ns = "finger_marker"
        finger_marker.type = Marker.CUBE
        finger_marker.action = Marker.ADD
        finger_marker.header.stamp = rospy.Time.now() 

        finger_marker.pose.position.x = self.finger_pose_vector[0][0]
        finger_marker.pose.position.y = self.finger_pose_vector[1][0]
        finger_marker.pose.position.z = self.finger_pose_vector[2][0]

        q = self.arm.get_current_pose().pose.orientation
        finger_marker.pose.orientation.x = q.x 
        finger_marker.pose.orientation.y = q.y 
        finger_marker.pose.orientation.z = q.z 
        finger_marker.pose.orientation.w = q.w 

        finger_marker.color.r = 0.8
        finger_marker.color.g = 0.0
        finger_marker.color.b = 0.8
        finger_marker.color.a = 1.0
        finger_marker.scale.x = 0.03
        finger_marker.scale.y = 0.05
        finger_marker.scale.z = 0.001

        finger_marker.lifetime = rospy.Duration(0.1)
        self.finger_pub.publish(finger_marker)

    def set_gripper_pose(self, value):
        gripper.set_joint_value_target(value)
        gripper.go()
        rospy.sleep(0.5)

    def set_arm_pose(self, pos, ori):
        # 手動で姿勢を指定するには以下のように指定
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = pos[0]
        target_pose.position.y = pos[1]
        target_pose.position.z = pos[2]

        #q = quaternion_from_euler(  )
        self.hand_ori = ori
        q = quaternion_from_euler( ori[0], ori[1], ori[2] )
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]

        self.arm.set_pose_target( target_pose )
        self.arm.go()
        rospy.sleep(1.0)
    

def main():
    rospy.init_node("pose_groupstate_example")

    hand_controller = HandController()

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)
    
    print("Group names:")
    print(hand_controller.robot.get_group_names())
    
    target_pos = [0.25, -0.03, 0.336]
    target_ori = [3.14/1, 0.0, 3.14/1]
    hand_controller.set_arm_pose(target_pos, target_ori)

    # 移動後の手先ポーズを表示
    arm_goal_pose = hand_controller.arm.get_current_pose().pose
    print("Arm goal pose:")
    print(arm_goal_pose)
    print("done")

    while not rospy.is_shutdown():
        hand_controller.update_finger_pose()
        hand_controller.publish_finger_marker()
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()
