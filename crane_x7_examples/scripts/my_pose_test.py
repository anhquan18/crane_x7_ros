#! /usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
import numpy as np
import time
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


def rotate_3Dmatrix_x_axis(position_vec, theta):
    rotation_matrix = np.array([[1, 0, 0],
                                [0, np.cos(theta), -np.sin(theta)],
                                [0, np.sin(theta), np.cos(theta)]])
    return np.dot(rotation_matrix, position_vec)

def rotate_3Dmatrix_y_axis(position_vec, theta):
    rotation_matrix = np.array([[np.cos(theta), 0, np.sin(theta)],
                                [0, 1, 0],
                                [-np.sin(theta), 0, np.cos(theta)]])
    return np.dot(rotation_matrix, position_vec)

def rotate_3Dmatrix_z_axis(position_vec, theta):
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                [np.sin(theta), np.cos(theta), 0],
                                [0, 0, 1]])
    return np.dot(rotation_matrix, position_vec)

def main():
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.7)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    finger_pub = rospy.Publisher("finger_pub", Marker, queue_size=5)

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)
    
    print("Group names:")
    print(robot.get_group_names())
    
    print("Current state:")
    print(robot.get_current_state().joint_state)
    
    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()
    

    '''
    print("testb")
    arm.set_named_target("testd")
    arm.go()
    '''

    # ハンドを少し閉じる
    gripper.set_joint_value_target([0.01, 0.01])
    gripper.go()

    # 手動で姿勢を指定するには以下のように指定
   
    
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.25
    target_pose.position.y = -0.02
    target_pose.position.z = 0.336
    q = quaternion_from_euler( 3.14/1, 0.0, 3.14/1 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )	# 目標ポーズ設定
    arm.go()							# 実行

    pos_vec = np.array([[target_pose.position.x], 
                        [target_pose.position.y], 
                        [target_pose.position.z]])

    pos_vec = rotate_3Dmatrix_x_axis(pos_vec, 3.14)
    pos_vec = rotate_3Dmatrix_y_axis(pos_vec, 0.0)
    pos_vec = rotate_3Dmatrix_z_axis(pos_vec, 3.14)

    while not rospy.is_shutdown():
        finger_marker = Marker()
        finger_marker.header.frame_id = "base_link"
        finger_marker.header.stamp = rospy.Time.now() 

        finger_marker.ns = "finger_marker"
        finger_marker.id = 1000
        finger_marker.type = Marker.CUBE
        finger_marker.action = Marker.ADD

        print("x:", pos_vec[0][0])
        print("y:", pos_vec[1][0])
        print("z:", pos_vec[2][0])
        finger_marker.pose.position.x = target_pose.position.x
        finger_marker.pose.position.y = target_pose.position.y
        finger_marker.pose.position.z = target_pose.position.z - 0.06 - 0.024

        finger_marker.pose.orientation.x = q[0]
        finger_marker.pose.orientation.y = q[1]
        finger_marker.pose.orientation.z = q[2]
        finger_marker.pose.orientation.w = q[3]

        finger_marker.color.r = 0.8
        finger_marker.color.g = 0.0
        finger_marker.color.b = 0.8
        finger_marker.color.a = 1.0
        finger_marker.scale.x = 0.02
        finger_marker.scale.y = 0.06
        finger_marker.scale.z = 0.001

        finger_marker.lifetime = rospy.Duration(0.5)
        finger_pub.publish(finger_marker)

        time.sleep(1.0)


    # 移動後の手先ポーズを表示
    arm_goal_pose = arm.get_current_pose().pose
    print("Arm goal pose:")
    print(arm_goal_pose)
    print("done")


if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
