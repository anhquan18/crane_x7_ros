#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool
import sys
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import numpy as np


robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")
arm.set_max_velocity_scaling_factor(0.3)
gripper = moveit_commander.MoveGroupCommander("gripper")


def getMerginPosition(origin_position, origin_quo, mergin):
    euler = euler_from_quaternion(origin_quo)
    mergin_np = np.array(mergin)

    # x軸まわりの回転
    x_roll = euler[0] 
    rx = np.matrix(np.array([[1, 0, 0],
        [0, math.cos(x_roll), math.sin(x_roll)],
        [0, -math.sin(x_roll), math.cos(x_roll)]]))
    p_x = np.dot(mergin_np, rx)

    # Y軸まわりの回転
    y_roll = euler[1]
    ry = np.matrix(np.array([[math.cos(y_roll), 0, -math.sin(y_roll)],
        [0, 1, 0],
        [math.sin(y_roll), 0, math.cos(y_roll)]]))
    p_xy = np.dot(p_x, ry)

    # z軸回りの回転
    z_roll = euler[2]
    rx = np.matrix(np.array([[math.cos(z_roll), math.sin(z_roll), 0],
        [-math.sin(z_roll), math.cos(z_roll), 0],
        [0, 0, 1]]))
    p_xyz = np.dot(p_xy, rx)

    result_position = Point()
    result_position.x = origin_position.x + p_xyz[0, 0]
    result_position.y = origin_position.y + p_xyz[0, 1]
    result_position.z = origin_position.z + p_xyz[0, 2]

    return result_position


def tackle_callback(msg):
    global robot
    global arm
    global gripper
  
    # 位置姿勢の代入
    x = msg.position.x
    y = msg.position.y
    z = msg.position.z
    q = quaternion_from_euler( 3.14, 0.0, 3.14 )

    pub_bool = Bool()
    pub_bool.data = False

    # グリッパーを閉じる
    gripper.set_joint_value_target([0.05, 0.05])
    gripper.go()

    # 近づく 
    mergin_position = getMerginPosition(msg.position, q, [0, 0, -0.1])
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = mergin_position.x 
    target_pose.position.y = mergin_position.y
    target_pose.position.z = mergin_position.z
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)
    arm.go()

    # 把持位置に移動
    target_pose = geometry_msgs.msg.Pose()
    mergin_position = getMerginPosition(msg.position, q, [0, 0, -0.055])
    target_pose.position.x = mergin_position.x 
    target_pose.position.y = mergin_position.y
    target_pose.position.z = mergin_position.z
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)
    arm.go()

    # グリッパーを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # グリッパーを閉じる
    gripper.set_joint_value_target([0.05, 0.05])
    gripper.go()

    # もとに戻る
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.26
    target_pose.position.y = 0.0
    target_pose.position.z = 0.336
    q = quaternion_from_euler( 3.14, 0.0, 3.14 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )	# 目標ポーズ設定
    arm.go()

    rospy.sleep(3)
    #print(" done")
    #print "==============================================="


def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    tackle_sub = rospy.Subscriber('/tackle_pose', Pose, tackle_callback)
    sub2 = rospy.Subscriber('/camera_pose', Pose, callback2)
    pub_only()
    rospy.spin()    
    
if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
