#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import std_msgs.msg
import rosnode
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool
import sys
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import time
from std_msgs.msg import Float32MultiArray
import numpy as np

pub = rospy.Publisher('/Pcl_StartFlag', Bool, queue_size=1)
pub2 = rospy.Publisher('/moved_flag', Bool, queue_size=1)
pub3 = rospy.Publisher('/reset_flag', Bool, queue_size=1)
count = 0

x_flag = 0
y_flag = 0
z_flag = 0
ex_flag = 0
ey_flag = 0
ez_flag = 0

sub_x = []
sub_y = []
sub_z = []
sub_ex = []
sub_ey = []
sub_ez = []

mode = 0 
flag_m = 0

robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")
arm.set_max_velocity_scaling_factor(0.7)
gripper = moveit_commander.MoveGroupCommander("gripper")

start = 0 #時間計測用変数
computing_flag = False


def pub_only():
    pub_bool = Bool()
    pub_bool.data = mode
    pub2.publish(pub_bool)   
    #print "pub_only   published ",
    #print mode 


def error_position_check( p ):
    gosa = abs( p[0] - p[1] )
    if ( gosa < 0.05):
        gosa = abs( p[1] - p[2] )
        if ( gosa < 0.05 ):
            r = 1
        else:
            r = 0
    else:
        r = 0
    return r


def error_orientation_check( e ):
    gosa = abs( math.fabs(e[0]) - math.fabs(e[1]) )
    if ( gosa < 0.5):
        gosa = abs( math.fabs(e[1]) - math.fabs(e[2]) )
        if ( gosa < 0.5 ):
            r = 1
        else:
            r = 0
    else:
        r = 0
    return r


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


def callback2(msg):
    global computing_flag
    if computing_flag == False:
        computing_flag = True

        global start
        start = time.time() #時間計測スタート
        #print "call back 2 start"
        #print start

    global robot
    global arm
    global gripper
    global count

    global flag_m
    global mode
    #print("callback2")
    #print("mode:{} ".format(mode))
    #print("flag_m:{}".format(flag_m))
    if ( mode == 0 and flag_m == 0 ):
        #print msg
            
        pub_bool = Bool()
        pub_bool.data = False
        pub2.publish(pub_bool)

        # pub_bool = Bool()
        # pub_bool.data = False
        # pub.publish(pub_bool)
        # print " Published bool Faulse"

        # グリッパーを開く
        gripper.set_joint_value_target([0.9, 0.9])
        gripper.go()

        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = msg.position.x
        target_pose.position.y = msg.position.y
        target_pose.position.z = msg.position.z

        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        #print "================ mode0 moving camera!!=================="
            


        # pub_bool = Bool()
        # pub_bool.data = True
        # pub.publish(pub_bool)

        pub_bool = Bool()
        pub_bool.data = False
        pub2.publish(pub_bool)
        #print " Published bool True"
        #print " done"
        #print "==============================================="

        rospy.sleep(3)
        flag_m = 1
        mode = 1

def callback(msg):
    global robot
    global arm
    global gripper

    global sub_x, sub_y, sub_z
    global sub_ex, sub_ey, sub_ez
    global count
    global mode
    global flag_m
    
    pub_bool = Bool()
    pub_bool.data = True
    pub2.publish(pub_bool)
  
    if (mode == 1 and flag_m == 1):
        #print("subscribed box pose") print(msg)
        #print ("second_count: {}".format( count ))
        #print " -----"

        #print(" Published bool Faulse")
        
        # 位置姿勢の代入
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        #e = tf.transformations.euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        pub_bool = Bool()
        pub_bool.data = False
        pub.publish(pub_bool)

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

        # グリッパーを開く
        gripper.set_joint_value_target([0.9, 0.9])
        gripper.go()

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

        # グリッパーを閉じる
        gripper.set_joint_value_target([0.05, 0.05])
        gripper.go()

        # 持ち上げる 
        target_pose = geometry_msgs.msg.Pose()
        mergin_position = getMerginPosition(msg.position, q, [0, 0, -0.15])
        target_pose.position.x = mergin_position.x 
        target_pose.position.y = mergin_position.y
        target_pose.position.z = mergin_position.z
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]                
        arm.set_pose_target(target_pose)
        arm.go()
        
        # 持ち上げたときのタイム
        global start
        end = time.time()
        elapsed_time = end - start  
        print ("{0}".format(elapsed_time))

        # もう一回置く
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

        # グリッパー開く
        gripper.set_joint_value_target([0.9, 0.9])
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

        global computing_flag
        rospy.sleep(3)

        pub_bool = Bool()
        pub_bool.data = True
        pub2.publish(pub_bool)

        mode = 0 
        flag_m = 0 
        computing_flag = False

        #print "================ mode1 flag error!!=================="
        
        pub_bool = Bool()
        pub_bool.data = True
        pub.publish(pub_bool)
        #print(" Published bool True")
        #print(" done")
        #print "==============================================="
        start = 0

def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    sub = rospy.Subscriber('/pick_pose', Pose, callback)
    sub2 = rospy.Subscriber('/camera_pose', Pose, callback2)
    pub_only()
    rospy.spin()    
    
if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
