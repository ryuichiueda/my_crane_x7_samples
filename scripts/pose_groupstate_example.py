#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, moveit_commander, geometry_msgs.msg, rosnode
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()                #MoveIt!に指令を送るコマンド
    arm = moveit_commander.MoveGroupCommander("arm")         #armグループを操作するためのオブジェクトを生成
    arm.set_max_velocity_scaling_factor(0.1)                 #速度を1/10におさえる
    gripper = moveit_commander.MoveGroupCommander("gripper") #gripperグループを操作するためのオブジェクト生成

    while not 'rviz' in ','.join(rosnode.get_node_names()):  #rvizが立ち上がるのを待つ
        rospy.sleep(0.1)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    print("home")
    arm.set_named_target("home")
    arm.go()

    # SRDFに定義されている"vertical"の姿勢にする
    print("vertical")
    arm.set_named_target("vertical")
    arm.go()

    # ハンドを少し閉じる
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    # 手動で姿勢を指定するには以下のように指定
    """
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.0
    target_pose.position.y = 0.0
    target_pose.position.z = 0.624
    q = quaternion_from_euler( 0.0, 0.0, 0.0 )
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target( target_pose )	# 目標ポーズ設定
    arm.go()							# 実行
    """

    # 移動後の手先ポーズを表示
    arm_goal_pose = arm.get_current_pose().pose
    print("Arm goal pose:")
    print(arm_goal_pose)
    print("done")


if __name__ == '__main__':
    ### この例外処理は・・・たぶんあまり意味がない（mainを呼び出すだけで十分） ###
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
