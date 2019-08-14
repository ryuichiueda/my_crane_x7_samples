#!/usr/bin/env python
# -*- coding: utf-8 -*-

# このサンプルは実機動作のみに対応しています
# fake_execution:=trueにすると、GripperCommandActionのサーバが立ち上がりません

import sys
import rospy
import time
import actionlib
import math
from std_msgs.msg import Float64
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)

class GripperClient(object):
    def __init__(self):
        # アクションクライアント（クライアント）を作っている
        # クライアントはアクションサーバにゴール（目標の姿勢）を投げて実行してもらったり、
        # 途中で動作を中断してもらったりする。
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self.clear() # 不要かもしれないけどゴールを初期化

        # アクションサーバを待つ（10秒でタイムアウト）
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)

    # アクションサーバにゴールを送るメソッド
    def command(self, position, effort):
        self._goal.command.position = position # グリッパの姿勢（開く角度）
        self._goal.command.max_effort = effort # トルク
        self._client.send_goal(self._goal,feedback_cb=self.feedback) # ゴールをサーバに送信

    # サーバが仕事をしている途中で呼ばれるメソッド（このサンプルでは呼ばれてない）
    def feedback(self,msg):
        print("feedback callback")
        print(msg)

    # 途中で中断するためのメソッド（これもこのサンプルでは使われていない）
    def stop(self):
        self._client.cancel_goal()

    # サーバからの結果待ちのためのメソッド
    def wait(self, timeout=0.1):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result() # 結果をもらう

    # ゴール情報のクリアのためのメソッド
    def clear(self):
        self._goal = GripperCommandGoal()

def main():
    # ROSノードとしての初期化
    # ROSの一つのプログラムは「ノード」と呼ばれる
    rospy.init_node("gipper_action_client")
    gc = GripperClient() # 上で作ったクラスのインスタンス

    # 45度にハンドを開く
    print "Open Gripper."
    gripper = 45.0
    gc.command(math.radians(gripper), 1.0) # サーバに指令を送信
    result = gc.wait(2.0) # 2秒待つ
    print result
    time.sleep(1)

    # ハンドを閉じる
    print "\nClose Gripper."
    gripper = 0.0
    gc.command(math.radians(gripper),1.0)
    result = gc.wait(2.0)
    print result
    time.sleep(1)

if __name__ == "__main__":
    main()
