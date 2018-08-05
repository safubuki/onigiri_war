#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -----------------------------------------------------------------------
# ファイル名  ：kameNigiri.py
# 機能概要  ：おにぎり大会用（予選）
#           対塩にぎり用で、とにかく相手付近に行き、速やかに一方勝ちする。
# 作成日時  ：2018.07.30
# 備考  ：予選用
# -----------------------------------------------------------------------

# Import
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class ActMode():
    START   = 0     # 開始
    RUN     = 1     # 前進
    TURN    = 2     # 旋回
    IPPON   = 3     # 一本

class RunState():
    VART    = 0     # 前進：縦
    HORI    = 1     # 前進：横

class KameNigiriBot():
    def __init__(self, bot_name):
        ### Parameter Settings
        # bot name
        self.name = bot_name
        # 動作状態
        self.mode = ActMode.START   # モード：開始
        self.state = RunState.VART  # 前進：縦
        self.pi_val = math.pi
        # Odometry情報
        self.odom_pos_x = 0
        self.odom_ori_z = 0
        # Odometry しきい値 （STARTモードで使用する値を設定）
        self.odom_pos_x_thresh = 0.00
        self.odom_ori_z_thresh = 0.34
        ### Publisher を ROS Masterに登録
        # Velocity
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        ### Subscriber を ROS Masterに登録
        # Odometry情報
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

    ### Odometry情報Topic Subscribe時のCallback関数
    def odom_callback(self, odom_val):
        # Odometry情報を保存する
        self.odom_pos_x = odom_val.pose.pose.position.x
        self.odom_ori_z = odom_val.pose.pose.orientation.z

    ### cmd_vel パラメータ設定＆Topic Publish関数
    def vel_ctrl(self, line_x, line_y, ang_z):
        vel_msg = Twist()
        vel_msg.linear.x = line_x
        vel_msg.linear.y = line_y
        vel_msg.angular.z = ang_z
        self.vel_pub.publish(vel_msg)

    ### ロボット動作のメイン処理
    def strategy(self):
        # 起動直後ウェイト
        rospy.sleep(1.0)  # 起動後、ウェイト（調整値）
        while not rospy.is_shutdown():
            if self.mode == ActMode.START:      # STARTモード  ：開始後の動作
                # しきい値を超えたら、一旦停止、次の状態のしきい値設定、モード変更
                if self.odom_ori_z >= self.odom_ori_z_thresh:
                    # 停止
                    self.vel_ctrl(0.00, 0.00, 0.00)
                    # しきい値設定
                    self.odom_pos_x_thresh = 2.00
                    self.odom_ori_z_thresh = 0.00
                    # モード変更
                    self.mode = ActMode.RUN             # 前進モード
                else:
                    self.vel_ctrl(0.00, 0.00, 0.30)
            elif self.mode == ActMode.RUN:      # RUNモード  ：前進
                if self.odom_pos_x >= self.odom_pos_x_thresh:
                    # 停止
                    self.vel_ctrl(0.00, 0.00, 0.00)
                    # 縦方向の移動 or 横方向の移動でしきい値、次状態を変える
                    if self.state == RunState.VART:     # 縦方向移動時
                        # しきい値設定
                        self.odom_pos_x_thresh =  0.00
                        self.odom_ori_z_thresh = -0.36
                        # モード変更
                        self.mode = ActMode.TURN        # 旋回モード
                    else:                               # 横方向移動時
                        # しきい値設定
                        self.odom_pos_x_thresh =  0.00
                        self.odom_ori_z_thresh = -0.95
                        # モード変更
                        self.mode = ActMode.IPPON       # 一本モード
                else:
                    self.vel_ctrl(0.30, 0.00, 0.00)
            elif self.mode == ActMode.TURN:     # TURNモード ：旋回
                if self.odom_ori_z <= self.odom_ori_z_thresh:
                    # 停止
                    self.vel_ctrl(0.00, 0.00, 0.00)
                    # しきい値設定
                    self.odom_pos_x_thresh = 3.90
                    self.odom_ori_z_thresh = 0.00
                    # モード・Run状態変更
                    self.state = RunState.HORI          # 前進：横
                    self.mode = ActMode.RUN             # 前進モード
                else:
                    self.vel_ctrl(0.00, 0.00, -0.30)
            elif self.mode == ActMode.IPPON:    # IPPONモード ：一本取る
                if self.odom_ori_z <= self.odom_ori_z_thresh:
                    self.vel_ctrl(0.00, 0.00, 0.00)     # 停止
                else:
                    self.vel_ctrl(0.00, 0.00, -1.50)    # 急速旋回
            else:
                pass
            # Loop時にSleepする(100msスリープ)
            rospy.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node('kameNigiri_node')
    bot = KameNigiriBot('kame')
    bot.strategy()
