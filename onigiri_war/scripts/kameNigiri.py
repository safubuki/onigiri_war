#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -----------------------------------------------------------------------
# ファイル名  ：kameNigiri.py
# 機能概要  ：おにぎり大会用（予選）
#           対塩にぎり用で、とにかく相手付近に行き、速やかに一方勝ちする。
# 作成日時  ：2018.07.30
# 備考  ：本選ではもう少し技術っぽいことする。頑張る。
# -----------------------------------------------------------------------

# Import
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math

# 下記は後ほど使用できたらいいな、、、
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
# from move_base_msgs.msg import MoveBaseActionResult
# import csv  # csvファイル操作のため
# import os   # ファイルパス操作のため

# PythonでEnum的なことを実現
class ActMode():
    START   = 0
    RUN     = 1
    TURN    = 2
    IPPON   = 3

class RunState():
    VART    = 0
    HORI    = 1

class KameNigiriBot():
    def __init__(self, bot_name):
        ### Parameter Settings
        # bot name
        self.name = bot_name
        # 動作状態
        self.mode = ActMode.START   # mode ：開始
        self.state = RunState.VART  # Run状態(Runのサブ状態)：縦
        self.pi_val = math.pi
        # Odometry情報
        self.odom_pos_x = 0
        self.odom_ori_z = 0
        # Odometry しきい値
        self.odom_pos_x_thresh = 0
        self.odom_ori_z_thresh = 0
        # # 超音波センサ検知 (超音波センサ 使うとき有効)
        # self.us_left_detect = False
        # self.us_right_detect = False
        # # スタート/ゴール座標
        # self.c_data          = []  # csvデータ
        # self.start_pos       = []  # スタート地点 ：pos_x,pos_y,ori_z,ori_w  リスト型
        # self.goal_pos        = []  # ゴール地点   ：pos_x,pos_y,ori_z,ori_w  リスト型
        ### Publisher を ROS Masterに登録
        # Velocity
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        ### Subscriber を ROS Masterに登録
        # Odometry情報
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        # # 超音波センサ
        # self.sub_us_left = rospy.Subscriber('us_left', LaserScan, self.us_left_callback, queue_size=1)
        # self.sub_us_right = rospy.Subscriber('us_right', LaserScan, self.us_right_callback, queue_size=1)

    # # CSVファイルから座標を取得する関数
    # def csv_data(self):
    #     # csvファイルをOpen
    #     csv_pass = os.path.dirname(__file__) + "/Start_Goal.csv"
    #     csv_file = open(csv_pass, "r")
    #     # データ読み込み
    #     pos_data = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True)
    #     # 最初の一行をヘッダーとして取得
    #     header = next(pos_data)
    #     # 各行のデータを抜き出し
    #     for row in pos_data:
    #         # データ保存用のリストにcsvファイルから取得したデータを保存する
    #         # appendでリストに別のリストとして要素を追加する
    #         self.c_data.append(row)
    #
    #     # スタート/ゴール地点設定
    #     # ※固定で1行目Start、2行目Goalにする(暫定)。
    #     #   ホントは外部操作契機で複数の地点の中から任意の地点を設定できるようにしたい。
    #     self.start_pos = self.c_data[0]
    #     self.goal_pos  = self.c_data[1]

    # ### 超音波センサTopic Subscribe時のCallback関数(左) (超音波センサ 使うとき有効)
    # def us_left_callback(self, sens):
    #     if sens.ranges[0] < 0.4:           # [unit :m]
    #         self.us_left_detect = True
    #     else:
    #         self.us_left_detect = False
    #     print('us_left = ',self.us_left_detect)
    #
    # ### 超音波センサTopic Subscribe時のCallback関数(右) (超音波センサ 使うとき有効)
    # def us_right_callback(self, sens):
    #     if sens.ranges[0] < 0.4:           # [unit :m]
    #         self.us_right_detect = True
    #     else:
    #         self.us_right_detect = False
    #     print('us_right = ',self.us_right_detect)

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
            if self.mode == ActMode.START:          # STARTモード  ：起動後の角度設定
                # print('mode = START')
                self.odom_pos_x_thresh = 0.00
                self.odom_ori_z_thresh = 0.34
                # print('odom_ori_z', self.odom_ori_z)
                if self.odom_ori_z >= self.odom_ori_z_thresh:
                    self.vel_ctrl(0.00, 0.00, 0.00)
                    self.odom_pos_x_thresh = 2.00
                    self.odom_ori_z_thresh = 0.00
                    self.mode = ActMode.RUN
                else:
                    self.vel_ctrl(0.00, 0.00, 0.30)
            elif self.mode == ActMode.RUN:          # RUNモード  ：前進
                # print('odom_pos_x', self.odom_pos_x)
                if self.odom_pos_x >= self.odom_pos_x_thresh:
                    self.vel_ctrl(0.00, 0.00, 0.00)
                    if self.state == RunState.VART:  # 縦方向移動時
                        self.odom_pos_x_thresh =  0.00
                        self.odom_ori_z_thresh = -0.36
                        self.mode = ActMode.TURN
                    else:  # 横方向移動時
                        self.odom_pos_x_thresh =  0.00
                        self.odom_ori_z_thresh = -0.95
                        self.mode = ActMode.IPPON
                else:
                    # self.vel_ctrl(0.25, 0.00, 0.00)
                    self.vel_ctrl(0.30, 0.00, 0.00)
            elif self.mode == ActMode.TURN:     # TURNモード ：かどで旋回する
                # print('odom_ori_z', self.odom_ori_z)
                if self.odom_ori_z <= self.odom_ori_z_thresh:
                    self.vel_ctrl(0.00, 0.00, 0.00)
                    # 動作モード、Run状態を設定する
                    if self.state == RunState.VART:
                        self.odom_pos_x_thresh = 3.90
                        self.odom_ori_z_thresh = 0.00
                        self.state = RunState.HORI
                        self.mode = ActMode.RUN
                    else:
                        pass
                        # # 動作モードを変更
                        # self.mode = ActMode.IPPON
                else:
                    self.vel_ctrl(0.00, 0.00, -0.30)
            elif self.mode == ActMode.IPPON:     # IPPONモード ：塩おにぎりから一本取る
                # print('mode = IPPON')
                if self.odom_ori_z <= self.odom_ori_z_thresh:
                    self.vel_ctrl(0.00, 0.00, 0.00)
                else:
                    self.vel_ctrl(0.00, 0.00, -1.50)       # 急速旋回
            else:
                pass
            # Loop時にSleepする(100msスリープ)
            rospy.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node('kameNigiri_node')
    bot = KameNigiriBot('kame')
    # bot.csv_data()
    bot.strategy()
