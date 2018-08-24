#!/usr/bin/env python
# -*- coding: utf-8 -*-

# -----------------------------------------------------------------------
# ファイル名 ：kameNigiri_evolution.py
# 機能概要  ：One Month ROBOCON（決勝用）
#           ・move_base, amcl, map_serverを使用
#           ・スタート/ゴール地点は"Start_Goal.csv"ファイルで設定する
# 作成日時  ：2018.08.22
# 備考     ：move_base,amclのパラメータ調整ができておらず、
# 		   自己位置を見失いがちなのがたまにキズ、、、
# -----------------------------------------------------------------------

# Import
import rospy
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
import csv  # csvファイル操作のため
import os   # ファイルパス操作のため

# PythonでEnum的なことを実現
class MoveState():
    STOP         = 0
    RUN          = 1
    DEFENSE      = 2

class KameNigiriBot():
    def __init__(self, bot_name):
        ### Parameter Settings
        self.move_state      = MoveState.STOP   # 移動状態           ：停止
        # CSV ファイルから取り出したデータ保存用リスト
        self.c_data          = []               # csvデータ
        self.c_data_cnt      = 0                # csvデータ順次取得のためのカウンタ
        # simple/goal用のシーケンス番号 ※これ無いとエラーになるため必要
        self.goal_seq_no     = 0

        ### Publisher を ROS Masterに登録
        self.pub_goal        = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        ### Subscriber を ROS Masterに登録
        self.sub_goal_result = rospy.Subscriber("move_base/result", MoveBaseActionResult, self.result_callback, queue_size=1)

    # CSVファイルから座標を取得する関数
    def csv_data(self):
        # csvファイルをOpen
        csv_pass = os.path.dirname(__file__) + "/position_list.csv"
        csv_file = open(csv_pass, "r")
        # データ読み込み
        pos_data = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True)
        # 最初の一行をヘッダーとして取得
        header = next(pos_data)
        # 各行のデータを抜き出し
        for row in pos_data:
            # データ保存用のリストにcsvファイルから取得したデータを保存する
            # appendでリストに別のリストとして要素を追加する
            self.c_data.append(row)

    # "move_base/result" TopicをSubscribeしたときのコールバック関数
    # ゴール座標への到達を検知
    def result_callback(self,goal_result):
        print('result_callback')    # ★★デバッグ
        print(goal_result)          # ★★デバッグ
        if goal_result.status.status == 3:  # ゴールに到着 (★失敗時のAbort:4も対応する)
            if self.move_state == MoveState.RUN:
                self.move_state = MoveState.STOP         # 移動状態を更新：停止

    # "move_base_simple/goal"Topicのパラメータを設定し、Publishする関数 (引数はリスト型で渡す)
    def simple_goal_publish(self,pos_list):
        # Goal Setting
        goal = PoseStamped()
        goal.header.seq = self.goal_seq_no
        self.goal_seq_no += 1                      # シーケンス番号を更新
        goal.header.frame_id = 'red_bot/base_link' # base_linkで座標系で指定する
        goal.header.stamp = rospy.Time.now()  # タイムスタンプは今の時間

        # メモ：座標、回転角度はrviz上で事前確認の上設定
        # 　　　rostopic echo で initialpose, move_base_simple/goal を確認して設定 (暫定)

        # ** 位置座標　※kame_mapを基準に設定
        goal.pose.position.x = float(pos_list[0])
        goal.pose.position.y = float(pos_list[1])
        goal.pose.position.z = 0
        # ** 回転方向
        # クォータニオンを地道に設定する方法
        goal.pose.orientation.x = 0
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = float(pos_list[2])
        goal.pose.orientation.w = float(pos_list[3])
        # debug print
        print(goal)
        # 実際にTopicを配信する
        self.pub_goal.publish(goal)

    # ロボット動作のメイン処理
    def strategy(self):
        # 起動直後ウェイト
        rospy.sleep(5.0)  # 起動後、ウェイト（調整値）
        while not rospy.is_shutdown():
            if self.move_state == MoveState.STOP:
                pos_info = self.c_data[self.c_data_cnt]
                if pos_info[4] == "way_point":
                    self.c_data_cnt += 1                    # csvデータカウンタを更新
                    self.simple_goal_publish(pos_info)      # 座標は後ほどCSVから取得できるように
                    self.move_state = MoveState.RUN
                else:
                    self.move_state = MoveState.DEFENSE
                    pass
            else:
                pass

            print('move_state')  # ★★デバッグ
            print(self.move_state)
            rospy.sleep(1)  # 1秒Wait

if __name__ == "__main__":
    rospy.init_node('kameNigiri_evo_node')
    bot = KameNigiriBot('kame')
    bot.csv_data()
    bot.strategy()
