#! /usr/bin/env python3
# coding=utf-8
# NOQA: E402

import time
import rospy
import signal
import json
import sys
import os
import yaml
import actionlib
from tf_conversions import transformations
from math import pi
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from PIL import Image



class RobotMap():
    def __init__(self) -> None:
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        # 订阅move_base服务器的消息
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(60))
        try:
            # 读取配置文件
            yaml_file = open("config/robot_config.yaml","r", encoding="utf-8")
            # yaml转json
            j = yaml.load(yaml_file, Loader=yaml.FullLoader)
            # 获取地图yaml路径
            map_yaml_path = j["chassis"]["map_yaml_path"]
            # 获取机器人底盘编号
            self.chassis_code = j["chassis"]["chassis_code"]
        except IOError as e:
            print(e)
        finally:
            if yaml_file is not None:
                yaml_file.close()
        f = open(map_yaml_path, 'r', encoding='utf-8')

        map_data = yaml.load(f, Loader=yaml.FullLoader)
        print(map_data["image"])
        map_file = Image.open(map_data["image"])
        # 当前地图图片路径
        self.map_path = map_data["image"]
        # 底盘在地图原始坐标x
        self.origin_x = map_data["origin"][0]
        # 底盘在地图原始坐标y
        self.origin_y = map_data["origin"][1]
        # 地图分辨率
        self.resolution = map_data["resolution"]
        # 地图size
        self.map_size = map_file.size
        # 地图宽高像素
        self.map_w = map_file.width
        self.map_h = map_file.height

    # 获取地图信息
    def getMap(self):
        return self.map_path, self.origin_x, self.origin_y, self.resolution, self.map_size ,self.map_w, self.map_h
    # 获取底盘原点坐标像素点位
    def getOriginalPoint(self):
        x = (0-(self.origin_x)) / self.resolution
        y = (0-(self.origin_y)) / self.resolution
        return x,y
    # 获取当前坐标
    def getCurrentPoint(self):
        (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        euler = transformations.euler_from_quaternion(rot)
        #print euler[2] / pi * 180
        x = trans[0]
        y = trans[1]
        p_x = (0-x) / self.resolution
        p_y = (0-y) / self.resolution
        z = trans[2]
        qua_x = rot[0]
        qua_y = rot[1]
        qua_z = rot[2]
        qua_w = rot[3]
        th = euler[2] / pi * 180
        return x, y, z, qua_x, qua_y, qua_z, qua_w, th, p_x, p_y
    # 根据地图图片像素转换为世界坐标
    def pixelToMapPoint(self,x=0, y=0):
        wx = x * self.resolution + self.origin_x
        wy = (self.map_h - y) * self.resolution +self.origin_y
        print(wx)
        print("=============")
        print(wy)
        return wx, wy
    
    # 根据图片像素转换成坐标，并让机器人移动到坐标
    def moveMapPoint(self,x=0,y=0, theta=0.01, qua_z=0.01, qua_w=0.999 ,metre=True):
        if metre:
            wx = x
            wy = y
        else:
            wx, wy = self.pixelToMapPoint(x,y)
        target_msg = PoseStamped()
        

        target_msg.header.frame_id = "map"
        target_msg.header.stamp = rospy.Time.now()
        target_msg.pose.position.x = wx
        target_msg.pose.position.y = wy
        target_msg.pose.position.z = 0

        target_msg.pose.orientation.x = 0
        target_msg.pose.orientation.y = 0
        target_msg.pose.orientation.z = qua_z
        target_msg.pose.orientation.w = qua_w
        #rospy.init_node("socket_move")
        pub_move = rospy.Publisher("/move_base_simple/goal", PoseStamped,queue_size=1)
        # 发布一次空值，ROS话题发布，第一次有可能会丢失，所以先发布一次空值
        pub_move.publish(PoseStamped())
        time.sleep(0.3)
        while True:
            pub_move.publish(target_msg)
            print("---------------------移动话题已发布"+str(target_msg))
            break
        msg = "开始移动机器人坐标为:position_x:"+str(target_msg.pose.position.x)+"  position_y:"+str(target_msg.pose.position.y)+"  position_z:"+str(target_msg.pose.position.z)+"  orientation_x:"+str(target_msg.pose.orientation.x)+"  orientation_y:"+str(target_msg.pose.orientation.y)+"  orientation_z:"+str(target_msg.pose.orientation.z)+"  orientation_w:"+str(target_msg.pose.orientation.w)
        #rospy.signal_shutdown("closed!")
        return msg


    # 标记初始地点
    def initialPoint(self):
        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, latch=True, queue_size=1)
        pose = {'x': -3.032415, 'y': 1.02663, 'a': 0.0099}
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.pose.position.x = pose['x']
        p.pose.pose.position.y = pose['y']
        p.pose.pose.position.z = pose['a']
        (p.pose.pose.orientation.x,
         p.pose.pose.orientation.y,
         p.pose.pose.orientation.z,
         p.pose.pose.orientation.w) = transformations.quaternion_from_euler(0, 0, pose['a'])
        p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        p.pose.covariance[6 * 3 + 3] = pi / 12.0 * pi / 12.0
        time.sleep(0.1)
        pub.publish(p)

