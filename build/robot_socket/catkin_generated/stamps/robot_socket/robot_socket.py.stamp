#! /usr/bin/env python3
# coding=utf-8
# NOQA: E402

import socketserver
import socket
import time
import rospy
import signal
import json
import sys

from PIL import Image
import os, glob
import yaml
sys.path.append('/home/hjx/ROS/robot_socket/src/robot_socket/scripts')
from robot_map import RobotMap

# 打印彩色字符
def colormsg(msg: str, color: str = "", timestamp: bool = True):
    str = ""
    if timestamp:
        str += time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())) + "  "
    if color == "red":
        str += "\033[1;31;40m"
    elif color == "green":
        str += "\033[1;32;40m"
    elif color == "yellow":
        str += "\033[1;33;40m"
    else:
        print(str + msg)
        return
    str += msg + "\033[0m"
    print(str)

class RobotSocket(socketserver.BaseRequestHandler):
    def setup(self) -> None:
        return super().setup()
    

    def handle(self):
        conn = self.request
        colormsg("监听到SOCKET请求",color="yellow")
        result = conn.recv(10240)
        data = json.loads(result.decode())
        if data["method"] == "exit":
            colormsg("接收到推出socket命令", color="red")
            server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            conn.close()
            server.shutdown()
            exit()
        
        # 获取底盘当前地图信息
        if data["method"] == "getMap":
            dic = {}
            dic["data"] = {}
            map_path, origin_x,origin_y,resolution,map_size ,map_w,map_h = RobotMap().getMap()
            #res = '{"method":"getMap","data":["map_path":"'+map_path+'","origin_x":"'+str(origin_x)+'","origin_y":"'+str(origin_y)+'","resolution":"'+str(resolution)+'","map_size":"'+str(map_size)+'","map_w":"'+str(map_w)+'","map_h":"'+str(map_h)+'"],"status":0,"msg":"success"}'
            dic["method"] = "getMap"
            dic["data"]["map_path"] = map_path
            dic["data"]["origin_x"] = origin_x
            dic["data"]["origin_y"] = origin_y
            dic["data"]["resolution"] = resolution
            dic["data"]["map_size"] = map_size
            dic["data"]["map_w"] = map_w
            dic["data"]["map_h"] = map_h
            dic["status"] = 0
            dic["msg"] = "success"
            res = json.dumps(dic)
            colormsg("返回的数据是："+str(res.encode()),color="green")
            conn.send(res.encode())
        # 获取底盘原点坐标像素点位
        if data["method"] == "getOriginalPoint":
            dic = {}
            dic["data"] = {}
            x,y = RobotMap().getOriginalPoint()
            dic["data"]["x"] = x
            dic["data"]["y"] = y
            dic["status"] = 0
            dic["msg"] = "success"
            res = json.dumps(dic)
            colormsg("返回的数据是："+str(res.encode()),color="green")
            conn.send(res.encode())
        # 以地图图片像素位置获取世界坐标位置
        if data["method"] == "pixelToMapPoint":
            dic = {}
            dic["data"] = {}
            dic["method"] = "pixelToMapPoint"
            wx,wy = RobotMap().pixelToMapPoint(x=data["x"], y=data["y"])
            dic["data"]["wx"] = wx
            dic["data"]["wy"] = wy
            dic["status"] = 0
            dic["msg"] = "success"
            res = json.dumps(dic)
            colormsg("返回的数据是："+str(res.encode()),color="green")
            conn.send(res.encode())
        # 点击地图图片移动机器人
        if data["method"] == "moveMapPoint":
            dic = {}
            dic["data"] = {}
            dic["method"] = "moveMapPoint"
            res = RobotMap().moveMapPoint(x=data["x"], y=data["y"], metre=data["metre"])
            colormsg("返回的数据是："+str(res.encode()),color="green")
            conn.send(res.encode())
        # 获取当前坐标
        if data["method"] == "getCurrentPoint":
            dic = {}
            dic["data"] = {}
            dic["method"] = "getCurrentPoint"
            x, y, z, qua_x, qua_y, qua_z, qua_w, th,p_x, p_y = RobotMap().getCurrentPoint()
            dic["data"]["x"] = x
            dic["data"]["y"] = y
            dic["data"]["p_x"] = p_x
            dic["data"]["p_y"] = p_y
            dic["data"]["z"] = z
            dic["data"]["qua_x"] = qua_x
            dic["data"]["qua_y"] = qua_y
            dic["data"]["qua_z"] = qua_z
            dic["data"]["qua_w"] = qua_w
            dic["data"]["th"] = th
            dic["status"] = 0
            dic["msg"] = "success"
            res = json.dumps(dic)
            colormsg("返回的数据是："+str(res.encode()),color="green")
            conn.send(res.encode())
        # 标记原始坐标
        if data["method"] == "initialPoint":
            dic = {}
            dic["data"] = {}
            dic["method"] = "initialPoint"
            RobotMap().initialPoint()
            dic["status"] = 0
            dic["msg"] = "success"
            res = json.dumps(dic)
            colormsg("返回的数据是："+str(res.encode()),color="green")
            conn.send(res.encode())
        # 地图格式转换
        if data["method"] == "pgmToJpg":
            self.batch_image("/home/hjx/ROS/map","/home/hjx/ROS/map")
            dic["status"] = 0
            dic["msg"] = "success"
            res = json.dumps(dic)
            colormsg("返回的数据是："+str(res.encode()),color="green")
            conn.send(res.encode())

    """
    将pgm格式地图转为jpg格式
    """
    def batch_image(self,in_dir, out_dir):
        if not os.path.exists(out_dir):
            print(out_dir, 'is not existed.')
            os.mkdir(out_dir)
        
        if not os.path.exists(in_dir):
            print(in_dir, 'is not existed.')
            return -1
        count = 0
        for files in glob.glob(in_dir+'/*'):
            filepath, filename = os.path.split(files)
            
            out_file = filename[0:9] + '.jpg'
            # print(filepath,',',filename, ',', out_file)
            im = Image.open(files)
            new_path = os.path.join(out_dir, out_file)
            print(count, ',', new_path)
            count = count + 1
            im.save(os.path.join(out_dir, out_file))


def handle(a, b):
    server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    sys.exit()
    exit()


if __name__ == "__main__":
    rospy.init_node("robot_socket", anonymous=True)
    while True:
        try:
            signal.signal(signal.SIGINT, handler=handle)
            signal.signal(signal.SIGTERM, handler=handle)
            server = socketserver.ThreadingTCPServer(('192.168.0.101', 9999), RobotSocket, False)
            server.allow_reuse_address = True
            server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
            server.server_bind()
            server.server_activate()
            colormsg("Socket接口启动成功",color="green")
            
            server.serve_forever()
            break
        except OSError:
            colormsg("Socket接口启动失败，10秒后重试",color="red")
            time.sleep(10)
            
    
    