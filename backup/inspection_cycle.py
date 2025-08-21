import time
import cv2
import numpy as np
import sys
import os
import socket
import logging
import json
import xml.etree.ElementTree as ET

current_file_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_file_dir)

from control.robotdog.dog_command import DogControl
from control.robot_arm.realman.real_robot import real_robot

# 日志配置
log_path = r'C:\Users\Lenovo\Desktop\abcd\robot_dog\log\inspection_log.txt'
# 导航返回错误码字典

errorStatus_dict = {
    "0000": "异常码默认值",
    "8962": "单点巡检任务被取消",
    "8960": "单点巡检任务执行完成",
    "41729": "运动状态异常，任务失败(软急停、摔倒)",
    "41730": "电量过低，任务失败",
    "41731": "电机过温异常，任务失败",
    "41732": "正在使用充电器充电，任务失败",
    "41745": "导航进程未启动，无法下发任务",
    "41746": "导航模块通讯异常，无法下发任务",
    "41747": "定位状态持续异常(超过 30s)",
    "41748": "地形模块状态异常",
    "41761": "发送起立失败",
    "41762": "执行起立失败",
    "41763": "切换力控失败",
    "41764": "切换行走模式失败",
    "41765": "趴下失败",
    "41766": "被软急停",
    "41767": "切换步态失败",
    "41768": "切换导航模式失败",
    "41769": "切换手动模式失败",
    "41770": "切换正常/匍匐身高风险失败",
    "41777": "切换避障模块的速度输入源失败",
    "41778": "设置地形图参数失败",
    "41793": "当前正在执行任务，下发新任务失败",
    "41794": "调度退出自主充电失败",
    "41795": "退出自主充电执行失败",
    "41796": "调度进入自主充电失败",
    "41797": "进入自主充电执行失败",
    "41798": "退桩后重定位失败",
    "41799": "开启累积积失败",
    "41800": "关闭累积积失败",
    "41801": "切换地图失败",
    "41802": "存在上位机连接断开，自动停止任务",
    "41803": "持续停滞异常，导航失败",
    "41804": "导航全局规划失败",
    "41805": "持续导航速度未刷新，导航失败",
    "41806": "自主充电流程中，下发任务失败",
    "41881": "重定位失败",
    "41983": "进程手动重启中，停止任务"
}

logging.basicConfig(
    filename=log_path,
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(message)s'
)

def log_dict(result_dict, prefix=""):
    if type(result_dict) is not dict:
        logging.info('%s%s', prefix, result_dict)
    else:
        logging.info('%s%s', prefix, json.dumps(result_dict, ensure_ascii=False))

def main(socket_ip, point_path):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((socket_ip, 30000))
    dog_control = DogControl(client_socket, point_path)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口重用
    server_socket.bind(('0.0.0.0', 30000))  # 绑定所有接口的30000端口
    server_socket.listen(2)  # 监听1个连接请求
    print("服务器启动，等待客户端连接...")
    robot = real_robot(hand_to_camera=np.array([[ 0.4850078, -0.81452437, 0.31830407, 0],
                                                [ 0.87364389,  0.43509852, -0.21779724, 0],
                                                [ 0.03890753,  0.38371777,  0.92263041, 0],
                                                [ 0, 0, 0, 1]]),
                        w=640,   
                        h=480,
                        server_socket=server_socket)

    # ans_localization = dog_control.localization_init(-1.397, 0.028, 0.0189, -1.637)
    # log_dict(ans_localization, "Localization: ")
    # if ans_localization['ErrorCode'] == '0':
    #     print('Localization initialized successfully.')
    # else:
    #     print('Localization initialization failed.')
        

    ans_battery = dog_control.battery_state()
    log_dict(ans_battery, "Battery: ")
    if int(ans_battery['BatteryLevel']) >= 70:
        print('battery has enough energy for inspection mission')
    else:
        print('dog needs food now!')
    

    mission_points = dog_control.read_points()
    log_dict(len(mission_points), "Mission Points Count: ")

    if mission_points is None:
        print('No mission points found.')
        return

    # 遍历任务点并发送导航任务
    try:
        mission_id = 5
        for p in mission_points:
            # if p[-1] < 10:
            #     continue
            print('number id of point: ', p[-1])
            client_socket.sendall(p[0])

            ans_nav = b''
            while True:
                part = client_socket.recv(4096)
                ans_nav += part
                if b'</PatrolDevice>' in ans_nav:
                    break
            # ans = client_socket.recv(1024)
            xml_bytes = ans_nav[p[2]:]
            xml_str = xml_bytes.decode('utf-8')
            # 解析xml
            root = ET.fromstring(xml_str)

            respo = root.find('.//ErrorCode').text
            if respo == '0':
                if p[1] == 1:
                    print('这是任务点')
                    mission_id += 1
                    time.sleep(1)
                    flag = robot.robot_find_task(mission_id)  #输入id
                    if flag:
                        log_dict(p[-1], 'arm mission success')
                        continue
                    else:
                        log_dict(p[-1], 'arm mission failed')
                        robot.move_by_joint([-5.215000152587891, -104.43299865722656, 142.6199951171875, -176.7429962158203, -99.36100006103516, 176.81500244140625])
                        dog_control.motion_control(15, -1)
                        break

            elif respo == '1':
                print('任务失败，导航任务目标点编号是 ', root.find('.//Value').text, '任务失败原因是 ', root.find('.//ErrorStatus').text, ': ', errorStatus_dict.get(root.find('.//ErrorStatus').text, '未知错误'))
                break
    except KeyboardInterrupt:
        print('keyboard interrupt')
    finally:
        # dog_control.motion_control(15, -1)
        client_socket.close()
        server_socket.close()
        robot.delete_robot()


if __name__ == "__main__":
    socket_ip = "192.168.1.106"
    point_path = r'C:\Users\Lenovo\Desktop\abcd\robot_dog\nav_path\30f233b0_factory807.json'
    main(socket_ip, point_path)
