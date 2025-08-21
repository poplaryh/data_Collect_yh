# -*- coding: UTF-8 -*-
import socket
import struct
import xml.etree.ElementTree as ET
import time
import json
import threading
import random
import string



class DogControl():
    def __init__(self, socket, point_path=None):
        self.client_socket = socket
        self.point_path = point_path

    def read_points(self):
        '''
        返回值：

        列表：[messages, PonintInfo, header_length, mission_id, crouch, value]

        - messages: 包含每个点位信息的可直接进行TCP发送的参数, header+xml_string.encode('utf-8')
        - PointInfo: 目标点位类型（过渡点 0, 任务点 1, 充电点 2)
        - header_length: header的长度
        - mission_id: 任务点编号
        - crouch: 下蹲标识 (0: 不需要下蹲, 1: 需要下蹲)
        - value: 点的编号（0, 1, 2, ...）

        '''
        path = self.point_path
        if not path:
            raise ValueError("point_path must be provided")
        output = []
        # 读取JSON文件
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)

        for idx, item in enumerate(data):
            root = ET.Element("PatrolDevice")
            ET.SubElement(root, "Type").text = "1003"
            ET.SubElement(root, "Command").text = "1"
            ET.SubElement(root, "Time").text = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime())

            items = ET.SubElement(root, "Items")
            ET.SubElement(items, "Value").text = str(item.get("Value", 0))
            ET.SubElement(items, "MapID").text = str(item.get("MapID", 0))
            ET.SubElement(items, "PosX").text = str(item.get("PosX", 0) - 0.05)
            ET.SubElement(items, "PosY").text = str(item.get("PosY", 0))
            ET.SubElement(items, "PosZ").text = str(item.get("PosZ", 0))
            ET.SubElement(items, "AngleYaw").text = str(item.get("AngleYaw", 0))
            ET.SubElement(items, "PointInfo").text = str(item.get("PointInfo", 0))
            ET.SubElement(items, "Gait").text = str(item.get("Gait", 0))
            ET.SubElement(items, "Speed").text = str(item.get("Speed", 0))
            ET.SubElement(items, "Manner").text = str(item.get("Manner", 0))
            ET.SubElement(items, "ObsMode").text = str(item.get("ObsMode", 0))
            ET.SubElement(items, "NavMode").text = str(item.get("NavMode", 0))
            ET.SubElement(items, "Terrain").text = str(item.get("Terrain", 0))
            ET.SubElement(items, "Posture").text = str(item.get("Posture", 0))

            xml_str = ET.tostring(root, encoding='utf-8').decode('utf-8')
            xml_str = "<?xml version='1.0' encoding='utf-8'?>\n" + xml_str.replace('><', '>\n<')

            header = struct.pack("<BBBBHHq", *(0xeb, 0x90, 0xeb, 0x90, len(xml_str), 100, 0))
            output.append([header + xml_str.encode('utf-8'), int(item.get('PointInfo', 0)), len(header), item.get('mission_id'), item.get('crouch'), item.get('Value')])

        return output

    def nav(self, pause_t: int, points: list = None):
        '''
        pause_t: pause after arriving at mission point

        points: list of points, which is the return from read_points(point_file.json)
        '''
        if points is None:
            points = self.read_points()
        client_socket = self.client_socket

        for p in points:
            print('number id of point: ', p[-1])
            client_socket.sendall(p[0])

            ans = b''
            while True:
                part = client_socket.recv(4096)
                ans += part
                if b'</PatrolDevice>' in ans:
                    break    
            # ans = client_socket.recv(1024)
            xml_bytes = ans[p[2]:]
            xml_str = xml_bytes.decode('utf-8')
            # 解析xml
            root = ET.fromstring(xml_str)

            respo = root.find('.//ErrorCode').text
            if respo == '0':
                if p[1] == 10:
                    print(f'这是任务点: mission_id={p[3]}, crouch={p[4]}, value={p[5]}')
                    if p[4] == 1:
                        ans = self.motion_control(15, -1)
                        print(ans)
                    else:
                        print('no need to crouch down here')
                    time.sleep(pause_t)
            elif respo == '1':
                print('任务失败，导航任务目标点编号是 ', root.find('.//Value').text, '任务失败原因是 ', root.find('.//ErrorStatus').text)
                break

    def nav_cancel(self):
        client_socket = self.client_socket
        root = ET.Element("PatrolDevice")
        ET.SubElement(root, "Type").text = "1004"
        ET.SubElement(root, "Command").text = "1"
        ET.SubElement(root, "Time").text = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime())
        ET.SubElement(root, "Items")
        xml_string = ET.tostring(root,encoding='utf-8')
        xml_string = "<?xml version='1.0' encoding='utf-8'?>\n" + xml_string.decode('utf-8').replace('><','>\n<')
        header = struct.pack("<BBBBHHq", *(0xeb, 0x90, 0xeb, 0x90, len(xml_string), 100, 0))

        all_data = header + xml_string.encode('utf-8')
        client_socket.sendall(all_data)

        ans = b''
        while True:
            part = client_socket.recv(4096)
            ans += part
            if b'</PatrolDevice>' in ans:
                break
        # ans = client_socket.recv(1024)
        header_len = len(header)
        xml_bytes = ans[header_len:]
        xml_str = xml_bytes.decode('utf-8')
        # 解析xml
        root_out = ET.fromstring(xml_str)
        items_out = root_out.find('Items')
        out_dict = dict()
        for child in items_out:
            out_dict[str(child.tag)] = child.text

        return out_dict

    def nav_inquire(self):
        client_socket = self.client_socket
        root = ET.Element("PatrolDevice")
        ET.SubElement(root, "Type").text = "1007"
        ET.SubElement(root, "Command").text = "1"
        ET.SubElement(root, "Time").text = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime())
        ET.SubElement(root, "Items")
        xml_string = ET.tostring(root,encoding='utf-8')
        xml_string = "<?xml version='1.0' encoding='utf-8'?>\n" + xml_string.decode('utf-8').replace('><','>\n<')
        header = struct.pack("<BBBBHHq", *(0xeb, 0x90, 0xeb, 0x90, len(xml_string), 100, 0))

        all_data = header + xml_string.encode('utf-8')
        client_socket.sendall(all_data)

        ans = b''
        while True:
            part = client_socket.recv(4096)
            ans += part
            if b'</PatrolDevice>' in ans:
                break
        # ans = client_socket.recv(1024)
        header_len = len(header)
        xml_bytes = ans[header_len:]
        xml_str = xml_bytes.decode('utf-8')
        # 解析xml
        root_out = ET.fromstring(xml_str)
        items_out = root_out.find('Items')
        out_dict = dict()
        for child in items_out:
            out_dict[str(child.tag)] = child.text

        return out_dict

    def motion_control(self, command: int, value: float):
        '''
        运动控制, 机器人平移或旋转的消息发送频率为3~5Hz

        command: 运动指令

            1: 前进
            2: 后退
            3: 左转
            4: 右转
            6: 速度清零
            11: 左移
            12: 右移
            13: 软急停
            14: 停止踏步
            15: 趴下
            20: 切换步态
        
        value: 运动速度或角度

            前进、后退、左移、右移时 为速度, 单位为m/s, 参数范围为 {-1}and[0, 1.3], -1表示默认值 0.5m/s

            左转、右转时 为角度, 单位为度, 参数范围为 {-1}and[0, 0.45], -1表示默认值 0.35rand/s

            切换步态时 为步态类型, 参数范围为 {0, 1, 2, 4}, 分别对应:
            
                0: 行走
                1: 普通楼梯
                2: 斜坡/防滑
                4: 感知楼梯
        '''
        client_socket = self.client_socket
        root = ET.Element("PatrolDevice")
        ET.SubElement(root, "Type").text = "2"
        ET.SubElement(root, "Command").text = str(command)
        ET.SubElement(root, "Time").text = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime())
        items = ET.SubElement(root, "Items")
        ET.SubElement(items, "Value").text = str(value)
        xml_string = ET.tostring(root, encoding='utf-8')
        xml_string = "<?xml version='1.0' encoding='utf-8'?>\n" + xml_string.decode('utf-8').replace('><','>\n<')
        header = struct.pack("<BBBBHHq", *(0xeb, 0x90, 0xeb, 0x90, len(xml_string), 100, 0))

        all_data = header + xml_string.encode('utf-8')
        client_socket.sendall(all_data)

        ans = b''
        while True:
            part = client_socket.recv(4096)
            ans += part
            if b'</PatrolDevice>' in ans:
                break
        # ans = client_socket.recv(1024)
        header_len = len(header)
        xml_bytes = ans[header_len:]
        xml_str = xml_bytes.decode('utf-8')
        # 解析xml
        root_out = ET.fromstring(xml_str)
        items_out = root_out.find('.//Items')
        out_dict = dict()
        for child in items_out:
            out_dict[str(child.tag)] = child.text

        return out_dict

    def battery_state(self):
        '''
        查询电池状态信息

        响应的字典的key及其value如下 (所有int都以str格式输出)：

        - Voltage: (V)
        - Current: (10mA)
        - RemainingCapacity: (10mAh)
        - NominalCapacity: (10mAh)
        - Cycles: (当前循环次数，单位：次)
        - ProotectedState: '0'(未触发异常保护) / '1' (已触发异常保护)
        - BatteryLevel: (%: 当前电量百分比)
        - BatteryTemperature: (℃: 当前电池温度)
        '''
        client_socket = self.client_socket
        root = ET.Element("PatrolDevice")
        ET.SubElement(root, "Type").text = "2001"
        ET.SubElement(root, "Command").text = "1"
        ET.SubElement(root, "Time").text = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime())
        ET.SubElement(root, "Items")
        xml_string = ET.tostring(root,encoding='utf-8')
        xml_string = "<?xml version='1.0' encoding='utf-8'?>\n" + xml_string.decode('utf-8').replace('><','>\n<')
        header = struct.pack("<BBBBHHq", *(0xeb, 0x90, 0xeb, 0x90, len(xml_string), 100, 0))

        all_data = header + xml_string.encode('utf-8')
        client_socket.sendall(all_data)

        ans = b''
        while True:
            part = client_socket.recv(4096)
            ans += part
            if b'</PatrolDevice>' in ans:
                break
        # ans = client_socket.recv(1024)
        header_len = len(header)
        xml_bytes = ans[header_len:]
        xml_str = xml_bytes.decode('utf-8')
        # 解析xml
        root_out = ET.fromstring(xml_str)
        items_out = root_out.find('Items')
        out_dict = dict()
        for child in items_out:
            out_dict[str(child.tag)] = child.text

        return out_dict

    def current_state(self):
        '''
        get dog current state
        '''
        client_socket = self.client_socket
        root = ET.Element("PatrolDevice")
        ET.SubElement(root, "Type").text = "1002"
        ET.SubElement(root, "Command").text = "1"
        ET.SubElement(root, "Time").text = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime())
        ET.SubElement(root, "Items")
        xml_string = ET.tostring(root,encoding='utf-8')
        xml_string = "<?xml version='1.0' encoding='utf-8'?>\n" + xml_string.decode('utf-8').replace('><','>\n<')
        header = struct.pack("<BBBBHHq", *(0xeb, 0x90, 0xeb, 0x90, len(xml_string), 100, 0))

        all_data = header + xml_string.encode('utf-8')
        client_socket.sendall(all_data)

        ans = b''
        while True:
            part = client_socket.recv(4096)
            ans += part
            if b'</PatrolDevice>' in ans:
                break
        # ans = client_socket.recv(4096)
        header_len = len(header)
        xml_bytes = ans[header_len:]
        xml_str = xml_bytes.decode('utf-8')
        # 解析xml
        root_out = ET.fromstring(xml_str)
        out_dic = dict()
        items_out = root_out.find('Items')
        for child in items_out:
            out_dic[str(child.tag)] = float(child.text)

        return out_dic  

    def obstacle_detection(self):
        client_socket = self.client_socket
        root = ET.Element("PatrolDevice")
        ET.SubElement(root, "Type").text = "2002"
        ET.SubElement(root, "Command").text = "1"
        ET.SubElement(root, "Time").text = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime())
        ET.SubElement(root, "Items")
        xml_string = ET.tostring(root,encoding='utf-8')
        xml_string = "<?xml version='1.0' encoding='utf-8'?>\n" + xml_string.decode('utf-8').replace('><','>\n<')
        header = struct.pack("<BBBBHHq", *(0xeb, 0x90, 0xeb, 0x90, len(xml_string), 100, 0))

        all_data = header + xml_string.encode('utf-8')
        client_socket.sendall(all_data)

        ans = b''
        while True:
            part = client_socket.recv(4096)
            ans += part
            if b'</PatrolDevice>' in ans:
                break
        # ans = client_socket.recv(1024)
        header_len = len(header)
        xml_bytes = ans[header_len:]
        xml_str = xml_bytes.decode('utf-8')
        # 解析xml
        root_out = ET.fromstring(xml_str)
        items_out = root_out.find('Items')
        out_dict = dict()
        for child in items_out:
            out_dict[str(child.tag)] = child.text

        return out_dict

    def localization_init(self, posX: float, posY: float, posZ: float, yaw: float):
        """
        初始化定位

        - posX: X坐标
        - posY: Y坐标
        - posZ: Z坐标
        - yaw: 方向角

        返回值：

        out_dict: 包含定位结果的字典，键为 ErrorCode ：

        - '0': 定位成功
        - '1': 定位失败
        """
        client_socket = self.client_socket
        root = ET.Element("PatrolDevice")
        ET.SubElement(root, "Type").text = "2101"
        ET.SubElement(root, "Command").text = '1'
        ET.SubElement(root, "Time").text = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime())
        items = ET.SubElement(root, "Items")
        ET.SubElement(items, "PosX").text = str(posX)
        ET.SubElement(items, "PosY").text = str(posY)
        ET.SubElement(items, "PosZ").text = str(posZ)
        ET.SubElement(items, "Yaw").text = str(yaw)
        xml_string = ET.tostring(root, encoding='utf-8')
        xml_string = "<?xml version='1.0' encoding='utf-8'?>\n" + xml_string.decode('utf-8').replace('><','>\n<')
        header = struct.pack("<BBBBHHq", *(0xeb, 0x90, 0xeb, 0x90, len(xml_string), 100, 0))

        all_data = header + xml_string.encode('utf-8')
        client_socket.sendall(all_data)

        ans = b''
        while True:
            part = client_socket.recv(4096)
            ans += part
            if b'</PatrolDevice>' in ans:
                break
        # ans = client_socket.recv(1024)
        header_len = len(header)
        xml_bytes = ans[header_len:]
        xml_str = xml_bytes.decode('utf-8')
        # 解析xml
        root_out = ET.fromstring(xml_str)
        items_out = root_out.find('Items')
        out_dict = dict()
        for child in items_out:
            out_dict[str(child.tag)] = child.text

        return out_dict

    def get_motor_temperature(self):
        '''
        获取12个电机的温度
        '''
        client_socket = self.client_socket
        root = ET.Element("PatrolDevice")
        ET.SubElement(root, "Type").text = "1008"
        ET.SubElement(root, "Command").text = "1"
        ET.SubElement(root, "Time").text = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime())
        ET.SubElement(root, "Items")
        xml_string = ET.tostring(root,encoding='utf-8')
        xml_string = "<?xml version='1.0' encoding='utf-8'?>\n" + xml_string.decode('utf-8').replace('><','>\n<')
        header = struct.pack("<BBBBHHq", *(0xeb, 0x90, 0xeb, 0x90, len(xml_string), 100, 0))

        all_data = header + xml_string.encode('utf-8')
        client_socket.sendall(all_data)

        ans = b''
        while True:
            part = client_socket.recv(4096)
            ans += part
            if b'</PatrolDevice>' in ans:
                break
        # ans = client_socket.recv(1024)
        header_len = len(header)
        xml_bytes = ans[header_len:]
        xml_str = xml_bytes.decode('utf-8')
        # 解析xml
        root_out = ET.fromstring(xml_str)
        items_out = root_out.find('Items')
        out_dict = dict()
        for child in items_out:
            out_dict[str(child.tag)] = child.text

        return out_dict

if __name__ == "__main__":
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(("192.168.1.106", 30000))
    point_path = '/home/fusionai/disk/codeup/robot_dog/nav_path/30f233b0_factory807.json'
    dogControl = DogControl(client_socket)
    # ans = dogControl.battery_state()
    # ans = dogControl.nav(pause_t=10, point_path=point_path)
    # ans = dogControl.localization_init()
    # ans = dogControl.current_state()
    ans = dogControl.motion_control(15, -1)
    print(ans)
    client_socket.close()
