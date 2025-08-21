from SQLite import RoboticArmDB
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import numpy as np
import cv2
import time
from Robotic_Arm.rm_robot_interface import *
import pyrealsense2 as rs
from ultralytics import YOLO
from filterpy.kalman import KalmanFilter

from cvall.cvmain import MeterReadingManager
import struct
import socket

import rospy
from sensor_msgs.msg import JointState
from rm_msgs.msg import MoveJ
from rm_msgs.msg import JointPos
from rm_msgs.msg import JointPosCustom
from rm_msgs.msg import GetArmState_Command

class real_robot():
    def __init__(self, 
                 ip='192.168.1.18', 
                 port=8080, 
                 db_path='/home/fusionai/disk/codeup/robot_dog/control/robot_arm/realman/arm_data.db', 
                 model_path='/home/fusionai/disk/codeup/robot_dog/control/robot_arm/cvall/model/detection/train12/weights/best.pt',
                 hand_to_camera=np.array([[-0.00310044, 0.99999236, 0.00238261, 0],  
                                          [-0.99997374, -0.00311597, 0.00654238, 0],
                                          [0.00654975, -0.00236226, 0.99997576, 0],
                                          [0, 0, 0, 1]]),
                 target_label="meter,digital_meter_right,digital_meter_left,digital_meter_mid,red_floatmeter,iron_floatmeter,ph_meter",
                 tolerance=15,
                 w=640,
                 h=480,
                 server_socket=None):  
        """
        初始化机械臂连接
        :param ip: 机械臂IP地址
        :param port: 机械臂透传端口
        :param db_path: 数据库路径，如果为None则使用默认路径
        """
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.robot.rm_create_robot_arm(ip, port)
        
        self.target_names = [name.strip() for name in target_label.split(",")]
        
        if self.handle:
            print(f"机械臂连接成功,ID: {self.handle.id}")
            self.connected = True
            # print(self.robot.rm_get_current_arm_state())
        else:
            print(f"机械臂连接失败,错误码: {self.handle}")
            self.connected = False
            exit()
        self.db_path = db_path
        self.model = YOLO(model_path) if model_path else None
        self.hand_to_camera = hand_to_camera
        self.target_label = target_label
        self.tolerance = tolerance
        self.w = w
        self.h = h
        self.server_socket = server_socket
        self.client_socket = None
        self.client_addr = None
        
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.array([0, 0, 0, 0])
        self.kf.F = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]])
        self.kf.H = np.array([[1,0,0,0], [0,1,0,0]])
        self.kf.P *= 10.
        self.kf.R *= 10.
        self.kf.Q *= 0.01
        self.kf_initialized = False

        s = self.robot.rm_get_current_arm_state()
        print(s)
        p = s[1]['pose']
        self.center = [w/2, h/2]  # 初始化中心点，z为当前机械臂z   

    def connect_client(self, timeout=30):
        """
        等待客户端连接
        :param timeout: 连接超时时间（秒）
        :return: bool, 连接是否成功
        """
        if self.server_socket is None:
            print("服务器socket未初始化")
            return False
            
        try:
            print("等待客户端连接...")
            self.server_socket.settimeout(timeout)
            self.client_socket, self.client_addr = self.server_socket.accept()
            self.server_socket.settimeout(None)  # 恢复阻塞模式
            print(f"客户端 {self.client_addr} 已连接")
            return True
        except socket.timeout:
            print(f"等待客户端连接超时（{timeout}秒）")
            return False
        except Exception as e:
            print(f"客户端连接失败: {e}")
            return False

        
        
    def from_datalist_get_joint_matrix(self, main_id):
        """
        用于从数据库读取指定主编号的动作序列关节数据
        获取指定主编号的所有动作序列关节数据列表
        :param main_id: 主编号（int或str）
        :return: joint_matrix（关节数据列表）
        """
        main_id = str(main_id)
        all_names = self.db.get_all_names()
        target_names = [name for name in all_names if name.startswith(f"{main_id}.")]
        joint_matrix = []
        for name in target_names:
            results = self.db.get_data_for_function(name=name)
            if results:
                joint_matrix.extend([row['joint'] for row in results])
        return joint_matrix
    
    def move_to_init_pose(self, mode: int):
        """
        初始化位姿
        :param mode: 0 - 初始位姿, 1 - 物体追踪初始位姿, 2 - 巡检位姿
        """
        if mode == 0:
            self.robot.rm_movej([0, 0, 0, 0, 0, 0], 20, 0, 0, 1)
            print("机械臂已移动到初始位姿")
        elif mode == 1:
            self.robot.rm_movej([-2.288, -8.908, 97.424, -0.788, 88.872, -0.019], 20, 0, 0, 1)
            print("机械臂已移动到物体追踪的初始位姿")
        elif mode == 2:
            self.robot.rm_movej([10, -10, 90, 0, 90, 0], 20, 0, 0, 1)
            print("机械臂已移动到巡检位姿")
        elif mode == 3:
            self.robot.rm_movej([-5.004,-102.260002,142.576996,-175.863007,-108.679001,176.755997], 20, 0, 0, 1)
            print("机械臂已移动到机器狗巡检位姿")
        else:
            print("未知模式")

    def get_current_pose(self):
        """
        获取当前机械臂位姿
        """
        state = self.robot.rm_get_current_arm_state()
        pose = state[1]['pose']
        print(f"当前机械臂位姿: {pose}")
        return pose

    def get_current_joint(self):
        """
        获取当前机械臂关节角度
        """
        state = self.robot.rm_get_current_arm_state()
        joint = state[1]['joint']
        print(f"当前机械臂关节角度: {joint}")
        return joint

    def step_move(self, direction: str, distance: float, count: int = 1):
        """
        步进操作示例 笛卡尔坐标系下的步进移动
        :param direction: 步进方向 "x", "y", "z"
        :param distance: 步进移动距离
        :param count: 步进次数
        """
        step = distance / count
        i = 0
        state = self.robot.rm_get_current_arm_state()
        print(f"当前机械臂状态: {state}")
        pose = state[1]['pose']
        x = pose[0]
        y = pose[1]
        z = pose[2]
        if direction not in ["x", "y", "z"]:
            print("方向错误，请输入 'x', 'y' 或 'z'")
            return
        else:
            if(direction == "x"):
                for i in range(count):
                    print(self.robot.rm_movej_p([x+step, y, z, pose[3], pose[4], pose[5]], 20, 0, 0, 1))
                    x += step
            elif(direction == "y"):
                for i in range(count):
                    print(self.robot.rm_movej_p([x, y+step, z, pose[3], pose[4], pose[5]], 20, 0, 0, 1))
                    y += step
            elif(direction == "z"):
                for i in range(count):
                    print(self.robot.rm_movej_p([x, y, z+step, pose[3], pose[4], pose[5]], 20, 0, 0, 1))
                    z += step
                    
    def button(self):
        """
        按钮操作示例
        """
        self.robot.rm_movej([0, -44, 133, 0, 0, 0], 20, 0, 0, 1)
        self.step_move('x',-0.2,5)
        
    def delete_robot(self):
        """
        删除机械臂
        """
        self.robot.rm_delete_robot_arm()
        print("机械臂已删除")

    def move_by_joint(self, joint_angles, speed=20, acc=0, dec=0, wait=1):
        """
        通过关节角度移动机械臂
        :param joint_angles: 关节角度列表
        :param speed: 移动速度
        :param acc: 加速度
        :param dec: 减速度
        :param wait: 是否阻塞
        """
        if not self.connected:
            print("机械臂未连接")
            return
        a = self.robot.rm_movej(joint_angles, speed, acc, dec, wait)
        print('result of move by joint: ', a)
    
    def move_by_pose(self, pose, speed=20, acc=0, dec=0, wait=1):
        """
        通过位姿移动机械臂
        :param pose: 位姿列表 [x, y, z, roll, pitch, yaw]
        :param speed: 移动速度
        :param acc: 加速度
        :param dec: 减速度
        :param wait: 是否阻塞
        """
        if not self.connected:
            print("机械臂未连接")
            return
        self.robot.rm_movej_p(pose, speed, acc, dec, wait)
        
    def TCPServer(self, target_label=None):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口重用
        server_socket.bind(('0.0.0.0', 30000))  # 绑定所有接口的30000端口
        server_socket.listen(2)  # 监听1个连接请求
        print("服务器启动，等待客户端连接...")

        try:
            # 等待客户端连接（阻塞直到有连接）
            client_socket, client_addr = server_socket.accept()
            print(f"客户端 {client_addr} 已连接")
            flag = False

            while not flag:
                # 读取长度前缀（4字节）
                # print("等待接收数据...")
                print('++++++++++++++++++++++++++++++++++++++')
                length_data = client_socket.recv(4)
                if not length_data:
                    print("客户端断开连接")
                    break
                total_len = struct.unpack('I', length_data)[0]  # 解包为大端无符号整数

                # 读取完整数据包（总长度）
                data = b''
                while len(data) < total_len:
                    recv_data = client_socket.recv(total_len - len(data))
                    if not recv_data:
                        print("数据接收中断")
                        break
                    data += recv_data

                # 解析元数据和图像数据
                # 格式：[4字节总长度][4字节宽度][4字节高度][图像数据]
                if len(data) < 8:  # 至少需要8字节（4+4）
                    print("数据包不完整")
                    continue
                
                width = struct.unpack('i', data[0:4])[0]  # 宽度（4字节）
                height = struct.unpack('i', data[4:8])[0]  # 高度（4字节）
                img_bytes = data[8:]  # 图像数据（剩余部分）

                # 重构图像（注意发送端格式是否为RGB）
                # 若发送端是RGB，接收端需转换为BGR（OpenCV默认显示BGR）
                img = np.frombuffer(img_bytes, dtype=np.uint8).reshape(height, width, 3)
                if img.size == 0:
                    print("图像数据为空")
                    continue
                
                # 显示图像（若发送端是RGB，需转换颜色空间）
                # cv2.imshow("Received Image", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break
                
                # try:
                #     flag = self.track(img, target_label=target_label)
                #     # time.sleep(2)
                # except Exception as e:
                #     print(f"追踪错误: {str(e)}")

            print("追踪完成，结束循环。")

        except Exception as e:
            print(f"运行错误: {str(e)}")
        finally:
            client_socket.close()
            server_socket.close()
            cv2.destroyAllWindows()

    def test1(self):
        # 确保客户端已连接
        if self.client_socket is None:
            print("客户端未连接，尝试建立连接...")
            if not self.connect_client():
                print("无法建立客户端连接")
                return False, None
                
        try:
            flag = False
            dect = False
            count = 0

            while (count < 50 and not flag) or count < 10:  
                # 读取长度前缀（4字节）
                # print("等待接收数据...")
                # print('++++++++++++++++++++++++++++++++++++++')
                # print("count:",count, 'flag', flag)
                length_data = self.client_socket.recv(4)
                if not length_data:
                    print("客户端断开连接")
                    break
                total_len = struct.unpack('I', length_data)[0]  # 解包为大端无符号整数

                # 读取完整数据包（总长度）
                data = b''
                while len(data) < total_len:
                    recv_data = self.client_socket.recv(total_len - len(data))
                    if not recv_data:
                        print("数据接收中断")
                        break
                    data += recv_data

                # 解析元数据和图像数据
                # 格式：[4字节总长度][4字节宽度][4字节高度][图像数据]
                if len(data) < 8:  # 至少需要8字节（4+4）
                    print("数据包不完整")
                    continue
                
                width = struct.unpack('i', data[0:4])[0]  # 宽度（4字节）
                height = struct.unpack('i', data[4:8])[0]  # 高度（4字节）
                img_bytes = data[8:]  # 图像数据（剩余部分）

                # 重构图像（注意发送端格式是否为RGB）
                # 若发送端是RGB，接收端需转换为BGR（OpenCV默认显示BGR）
                img = np.frombuffer(img_bytes, dtype=np.uint8).reshape(height, width, 3)
                if img.size == 0:
                    print("图像数据为空")
                    continue
                if count<5:
                    count +=1
                    continue
                # cv2.imwrite(f'C:/Users/Lenovo/Desktop/abcd/robot_dog/control/robot_arm/realman/captured_image{count}.jpg', img)
                # flag , dect = self.track1(img,count,'C:/Users/Lenovo/Desktop/abcd/robot_dog/control/robot_arm/realman/captured_image{count}.jpg') 
                flag , dect = self.track(img) 
                if not dect or count < 10:
                    count += 1 
        except Exception as e:
            print(f"运行错误: {str(e)}")
        finally:
            # client_socket.close()
            # server_socket.close()
            cv2.destroyAllWindows()
            if flag == True:
                return flag,img 
            else:
                return flag,None
    def track(self, color_image):  
        # print('---------------------------------------------------')
        try:
            stop_tracking = False
            dect = False
            start_time = time.time()
            r = next(self.model(color_image, stream=True))

            annotated_frame = r.plot()
            
            for box in r.boxes:
                if box is not None:
                    cls = box.cls
                    conf = float(box.conf)
                    if cls is not None:
                        label = self.model.names[int(cls)]
                        if label in self.target_names and conf > 0.65:
                            dect = True
                            x_min, y_min, x_max, y_max = box.xyxy.cpu().numpy().flatten().astype(int)
                            center_x = (x_min + x_max) / 2
                            center_y = (y_min + y_max) / 2

                            # 卡尔曼滤波
                            if not self.kf_initialized:
                                self.kf.x[:2] = [center_x, center_y]
                                self.kf_initialized = True
                            else:
                                self.kf.predict()
                                self.kf.update([center_x, center_y])
                            filtered_cx, filtered_cy = self.kf.x[0], self.kf.x[1]

                            # 偏离中心则控制机械臂
                            d_x, d_y = filtered_cx - self.center[0], filtered_cy - self.center[1]
                            if abs(d_x) > self.tolerance or abs(d_y) > self.tolerance:
                                dt = 0.0001
                                # 控制机械臂
                                if abs(d_x) > self.tolerance and d_x / abs(d_x) > 0:
                                    dy = -dt * abs(d_x)
                                elif abs(d_x) > self.tolerance and d_x / abs(d_x) < 0:
                                    dy = dt * abs(d_x)
                                else:
                                    dy = 0
                                
                                if abs(d_y) > self.tolerance and d_y / abs(d_y) > 0: 
                                    dx = dt * abs(d_y)
                                elif abs(d_y) > self.tolerance and d_y / abs(d_y) < 0:
                                    dx = -dt * abs(d_y)
                                else:
                                    dx = 0

                                print(d_x, d_y, dx, dy, d_x / abs(d_x), d_y / abs(d_y))

                                try:
                                    state = self.robot.rm_get_current_arm_state()
                                    current_joint = state[1]['joint']
                                    aa = self.robot.rm_algo_cartesian_tool(current_joint, dx, dy, 0)
                                    params = rm_inverse_kinematics_params_t(current_joint, aa, 1)
                                    bb = self.robot.rm_algo_inverse_kinematics(params)
                                    if bb[0] == 0:
                                        print('机械臂位移返回： ', self.robot.rm_movej_canfd(bb[1], False))
                                    else:
                                        print('机械臂逆解失败，错误码：', bb[0])
                                        dect = False
                                except Exception as e:
                                    print("机械臂控制异常：", e)
                            else:
                                print("目标已到达容忍范围内，停止追踪。", abs(filtered_cx - self.center[0]), filtered_cy - self.center[1], 'dx', 'dy')
                                # if count > 5 :
                                #     stop_tracking = True
                                # else:
                                stop_tracking = True
                            # break  # 检测到一个目标就不再处理其他目标

            # 显示检测结果和帧率
            fps = 1.0 / (time.time() - start_time)
            cv2.putText(annotated_frame, f'FPS: {fps:.2f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('YOLO Detection', annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit(1)
        except Exception as e:
            print(f"运行错误: {str(e)}")
        
        return stop_tracking, dect
          
    def robot_find_task(self, main_id=None, db_path=None):
        """
        机器人任务处理逻辑
        
        返回flag：
        
        - true 表示任务成功，false表示任务失败
        ...
        """
        self.db = RoboticArmDB(db_path) if db_path else RoboticArmDB()
        
        joint_list = self.from_datalist_get_joint_matrix(main_id)
        
        print(f"main_id={main_id} 共{len(joint_list)}个位姿")
        try:
            for idx, joint in enumerate(joint_list):
                print(f"切换到第{idx+1}个位姿: {joint}")
                self.move_by_joint(joint_angles = joint)
                time.sleep(2)  # 等机械臂稳定
                flag, img = self.test1()
                if flag:
                    # print(type(img), img)
                    manage = MeterReadingManager()
                    manage.read(img)
                    time.sleep(3)  # 等待读取完成
                    # self.move_to_init_pose(mode=3)
                    # time.sleep(5)
                    print("任务成功")        
                    break         
                else:
                    print("任务失败")
        finally:
            self.move_to_init_pose(mode=3)
            time.sleep(2)
        return flag

if __name__ == "__main__":
#     # 先创建服务器socket
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许端口重用
#     server_socket.bind(('0.0.0.0', 30000))  # 绑定所有接口的30000端口
#     server_socket.listen(2)  # 监听1个连接请求
#     print("服务器启动，等待客户端连接...")
#     
#     # 创建机械臂对象（不会立即连接socket客户端）
    robot = real_robot(w=1920, h=1080,server_socket=server_socket)
    robot.move_to_init_pose(0)
#     
#     # 先测试机械臂连接
#     if robot.connected:
#         print("机械臂连接成功，开始等待socket客户端...")
#         # 在需要时再连接socket客户端
#         robot.robot_find_task(1)  #输入id
#     else:
#         print("机械臂连接失败")
#     
#     robot.delete_robot()
#     server_socket.close()
