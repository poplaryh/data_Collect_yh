# from Robotic_Arm.rm_robot_interface import *
# from Robotic_Arm import *
import numpy as np
from scipy.optimize import fsolve
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from ultralytics import YOLO
from filterpy.kalman import KalmanFilter
import cv2

import rospy
import time
from sensor_msgs.msg import JointState
from rm_msgs.msg import MoveJ
from rm_msgs.msg import JointPos
from rm_msgs.msg import JointPosCustom
from rm_msgs.msg import GetArmState_Command
from rm_msgs.msg import Arm_Current_State
from rm_msgs.msg import CartePosCustom
from geometry_msgs.msg import Pose

from ctypes import *
import os
import sys
from pynput import keyboard
import threading
import queue  
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
from robot_dog.control.robot_arm.hikang.HCNetSDK import *

from robot_dog.control.robot_arm.hikang.main import devClass


class RealManSDK:
    def __init__(self):
        self.dh_params = [
            [0.0, 0.0, 0.2405, 0.0],     # 关节1: a=0, alpha=0, d=0.1m
            [0, np.pi/2, 0.0, np.pi/2],  # 关节2: a=0.5m, alpha=-90°, d=0
            [0.256, 0.0, 0.0, np.pi/2],  # 关节3: a=0.5m, alpha=0°, d=0
            [0, np.pi/2, 0.21, 0.0],     # 关节4: a=0, alpha=-90°, d=0.3m
            [0.0, -np.pi/2, 0.0, 0],     # 关节5: a=0, alpha=90°, d=0
            [0.0, np.pi/2, 0.1612, 0]    # 关节6: a=0, alpha=-90°, d=0
        ]
        self.joint_limits = np.radians([
            [-178, 178],  # 关节1
            [-130, 130],  # 关节2
            [-135, 135],  # 关节3
            [-178, 178],  # 关节4
            [-128, 128],  # 关节5
            [-360, 360]   # 关节6
        ])
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.end = False
        self.pub_canfd = None
        self.pub_get_state = None
        self.sub = None

    def get_arm_state_joint(self, msg):
        """
        获取机械臂当前状态，包括关节角度和末端执行器位姿
        """
        current_joint = list(msg.joint)
        current_pose = list(msg.Pose)

        if self.dx != 0.0 and self.dy != 0.0:
            dx, dy = self.dx, self.dy
            print('lalalalalal', dx, dy)
            predict_pose = self.move_in_end_coordinate_joint(dx, dy, 0, current_joint)
            inverse_joint = self.inverse_kinematics(predict_pose, current_joint, flag='rad')

            if inverse_joint is None:
                print("逆运动学求解失败，无法发布命令")
                return False
            else:
                print('calculate success!!!!!!!!!!!!!!!!!!!!')

            cmd = JointPosCustom()
            cmd.joint = inverse_joint
            cmd.expand = 0.0
            cmd.follow = False  # False-低跟随模式、True-高跟随模式
            cmd.trajectory_mode = 0  # 0-完全透传模式、1-曲线拟合模式、2-滤波模式
            cmd.radio = 10
            self.pub_canfd.publish(cmd)
            time.sleep(0.1)
            rospy.loginfo("Published JointPosCustom: position=%s, expand=%f, follow=%s, trajectory_mode=%d, radio=%d", 
                    cmd.joint, cmd.expand, cmd.follow, cmd.trajectory_mode, cmd.radio)
        print('-----------------------------------------------------------')
    
    def get_arm_state_pose(self, msg):
        """
        获取机械臂当前状态，包括关节角度和末端执行器位姿
        """
        current_joint = list(msg.joint)
        current_pose = list(msg.Pose)

        if self.dx != 0.0 and self.dy != 0.0:
            dx, dy = self.dx, self.dy
            print('pppoooooooooppp', dx, dy)
            predict_pose = self.move_in_end_coordinate_joint(dx, dy, 0, current_joint)
            q = self.euler2quot([predict_pose[3], predict_pose[4], predict_pose[5]])

            cmd = CartePosCustom()
            pose = Pose()
            pose.position.x = predict_pose[0]
            pose.position.y = predict_pose[1]
            pose.position.z = predict_pose[2]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q[0], q[1], q[2], q[3]
            cmd.Pose = pose
            cmd.follow = False  # False-低跟随模式、True-高跟随模式
            cmd.trajectory_mode = 0  # 0-完全透传模式、1-曲线拟合模式、2-滤波模式
            cmd.radio = 10
            self.pub_canfd_pose.publish(cmd)
            time.sleep(0.1)
            rospy.loginfo("Published JointPosCustom: position=%s, expand=%f, follow=%s, trajectory_mode=%d, radio=%d", 
                    cmd.joint, cmd.expand, cmd.follow, cmd.trajectory_mode, cmd.radio)
        print('-----------------------------------------------------------')

    def homogeneous_matrix_to_pose(self, T: np.ndarray) -> list:
        """
        将4×4齐次变换矩阵转换为6D位姿（x, y, z + ZYX欧拉角）
        
        参数:
            T (np.ndarray): 4×4齐次变换矩阵
            
        返回:
            list: [x, y, z, ex, ey, ez]，其中：
                x, y, z (float): 平移坐标（米）
                ex, ey, ez (float): 欧拉角（弧度），旋转顺序为ZYX（先Z，再Y，最后X）
        """
        # 验证输入矩阵形状
        assert T.shape == (4, 4), "输入必须是4×4齐次变换矩阵"
        assert np.allclose(T[3, :], [0, 0, 0, 1]), "齐次矩阵最后一行为非单位向量"
        
        # 1. 提取平移向量 (x, y, z)
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        
        # 2. 提取旋转矩阵 R (3×3)
        R_mat = T[:3, :3]
        
        # 3. 旋转矩阵转ZYX顺序的欧拉角
        rotation = R.from_matrix(R_mat)
        ex, ey, ez = rotation.as_euler('xyz')  # 输出为弧度
        
        return [x, y, z, ex, ey, ez]

    def pose_to_homogeneous_matrix(self, pose: list) -> np.ndarray:
        """
        将机械臂的6D位姿（x,y,z + ZYX欧拉角）转换为4×4齐次变换矩阵
        
        参数:
            pose (list): 末端执行器的位置坐标（单位：米）和欧拉角（单位：弧度），格式为 [x, y, z, ex, ey, ez]
            x, y, z (float): 末端执行器的位置坐标（单位：米）
            ex, ey, ez (float): 欧拉角（单位：弧度），旋转顺序为ZYX（先Z，再Y，最后X）
            
        返回:
            np.ndarray: 4×4齐次变换矩阵
        """
        x, y, z, ex, ey, ez = pose
        # 1. 计算绕X轴的旋转矩阵（Roll）
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(ex), -np.sin(ex)],
            [0, np.sin(ex), np.cos(ex)]
        ])
        
        # 2. 计算绕Y轴的旋转矩阵（Pitch）
        Ry = np.array([
            [np.cos(ey), 0, np.sin(ey)],
            [0, 1, 0],
            [-np.sin(ey), 0, np.cos(ey)]
        ])
        
        # 3. 计算绕Z轴的旋转矩阵（Yaw）
        Rz = np.array([
            [np.cos(ez), -np.sin(ez), 0],
            [np.sin(ez), np.cos(ez), 0],
            [0, 0, 1]
        ])
        
        # 4. 总旋转矩阵（ZYX顺序：R = Rx * Ry * Rz）
        # 注意：矩阵乘法顺序与旋转顺序相反（先Z后Y再X，对应矩阵右乘）
        # R_total = Rx @ Ry @ Rz  # 等价于 np.matmul(np.matmul(Rx, Ry), Rz)
        R_total = Rz @ Ry @ Rx  # 等价于 np.matmul(np.matmul(Rx, Ry), Rz)
        
        # 5. 构造齐次变换矩阵（旋转+平移）
        T = np.eye(4)  # 初始化4×4单位矩阵
        T[:3, :3] = R_total  # 前3×3为旋转矩阵
        T[:3, 3] = [x, y, z]  # 前3行第4列为平移向量
        
        return T

    def mdh_matrix_main(self, a, alpha, d, theta):
        """计算单个关节的MDH变换矩阵"""
        return np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
            [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
            [0, 0, 0, 1]
        ])

    def forward_kinematics_main(self, thetas):
        """正运动学：根据关节角度计算末端位姿"""
        thetas = np.radians(thetas)
        T = np.eye(4)
        for i in range(len(self.dh_params)):
            a, alpha, d, theta_offset = self.dh_params[i]
            T = T @ self.mdh_matrix_main(a, alpha, d, thetas[i] + theta_offset)
        return T

    def move_in_end_coordinate_pose(self, dx, dy, dz, current_pose):
        '''
        计算末端坐标系下位姿增量在基座标系下的表示
        '''
        T = self.pose_to_homogeneous_matrix(current_pose)
        dxy = np.array([dx, dy, dz, 0, 0, 0])
        # print('位姿增量', dxy)
        dxy = self.pose_to_homogeneous_matrix(dxy)
        # print('位姿增量', dxy)
        t_custom = T @ dxy
        # print('端部运动自定义计算位姿', '\n', t_custom)
        pose_custom = self.homogeneous_matrix_to_pose(t_custom)
        return pose_custom

    def move_in_end_coordinate_joint(self, dx, dy, dz, current_joint):
        '''
        计算末端坐标系下位姿增量在基座标系下的表示
        '''
        T = self.forward_kinematics_main(current_joint)
        # p = arm.rm_algo_forward_kinematics(current_joint)
        # T = self.read_mat(arm.rm_algo_pos2matrix(p))
        dxy = np.array([dx, dy, dz, 0, 0, 0])
        # print('位姿增量', dxy)
        dxy = self.pose_to_homogeneous_matrix(dxy)
        # print('位姿增量', dxy)
        t_custom = T @ dxy
        # print('端部运动自定义计算位姿', '\n', t_custom)
        pose_custom = self.homogeneous_matrix_to_pose(t_custom)
        return pose_custom

    def read_mat(self, mat_realman):
        print("读取矩阵数据")
        print(mat_realman.irow, mat_realman.iline)
        data = [float(mat_realman.data[i]) for i in range(len(mat_realman.data))]
        T = np.reshape(data, (4, 4))
        return T

    def inverse_kinematics(self, pose, current_joint, flag: str):
        """
        输入6轴机械臂的MDH参数和6D位姿，输出6个轴的关节角度
        
        参数:
            dh_params (list): MDH 参数列表，每个元素为 [a, alpha, d, theta_offset]
                            (a: 连杆长度, alpha: 连杆扭角, d: 关节偏移, theta_offset: 关节角度偏移)
            pose (list): 目标末端执行器的6D位姿 [x, y, z, ex, ey, ez]
                        x, y, z (float): 位置坐标（米）
                        ex, ey, ez (float): 欧拉角（弧度），旋转顺序为ZYX（先Z，再Y，最后X）
                        
        返回:
            list: 6个关节角度（单位：度），若求解失败返回 None
        """
        def mdh_matrix(a, alpha, d, theta):
            """计算单个关节的MDH变换矩阵"""
            return np.array([
                [np.cos(theta), -np.sin(theta), 0, a],
                [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
                [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
                [0, 0, 0, 1]
            ])

        def forward_kinematics(thetas):
            """正运动学：根据关节角度计算末端位姿"""
            T = np.eye(4)
            for i in range(len(self.dh_params)):
                a, alpha, d, theta_offset = self.dh_params[i]
                T = T @ mdh_matrix(a, alpha, d, thetas[i] + theta_offset)
            return T

        def error_func(thetas):
            """误差函数：正运动学计算的末端位姿与目标位姿的差异"""
            T_current = forward_kinematics(thetas)
            pos_error = T_current[:3, 3] - np.array(pose[:3])  # 位置误差
            R_current = T_current[:3, :3]
            rotation = R.from_matrix(R_current)
            ex, ey, ez = rotation.as_euler('xyz')
            return np.concatenate([pos_error, [ex - pose[3], ey - pose[4], ez - pose[5]]])

        # 初始猜测（6个关节角度，单位：弧度）
        initial_guess = np.radians(current_joint)  # 将角度转换为弧度

        # 使用 fsolve 求解逆运动学
        solution, _, ier, _ = fsolve(error_func, initial_guess, full_output=True)

        # 检查是否收敛
        if ier != 1:
            print("逆运动学求解失败")
            return None

        # 将解转换为角度并返回
        if flag == 'joint':
            output = np.degrees(solution).tolist()
        else:
            output = solution.tolist()
        return output

    def forward_main(self):
        """
        输入末端沿着工具坐标系的位移，直接进行计算并透传至机械臂
        """
        rospy.init_node('real_man_sdk', anonymous=True)
        self.pub_get_state = rospy.Publisher('/rm_driver/GetArmState_Cmd', GetArmState_Command, queue_size=20)
        self.pub_canfd_joint = rospy.Publisher('/rm_driver/MoveJ_Fd_Custom_Cmd', JointPosCustom, queue_size=20)
        self.pub_canfd_pose = rospy.Publisher('/rm_driver/MoveP_Fd_Custom_Cmd', CartePosCustom, queue_size=20)
        rospy.Subscriber('/rm_driver/Arm_Current_State', Arm_Current_State, self.get_arm_state_joint)
        time.sleep(1)

        ip = b'192.168.1.120'
        port = 8000
        username = b'admin'
        pwd = b'ruining111'
        
        #--------------------可见光配置--------------------
        dev_vis = devClass()
        dev_vis.SetSDKInitCfg()
        dev_vis.hikSDK.NET_DVR_Init()
        dev_vis.GeneralSetting()
        dev_vis.LoginDev(ip, username, pwd)
        dev_vis.startPlay(playTime=0, channel=1)
        
        #--------------------红外配置--------------------
        dev_ir = devClass()
        dev_ir.SetSDKInitCfg()
        dev_ir.hikSDK.NET_DVR_Init()
        dev_ir.GeneralSetting()
        dev_ir.LoginDev(ip, username, pwd)
        dev_ir.startPlay(playTime=0, channel=2)  # 通道2：红外
        
        # 设置默认控制通道
        dev_vis.set_control_channel(1)  # 设置可见光通道为默认控制通道
        
        scale = 1.0
        zooming = {"in": False, "out": False}
        
        #--------------------键盘监听---------------------
        def zoom_in_thread():
            while zooming["in"]:
                dev_vis.zoom_in(duration=0.1)  # 使用 dev_vis 的变焦方法
                time.sleep(0.05)

        def zoom_out_thread():
            while zooming["out"]:
                dev_vis.zoom_out(duration=0.1)  # 使用 dev_vis 的变焦方法
                time.sleep(0.05)

        def on_press(key):
            try:
                if key.char == 'z' and not zooming["in"]:
                    zooming["in"] = True
                    threading.Thread(target=zoom_in_thread, daemon=True).start()
                elif key.char == 'x' and not zooming["out"]:
                    zooming["out"] = True
                    threading.Thread(target=zoom_out_thread, daemon=True).start()
            except AttributeError:
                pass

        def on_release(key):
            try:
                if key.char == 'z':
                    zooming["in"] = False
                elif key.char == 'x':
                    zooming["out"] = False
            except AttributeError:
                pass
        
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        # 获取帧尺寸
        frame_h, frame_w = dev_vis.frame_queue.get().shape[:2]
        aspect_ratio = frame_w / frame_h

        win_name_vis = "see-YOLO"
        # win_name_ir = "redsee"
        cv2.namedWindow(win_name_vis, cv2.WINDOW_NORMAL)
        print('ratio', frame_h, frame_w)

        kf = KalmanFilter(dim_x=4, dim_z=2)
        kf.x = np.array([0, 0, 0, 0])
        kf.F = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]])
        kf.H = np.array([[1,0,0,0], [0,1,0,0]])
        kf.P *= 10.
        kf.R *= 10.
        kf.Q *= 0.01
        kf_initialized = False
        center = [frame_h / 2, frame_w / 2]
        tolerance = 15
        count = 0

        # 设定默认追踪目标
        target = "meter,digital_meter_right,digital_meter_left,digital_meter_mid,red_floatmeter,iron_floatmeter,ph_meter"
        target_names = [name.strip() for name in target.split(",")]

        try:
            model = YOLO('/home/fusionai/disk/codeup/robot_dog/control/robot_arm/cvall/model/detection/train12/weights/best.pt')
            stop_track = False

            while (not self.end and count < 100) or count < 5:
                print(count, ' ++++++++++++')
                start_time = time.time()
                detc = False
                try:
                    # 清空旧帧，只要最新的
                    while dev_vis.frame_queue.qsize() > 1:
                        dev_vis.frame_queue.get_nowait()
                    color_image = dev_vis.frame_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                if count < 10:
                    count += 1
                    continue

                r = next(model(color_image, stream=True))

                annotated_frame = r.plot()
                
                for box in r.boxes:
                    if box is not None:
                        cls = box.cls
                        conf = float(box.conf)
                        if cls is not None:
                            label = model.names[int(cls)]
                            if label in str("meter,digital_meter_right,digital_meter_left,digital_meter_mid,red_floatmeter,iron_floatmeter,ph_meter") and conf > 0.65:
                                detc = True
                                x_min, y_min, x_max, y_max = box.xyxy.cpu().numpy().flatten().astype(int)
                                center_x = (x_min + x_max) / 2
                                center_y = (y_min + y_max) / 2

                                # 卡尔曼滤波
                                if not kf_initialized:
                                    kf.x[:2] = [center_x, center_y]
                                    kf_initialized = True
                                else:
                                    kf.predict()
                                    kf.update([center_x, center_y])
                                filtered_cx, filtered_cy = kf.x[0], kf.x[1]

                                # 偏离中心则控制机械臂
                                d_x, d_y = filtered_cx - center[0], filtered_cy - center[1]
                                if abs(d_x) > tolerance or abs(d_y) > tolerance:
                                    dt = 0.0001
                                    # 控制机械臂
                                    if abs(d_x) > tolerance and d_x / abs(d_x) > 0:
                                        dy = -dt * abs(d_x)
                                    elif abs(d_x) > tolerance and d_x / abs(d_x) < 0:
                                        dy = dt * abs(d_x)
                                    else:
                                        dy = 0
                                    
                                    if abs(d_y) > tolerance and d_y / abs(d_y) > 0: 
                                        dx = dt * abs(d_y)
                                    elif abs(d_y) > tolerance and d_y / abs(d_y) < 0:
                                        dx = -dt * abs(d_y)
                                    else:
                                        dx = 0

                                    print(d_x, d_y, dx, dy, d_x / abs(d_x), d_y / abs(d_y))

                                    try:
                                        self.dx, self.dy, self.dz = dx, dy, 0
                                        cmd_txt = GetArmState_Command()
                                        cmd_txt.command = 'get_current_arm_state'
                                        self.pub_get_state.publish(cmd_txt)
                                        print('fa song zhi ling yi huo de dang qian wei zi xin xi')
                                        time.sleep(0.5)

                                    except Exception as e:
                                        print("机械臂控制异常：", e)
                                else:
                                    print("目标已到达容忍范围内，停止追踪。", abs(filtered_cx - center[0]), filtered_cy - center[1], 'dx', 'dy')
                                    self.end = True
                                # break  # 检测到一个目标就不再处理其他目标

                if not detc:
                    count += 1
                # 显示检测结果和帧率
                fps = 1.0 / (time.time() - start_time)
                cv2.putText(annotated_frame, f'FPS: {fps:.2f}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow(win_name_vis, annotated_frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    exit(1)
        except KeyboardInterrupt:
            print('Interrupt by control C')
        finally:
            dev_vis.stopPlay()
            dev_vis.LogoutDev()
            dev_vis.hikSDK.NET_DVR_Cleanup()
            dev_ir.stopPlay()
            dev_ir.LogoutDev()
            dev_ir.hikSDK.NET_DVR_Cleanup()
            cv2.destroyAllWindows()
            return self.end

    def ans_return(self):
        return self.end
    
    def euler2quot(self, euler):
        '''
        return w, x, y, z
        '''
        r = R.from_euler('xyz', euler, degrees=False)
        q = r.as_quat()
        return [q[-1], q[0], q[1], q[2]]


if __name__ == "__main__":
    # 示例D-H参数（6轴机械臂，假设结构为RRRRRR）
    # 格式：[a, alpha, d, theta_offset]（theta_offset为关节角度偏移，通常为0）
    # dh_params = [
    #     [0.0, 0.0, 0.172, 0.0],     # 关节1: a=0, alpha=0, d=0.1m
    #     [-0.086, -np.pi/2, 0.0, -np.pi/2], # 关节2: a=0.5m, alpha=-90°, d=0
    #     [0.38, 0.0, 0.0, np.pi/2],      # 关节3: a=0.5m, alpha=0°, d=0
    #     [0.069, np.pi/2, 0.405, 0.0], # 关节4: a=0, alpha=-90°, d=0.3m
    #     [0.0, -np.pi/2, 0.0, np.pi],  # 关节5: a=0, alpha=90°, d=0
    #     [0.0, -np.pi/2, 0.1151, np.pi]  # 关节6: a=0, alpha=-90°, d=0
    # ]

    dh_params = [
        [0.0, 0.0, 0.2405, 0.0],     # 关节1: a=0, alpha=0, d=0.1m
        [0, np.pi/2, 0.0, np.pi/2], # 关节2: a=0.5m, alpha=-90°, d=0
        [0.256, 0.0, 0.0, np.pi/2],      # 关节3: a=0.5m, alpha=0°, d=0
        [0, np.pi/2, 0.21, 0.0], # 关节4: a=0, alpha=-90°, d=0.3m
        [0.0, -np.pi/2, 0.0, 0],  # 关节5: a=0, alpha=90°, d=0
        [0.0, np.pi/2, 0.1612, 0]  # 关节6: a=0, alpha=-90°, d=0
    ]

    # 关节角度限制（角度制，示例范围）
    # joint_limits_deg = [
    #     [-178, 178],   # 关节1: ±180°
    #     [-178, 178],     # 关节2: ±90°
    #     [-178, 145],   # 关节3: ±180°
    #     [-178, 178],     # 关节4: ±90°
    #     [-178, 178],   # 关节5: ±180°
    #     [-360, 360]    # 关节6: ±180°
    # ]
    joint_limits = np.radians([
        [-178, 178],  # 关节1
        [-130, 130],  # 关节2
        [-135, 135],  # 关节3
        [-178, 178],  # 关节4
        [-128, 128],  # 关节5
        [-360, 360]   # 关节6
    ])

    sdk = RealManSDK()

    sdk.forward_main()


