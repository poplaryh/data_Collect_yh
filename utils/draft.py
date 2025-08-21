from Robotic_Arm.rm_robot_interface import *
from Robotic_Arm import *
import numpy as np
from scipy.optimize import fsolve
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

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

    def inverse_kinematics(self, pose, current_joint):
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
        # print('+++++++++++++++++++++++++++++++++++++++')
        # a = forward_kinematics(np.radians(current_joint))
        # a = self.homogeneous_matrix_to_pose(a)
        # b = arm.rm_algo_forward_kinematics(current_joint)
        # print('new comparison: ', np.allclose(a, b))
        # print([a[i] - b[i] for i in range(6)])
        # print('+++++++++++++++++++++++++++++++++++++++')

        # 检查是否收敛
        if ier != 1:
            print("逆运动学求解失败")
            return None

        # 将解转换为角度并返回
        return np.degrees(solution).tolist()


def inverse_kinematics2(dh_params, pose, current_joint, current_pose):

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
        for i in range(len(dh_params)):
            a, alpha, d, theta_offset = dh_params[i]
            T = T @ mdh_matrix(a, alpha, d, thetas[i] + theta_offset)
        return T


    def error_func(thetas):
        T_current = forward_kinematics(thetas)
        pos_error = T_current[:3, 3] - np.array(pose[:3])  # 位置误差
        R_current = T_current[:3, :3]
        rotation = R.from_matrix(R_current)
        ex, ey, ez = rotation.as_euler('xyz')
        return np.concatenate([pos_error, [ex - pose[3], ey - pose[4], ez - pose[5]]])

    # 初始猜测（6个关节角度，单位：弧度）
    initial_guess = np.radians(current_joint)

    # 设置关节角度限制（单位：弧度）
    joint_limits = np.radians([
        [-178, 178],  # 关节1
        [-130, 130],  # 关节2
        [-135, 135],  # 关节3
        [-178, 178],  # 关节4
        [-128, 128],  # 关节5
        [-360, 360]   # 关节6
    ])

    # 使用 least_squares 求解逆运动学
    result = least_squares(
        error_func,
        initial_guess,
        method='lm',
        max_nfev=1000,
        xtol=1e-7
    )

    print('------------------------------------------')
    m = forward_kinematics(np.radians(current_joint))
    rotation = R.from_matrix(m[:3, :3])
    ex, ey, ez = rotation.as_euler('xyz')
    # ez = -ez
    print(ex, ey, ez)
    # ex, ey, ez = np.degrees([ex, ey, ez])
    pos_ = m[:3, 3]
    pose_ = [pos_[0], pos_[1], pos_[2], ex, ey, ez]
    print('comparison: lllllll', np.allclose(current_pose, pose_))
    print([current_pose[i] - pose_[i] for i in range(6)])
    print('------------------------------------------')

    if not result.success:
        print("逆运动学求解失败")
        return None

    # 将解转换为角度并返回
    return np.degrees(result.x).tolist()



def abcd(mdh_params, joint_angles_deg, output_units='degrees'):
    """
    基于 MDH 参数的正运动学函数：输入关节角度，输出末端6D位姿（位置+ZYX欧拉角）
    
    参数:
        mdh_params (list): MDH 参数列表（6轴），每个元素为 [a, alpha, d, *]  
                          (a: 连杆长度(m), alpha: 连杆扭角(rad), d: 关节偏移量(m), *: 冗余参数)
        joint_angles_deg (list): 关节角度（度），长度6，顺序为 [θ1, θ2, θ3, θ4, θ5, θ6]
        output_units (str): 输出角度单位（'degrees' 或 'radians'，默认'degrees'）
        
    返回:
        tuple: 6D位姿 (x, y, z, ex, ey, ez)
            - x, y, z (float): 末端位置坐标（米）
            - ex, ey, ez (float): 末端姿态欧拉角（单位由 output_units 决定），旋转顺序为ZYX
    """
    # --------------------------
    # 输入验证与预处理
    # --------------------------
    assert len(mdh_params) == 6, "MDH参数需包含6个连杆的参数（每个元素至少前3个为[a, alpha, d]）"
    assert len(joint_angles_deg) == 6, "关节角度需包含6个轴的角度（度）"
    assert output_units in ['degrees', 'radians'], "输出单位只能是'degrees'或'radians'"
    
    # 将关节角度转换为弧度（内部计算使用弧度）
    joint_angles_rad = np.radians(joint_angles_deg)
    
    # --------------------------
    # 定义单关节D-H变换矩阵（适配MDH参数，忽略第4个冗余参数）
    # --------------------------
    def dh_transform(a, alpha, d, theta):
        """
        计算单个连杆的D-H齐次变换矩阵（MDH参数）
        
        参数:
            a (float): 连杆长度（米）
            alpha (float): 连杆扭角（弧度）
            d (float): 关节偏移量（沿下一个关节轴的偏移，米）
            theta (float): 关节角（弧度）
            
        返回:
            np.ndarray: 4×4齐次变换矩阵
        """
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct,  -st,  0,  a],   # 第1行：x轴变换
            [st*ca,   ct*ca, -sa,  -sa*d],   # 第2行：y轴变换
            [st*sa,     ct*sa,     ca,     ca*d],     # 第3行：z轴变换（d为偏移量）
            [0,     0,      0,      1]      # 第4行：齐次坐标
        ])
    
    # --------------------------
    # 正运动学计算（累积变换矩阵）
    # --------------------------
    T_total = np.eye(4)  # 初始为单位矩阵（基座坐标系）
    
    for i in range(6):  # 遍历6个关节
        # 提取当前连杆的MDH参数（前3个为a, alpha, d，忽略第4个冗余参数）
        a, alpha, d, k = mdh_params[i]  # 关键：仅取前3个参数
        theta = joint_angles_rad[i]       # 当前关节角（弧度）
        T_joint = dh_transform(a, alpha, d, theta + k)  # 当前关节的变换矩阵
        T_total = T_total @ T_joint       # 累积变换（基座→当前连杆末端）
    
    # --------------------------
    # 提取末端位姿（位置+旋转）
    # --------------------------
    # 位置坐标（齐次矩阵最后一列前3个元素）
    x, y, z = T_total[:3, 3]
    
    # 旋转矩阵（齐次矩阵前3×3子矩阵）
    R_current = T_total[:3, :3]
    
    # 旋转矩阵转XYZ顺序欧拉角（弧度）
    rotation = R.from_matrix(R_current)
    ex_rad, ey_rad, ez_rad = rotation.as_euler('xyz')  # ZYX顺序
    
    # 转换输出单位（度或弧度）
    if output_units == 'degrees':
        ex, ey, ez = np.degrees([ex_rad, ey_rad, ez_rad])
    else:
        ex, ey, ez = ex_rad, ey_rad, ez_rad
    
    return [x, y, z, ex, ey, ez]


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

    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

    handle = arm.rm_create_robot_arm("192.168.110.118", 8080)

    sdk = RealManSDK()

    state = arm.rm_get_current_arm_state()
    print(state)

    current_joint = state[1]['joint']
    current_pose = state[1]['pose']

    # T_sdk = sdk.read_mat(arm.rm_algo_pos2matrix(current_pose))
    # T_custom = sdk.pose_to_homogeneous_matrix(current_pose)
    # print('sdk位姿转矩阵', '\n', T_sdk)
    # print('自定义位姿转矩阵', '\n', T_custom)
    # print('两者矩阵相等:', np.allclose(T_sdk, T_custom))
    # print('转换回去的是否相等: ', np.allclose(current_pose, sdk.homogeneous_matrix_to_pose(T_sdk)))
    # print(current_pose, '\n', sdk.homogeneous_matrix_to_pose(T_sdk))

    dx, dy = 0.1, 0.03
    # dxy = np.array([dx, dy, 0.0, 0.0, 0.0, 0.0])
    # print('位姿增量', dxy)
    # dxy = sdk.pose_to_homogeneous_matrix(dxy)
    # print('位姿增量', dxy)

    pose_sdk = arm.rm_algo_cartesian_tool(current_joint, dx, dy, 0)
    # t_custom = T_custom @ dxy
    # print('端部运动自定义计算位姿', '\n', t_custom)
    # pose_custom = sdk.homogeneous_matrix_to_pose(t_custom)
    pose_custom = sdk.move_in_end_coordinate_pose(dx, dy, 0, current_pose)
    pose_custom_joint = sdk.move_in_end_coordinate_joint(dx, dy, 0, current_joint)
    print('端部运动sdk计算结果', '\n', pose_sdk)
    print('端部运动自定义计算结果', '\n', pose_custom)
    print('----------------------')
    print('两者结果相等: ??', np.allclose(pose_sdk, pose_custom))
    print([pose_sdk[i] - pose_custom[i] for i in range(6)])
    print('----------------------')
    print('端部运动自定义计算关节结果', '\n', pose_custom_joint)
    print('两者结果相等:', np.allclose(pose_sdk, pose_custom_joint))
    print([pose_sdk[i] - pose_custom_joint[i] for i in range(6)])
    print('----------------------')

    joint_sdk = arm.rm_algo_inverse_kinematics(rm_inverse_kinematics_params_t(current_joint, pose_sdk, 1))
    joint_custom1 = sdk.inverse_kinematics(pose_custom, current_joint)
    # joint_custom2 = inverse_kinematics2(dh_params, pose_custom_joint, current_joint, current_pose)
    joint_custom2 = sdk.inverse_kinematics(pose_custom_joint, current_joint)
    print('sdk逆解结果', joint_sdk)
    print('自定义逆解结果2', joint_custom2)
    print('两者结果相等2:', np.allclose(joint_sdk[1], joint_custom2))
    print([joint_sdk[1][i] - joint_custom2[i] for i in range(6)])
    print('-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+')
    print('自定义逆解结果1', joint_custom1)
    print('两者结果相等1:', np.allclose(joint_sdk[1], joint_custom1))
    print([joint_sdk[1][i] - joint_custom1[i] for i in range(6)])

    print(arm.rm_algo_get_dh())

    # a = abcd(dh_params, current_joint, output_units='degrees')
    # b = arm.rm_algo_forward_kinematics(current_joint)
    # print('new comparison: ', np.allclose(a, b))
    # print([a[i] - b[i] for i in range(6)])

    arm.rm_delete_robot_arm()

