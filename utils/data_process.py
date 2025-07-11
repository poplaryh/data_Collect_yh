import os
import json
import h5py
import numpy as np
from glob import glob
from tqdm import tqdm  # 新增

def load_json_joint_data(json_path):
    """
    加载机械臂关节数据的json文件，返回时间戳数组和关节角度数组
    :param json_path: json文件路径
    :return: (timestamps, joints)
    """
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    timestamps = np.array([item['timestamp'] for item in data])
    joints = np.array([item['positions'] for item in data])
    return timestamps, joints

def align_by_timestamp(target_ts, src_ts, src_data):
    """
    以target_ts为基准，找到src_ts中与每个target_ts最近的时刻，返回对应src_data
    :param target_ts: 目标时间戳（如相机的时间戳，低频）
    :param src_ts: 源数据的时间戳（如机械臂的时间戳，高频）
    :param src_data: 源数据（如机械臂的关节角度）
    :return: 对齐后的数据（与target_ts等长，每个元素为最近邻的src_data）
    """
    idxs = np.searchsorted(src_ts, target_ts, side='left')
    idxs = np.clip(idxs, 0, len(src_ts)-1)
    for i, t in enumerate(target_ts):
        if idxs[i] > 0 and (abs(src_ts[idxs[i]] - t) >= abs(src_ts[idxs[i]-1] - t)):
            idxs[i] = idxs[i]-1
    aligned = src_data[idxs]
    return aligned, idxs

def process_folder(folder_path, output_path):
    """
    处理单个子文件夹：加载两个机械臂的json和相机的hdf5数据，
    以相机时间戳为基准对齐机械臂数据，并保存为新的hdf5文件
    :param folder_path: 子文件夹路径
    :param output_path: 输出hdf5文件路径
    """
    json_files = sorted(glob(os.path.join(folder_path, '*.json')))
    hdf5_file = glob(os.path.join(folder_path, '*.hdf5'))[0]

    # 加载机械臂数据
    arm1_ts, arm1_joints = load_json_joint_data(json_files[0])
    arm2_ts, arm2_joints = load_json_joint_data(json_files[1])

    # 加载相机数据
    with h5py.File(hdf5_file, 'r') as f:
        cam_ts = f['timestamps'][:]   # 相机时间戳
        cam1 = f['camera1_images'][:]       # 相机1图片数据
        cam2 = f['camera2_images'][:]       # 相机2图片数据

    # 对齐机械臂数据到相机时间戳
    arm1_aligned, idx1 = align_by_timestamp(cam_ts, arm1_ts, arm1_joints)
    arm2_aligned, idx2 = align_by_timestamp(cam_ts, arm2_ts, arm2_joints)

    # 保存到新的hdf5
    with h5py.File(output_path, 'w') as f:
        f.create_dataset('timestamps', data=cam_ts[: -1])
        f.create_dataset('actions', data=arm1_aligned[: -1])
        f.create_dataset('qpos', data=arm2_aligned[: -1])
        f.create_dataset('camera1_images', data=cam1[: -1])
        f.create_dataset('camera2_images', data=cam2[: -1])

def main(data_root, output_root):
    """
    遍历所有子文件夹，批量处理并保存对齐后的hdf5文件
    :param data_root: 原始数据根目录
    :param output_root: 输出数据根目录
    """
    os.makedirs(output_root, exist_ok=True)
    subfolders = [os.path.join(data_root, d) for d in os.listdir(data_root) if os.path.isdir(os.path.join(data_root, d))]
    # 用tqdm可视化进度
    for folder in tqdm(subfolders, desc="Processing folders"):
        folder_name = os.path.basename(folder)
        output_path = os.path.join(output_root, f'{folder_name}_aligned.hdf5')
        process_folder(folder, output_path)
        # tqdm会自动显示进度，无需print

if __name__ == '__main__':
    # 输入数据目录和输出数据目录
    input_path = '/home/arm_hao/ros_yh/src/datacollect/data'
    output_path = '/home/arm_hao/ros_yh/src/datacollect/data1'
    main(input_path, output_path)