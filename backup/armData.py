import numpy as np
import sys
import os
import pandas as pd
from Robotic_Arm import *
from Robotic_Arm.rm_robot_interface import *
import cv2
import json
from typing import Dict, Any




class armDataCollection:
    def __init__(self, path='path/to/json/dictionary'):
        self.ip, self.port = '192.168.110.119', 8080
        self.path = path
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        self.dictionary = self.load_dict_from_json(self.path)

    def collect_data(self, mission_id: int):
        robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = robot.rm_create_robot_arm(self.ip, self.port)

        key_dict_pose ='missionID_pose_' + str(mission_id)
        key_dict_joint = 'missionID_joint_' + str(mission_id)
        self.dictionary.setdefault(key_dict_joint, list())
        self.dictionary.setdefault(key_dict_pose, list())

        cv2.namedWindow("collect", cv2.WINDOW_NORMAL)
        
        try:
            while True:
                keyboard = cv2.waitKey(1) & 0xFF
                if keyboard == ord('s'):  # 按下 's' 键保存机械臂位姿
                    # point_id = input("请输入点位号（如1）：").strip()
                    # if not point_id.isdigit():
                    #     print("点位号必须为数字！")
                    #     continue
                    state = robot.rm_get_current_arm_state()
                    if state[0] != 0:
                        print('failure getting current arm atate')
                        continue
                    print('the current state of arm is as follows: ', '\n', state)
                    self.dictionary[key_dict_joint].append(state[1]['joint'].copy())
                    self.dictionary[key_dict_pose].append(state[1]['pose'].copy())
                    print('arm pose recorded')

                elif keyboard == ord('q'):  # 按下 'q' 键退出程序
                    print("退出程序")
                    break

        finally:            
            self.save_dict_to_json(self.dictionary, self.path)
            print("saved & exit")
            cv2.destroyAllWindows()

    def save_dict_to_json(self, data: Dict[str, Any], file_path: str) -> None:
        """
        Save a dictionary to a JSON file.

        Parameters
        ----------
        data : dict
            The dictionary to save.
        file_path : str or pathlib.Path
            Destination file path (ending in .json).
        """
        os.makedirs(os.path.dirname(file_path) or ".", exist_ok=True)

        with open(file_path, "w", encoding="utf-8") as fp:
            json.dump(data, fp, indent=2, ensure_ascii=False)

    def load_dict_from_json(self, file_path: str) -> Dict[str, Any]:
        """
        Load a dictionary from a JSON file.

        Parameters
        ----------
        file_path : str or pathlib.Path
            Path to the JSON file.

        Returns
        -------
        dict
            Dictionary reconstructed from the JSON file.
        """

        if not os.path.isfile(file_path):
            return dict()                   # brand-new file
        try:
            with open(file_path, "r", encoding="utf-8") as fp:
                return json.load(fp) or dict()  # empty file -> {}
        except json.JSONDecodeError:
            return dict()
        

if __name__ == "__main__":
    sdk = armDataCollection('/home/fusionai/disk/codeup/ros_ws/src/test/data/arm_pose.json')
    sdk.collect_data(3)