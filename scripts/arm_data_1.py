#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import JointState
import os
import json

joint_data_master = []
joint_data_slave = []

def joint_state_callback_master(msg):
    # 存储关节角度和时间戳
    joint_data_master.append({
        "positions": list(msg.position),
        "timestamp": msg.header.stamp.to_sec()
    })

def joint_state_callback_slave(msg):
    # 存储关节角度和时间戳
    joint_data_slave.append({
        "positions": list(msg.position),
        "timestamp": msg.header.stamp.to_sec()
    })

    
def save_joint_data_json(save_path_master, save_path_slave):
    if not joint_data_master or not joint_data_slave:
        rospy.logwarn("No joint data to save.")
        return

    os.makedirs(os.path.dirname(save_path_master), exist_ok=True)
    with open(save_path_master, 'w') as f:
        json.dump(joint_data_master, f, indent=2)

    os.makedirs(os.path.dirname(save_path_slave), exist_ok=True)
    with open(save_path_slave, 'w') as f:
        json.dump(joint_data_slave, f, indent=2)

if __name__ == '__main__':
    rospy.init_node('master_joint_listener', anonymous=True)

    save_path = rospy.get_param('~data_path', '/home/arm_hao/ros_yh/src/datacollect/data1')  # 可通过launch传入
    exp_id = rospy.get_param('~exp_id', 'default_experiment')  # 实验ID
    rospy.loginfo("Experiment ID: %s , save path is %s", exp_id, save_path)

    save_path_master = os.path.join(save_path, str(exp_id), "joint_data_master.json")
    save_path_slave = os.path.join(save_path, str(exp_id), "joint_data_slave.json")

    rospy.Subscriber('/master_arm/joint_states', JointState, joint_state_callback_master)

    rospy.Subscriber('/slave_arm/joint_states', JointState, joint_state_callback_slave)

    def shutdown_hook():
        save_joint_data_json(save_path_master, save_path_slave)
    rospy.on_shutdown(shutdown_hook)

    rospy.spin()