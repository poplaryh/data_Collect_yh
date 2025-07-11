#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import JointState
import os
import json

joint_data = []

def joint_state_callback(msg):
    # 存储关节角度和时间戳
    joint_data.append({
        "positions": list(msg.position),
        "timestamp": msg.header.stamp.to_sec()
    })
    # rospy.loginfo("Received JointState: positions=%s, timestamp=%.3f", msg.position, msg.header.stamp.to_sec())

    
def save_joint_data_json(save_path):
    if not joint_data:
        rospy.logwarn("No joint data to save.")
        return
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    with open(save_path, 'w') as f:
        json.dump(joint_data, f, indent=2)
    # rospy.loginfo("Joint data saved to %s", save_path)

if __name__ == '__main__':
    rospy.init_node('master_joint_listener', anonymous=True)

    arm_namespace = rospy.get_param('~arm_namespace', 'master_arm')
    save_path = rospy.get_param('~data_path', '/home/arm_hao/ros_yh/src/datacollect/data1')  # 可通过launch传入
    exp_id = rospy.get_param('~exp_id', 'default_experiment')  # 实验ID
    rospy.loginfo("Experiment ID: %s , save path is %s , namespace is %s", exp_id, save_path, arm_namespace)

    savePath = os.path.join(save_path, str(exp_id), f"joint_data_{arm_namespace}.json")
    rospy.loginfo("Saving joint data to: %s", savePath)

    topic_name = '/' + arm_namespace + '/joint_states'
    rospy.Subscriber(topic_name, JointState, joint_state_callback)

    def shutdown_hook():
        save_joint_data_json(savePath)
    rospy.on_shutdown(shutdown_hook)

    rospy.spin()