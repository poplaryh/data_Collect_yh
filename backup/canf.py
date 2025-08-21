import rospy
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

# from realmansdk import RealManSDK
from sensor_msgs.msg import JointState
from rm_msgs.msg import MoveJ
from rm_msgs.msg import JointPos
from rm_msgs.msg import JointPosCustom
from rm_msgs.msg import GetArmState_Command
from rm_msgs.msg import Arm_Current_State
from geometry_msgs.msg import Pose
from rm_msgs.msg import CartePosCustom

def euler2quot(euler):
    '''
    return w, x, y, z
    '''
    r = R.from_euler('xyz', euler, degrees=False)
    q = r.as_quat()
    return [q[-1], q[0], q[1], q[2]]

def get_arm_state(msg):
    # rospy.wait_for_service('/rm_driver/GetArmState_Cmd')
    rospy.loginfo("Joint Positions: %s", ', '.join([str(p) for p in msg.joint]))
    rospy.loginfo("Pose: %s", ', '.join([str(p) for p in msg.Pose]))
    print('---------------------------------------------')
    joint = list(msg.joint)
    pose = list(msg.Pose)
    print(joint, type(joint), type(joint[0]))
    print(pose, type(pose), type(pose[0]))
    print('---------------------------------------------')

if __name__ == "__main__":
    rospy.init_node('lalala')
    pub =  rospy.Publisher('/rm_driver/MoveJ_Cmd', MoveJ, queue_size=20)
    # pub_get_current_state = rospy.Publisher('/rm_driver/GetArmState_Cmd', GetArmState_Command, queue_size=20)
    # sub = rospy.Subscriber('/rm_driver/Arm_Current_State', Arm_Current_State, get_arm_state)
    # # sub = rospy.Subscriber('/joint_states', JointState, queue_size=20)
    pub_canfd = rospy.Publisher('/rm_driver/MoveJ_Fd_Custom_Cmd', JointPosCustom, queue_size=20)
    pub_canfd_pose = rospy.Publisher('/rm_driver/MoveP_Fd_Custom_Cmd', CartePosCustom, queue_size=20)

    time.sleep(0.5)  # Wait for the publisher to be ready
    # txt = MoveJ()
    # # txt.joint = list([0, 0, 0, 1.57, 1.57, 0])
    # # txt.joint = [0.31781142898880443, 0.7219615207287516, 0.465090305777691, 1.4737204221936213, 1.3056730194660882, 0.18129564549507968]
    # txt.joint = list([0, 0, 0, 0, 0, 0])
    # txt.speed = 0.2
    # txt.trajectory_connect = 0

    # cmd = JointPosCustom()
    # cmd.expand = 0.0
    # cmd.follow = False  # False-低跟随模式、True-高跟随模式
    # cmd.trajectory_mode = 0  # 0-完全透传模式、1-曲线拟合模式、2-滤波模式
    # cmd.radio = 10
    # inverse_joint = [0.31781142898880443, 0.7219615207287516, 0.465090305777691, 1.4737204221936213, 1.3056730194660882, 0.18129564549507968]
    # for _ in range(10):
    #     inverse_joint[-1] += 0.05
    #     cmd.joint = inverse_joint
    #     # print(cmd.joint)
    #     pub_canfd.publish(cmd)
    #     time.sleep(0.1)

    # sdk = RealManSDK()
    current_pose = [-0.567467, 0.064813, 0.226653, -1.643, 0.511, 2.505]
    q = euler2quot([current_pose[3], current_pose[4], current_pose[5]])

    cmd = CartePosCustom()
    pose = Pose()
    pose.position.x = current_pose[0]
    pose.position.y = current_pose[1]
    pose.position.z = current_pose[2]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q[0], q[1], q[2], q[3]
    cmd.follow = False  # False-低跟随模式、True-高跟随模式
    cmd.trajectory_mode = 0  # 0-完全透传模式、1-曲线拟合模式、2-滤波模式
    cmd.radio = 10
    for _ in range(5):
        pose.orientation.x -= 0.005
        cmd.Pose = pose
        pub_canfd_pose.publish(cmd)
        time.sleep(0.5)


    # msg_get_current_state = GetArmState_Command()
    # msg_get_current_state.command = 'get_current_arm_state'

    # pub_get_current_state.publish(msg_get_current_state)
    # pub.publish(txt)
    # rospy.loginfo('---------------------------------------')
    # time.sleep(1)



    # for _ in range(2):
    #     time.sleep(1) # Wait for the subscriber to be ready
    #     pub_get_current_state.publish(msg_get_current_state)

    # rospy.spin()