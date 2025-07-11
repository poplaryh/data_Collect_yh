#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import JointState
from rm_msgs.msg import MoveJ
from rm_msgs.msg import JointPos
from rm_msgs.msg import JointPosCustom

def joint_state_callback(msg):
    # print('---------start----------------------------------------------------------------------------------')
    # rospy.loginfo("Header: seq=%d, stamp=%.3f, frame_id=%s", msg.header.seq, msg.header.stamp.to_sec(), msg.header.frame_id)
    # rospy.loginfo("Names: %s", ', '.join(msg.name))
    # rospy.loginfo("Positions: %s", ', '.join([str(p) for p in msg.position]))
    # rospy.loginfo("Velocities: %s", ', '.join([str(v) for v in msg.velocity]))
    # rospy.loginfo("Efforts: %s", ', '.join([str(e) for e in msg.effort]))
    # print('---------end----------------------------------------------------------------------------------')

    # cmd = MoveJ()
    # cmd.joint = msg.position
    # cmd.speed = 0.5
    # cmd.trajectory_connect = 0
    # pub01.publish(cmd)
    # rospy.loginfo("Published JointCmd: joint=%s, speed=%f", cmd.joint, cmd.speed)
    # print('-----------------------------------------------------------')

    # cmd = JointPos()
    # cmd.joint = msg.position
    # cmd.expand = 0.0
    # pub01.publish(cmd)
    # rospy.loginfo("Published JointPos: position=%s", cmd.joint)

    cmd = JointPosCustom()
    cmd.joint = msg.position
    cmd.expand = 0.0
    cmd.follow = False  # False-低跟随模式、True-高跟随模式
    cmd.trajectory_mode = 0  # 0-完全透传模式、1-曲线拟合模式、2-滤波模式
    cmd.radio = 10
    pub01.publish(cmd)
    rospy.loginfo("Published JointPosCustom: position=%s, expand=%f, follow=%s, trajectory_mode=%d, radio=%d", 
                  cmd.joint, cmd.expand, cmd.follow, cmd.trajectory_mode, cmd.radio)
    print('-----------------------------------------------------------')


if __name__ == '__main__':
    rospy.init_node('master_joint_listener', anonymous=True)
    pub01 = rospy.Publisher('/slave_arm/rm_driver/MoveJ_Fd_Custom_Cmd', JointPosCustom, queue_size=20)
    # pub01 = rospy.Publisher('/slave_arm/rm_driver/JointPos', JointPos, queue_size=20)
    # pub01 = rospy.Publisher('/slave_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=20)
    rospy.Subscriber('/master_arm/joint_states', JointState, joint_state_callback)
    # pub01 = rospy.Publisher('/slave_arm/rm_driver/JointPos', JointPos, queue_size=10)
    rospy.spin()