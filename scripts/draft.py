import rospy 
import cv2
import numpy as np
import pyrealsense2 as rs
import os
import json
import time
import h5py

if __name__ == '__main__':
    rospy.init_node('camera_node7', anonymous=True)

    # 确定图像的输入分辨率与帧率
    resolution_width = 640  # pixels
    resolution_height = 480  # pixels
    frame_rate = 15  # fps

    # 注册数据流，并对其图像
    rs_config = rs.config()
    rs_config.enable_stream(rs.stream.color, resolution_width, resolution_height, rs.format.bgr8, frame_rate)
    # check相机是不是进来了
    connect_device = []
    for d in rs.context().devices:
        print('Found device: ',
              d.get_info(rs.camera_info.name), ' ',
              d.get_info(rs.camera_info.serial_number))
        if d.get_info(rs.camera_info.name).lower() != 'platform camera':
            connect_device.append(d.get_info(rs.camera_info.serial_number))


    if len(connect_device) < 2:
        print('Registrition needs two camera connected.But got one.')
        exit()

    # 确认相机并获取相机的内部参数
    pipeline1 = rs.pipeline()
    rs_config.enable_device(connect_device[0])
    # pipeline_profile1 = pipeline1.start(rs_config)
    pipeline1.start(rs_config)

    pipeline2 = rs.pipeline()
    rs_config.enable_device(connect_device[1])
    # pipeline_profile2 = pipeline2.start(rs_config)
    pipeline2.start(rs_config)

    try:

        while not rospy.is_shutdown():

            # 等待数据进来
            print("Waiting for frames...")
            frames1 = pipeline1.wait_for_frames()
            frames2 = pipeline2.wait_for_frames()

            # 将对其的RGB—D图取出来
            color_frame1 = frames1.get_color_frame()
            color_frame2 = frames2.get_color_frame()

            color_image1 = np.asanyarray(color_frame1.get_data())

            color_image2 = np.asanyarray(color_frame2.get_data())
            
            cv2.imshow('RealSense1', color_image1)
            cv2.imshow('RealSense2', color_image2)

            key = cv2.waitKey(1)

            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline1.stop()
        pipeline2.stop()



    rospy.spin()