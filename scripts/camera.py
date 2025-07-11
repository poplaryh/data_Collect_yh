import rospy 
import cv2
import numpy as np
import pyrealsense2 as rs
import os
import json
import time
import h5py


def save_camera_data_h5(save_path, images1=None, images2=None, timestamps=None):
    if not (images1 or images2):
        rospy.logwarn("No camera data to save.")
        return
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    with h5py.File(save_path, 'w') as f:
        f.create_dataset('camera1_images', data=np.array(images1))
        f.create_dataset('camera2_images', data=np.array(images2))
        f.create_dataset('timestamps', data=np.array(timestamps))
    rospy.loginfo("Camera data saved to %s", save_path)
    # pipeline1.stop()
    # pipeline2.stop()

if __name__ == '__main__':
    rospy.init_node('camera_node7', anonymous=True)

    save_path = rospy.get_param('~data_path', '/tmp/camera_data.hdf5')
    exp_id = rospy.get_param('~exp_id', 'default_experiment')
    save_h5_path = os.path.join(save_path, str(exp_id), "camera_data.hdf5")
    os.makedirs(os.path.dirname(save_h5_path), exist_ok=True)


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

    images1 = []
    images2 = []
    time_stamp = []


    try:
        while not rospy.is_shutdown():
            frames1 = pipeline1.wait_for_frames()
            frames2 = pipeline2.wait_for_frames()
            if not frames1 or not frames2:
                print("未获取到相机帧数据，尝试重新获取...")
                continue
            ts = time.time()
            color_frame1, color_frame2 = frames1.get_color_frame(), frames2.get_color_frame()
            color_image1, color_image2 = np.asanyarray(color_frame1.get_data()), np.asanyarray(color_frame2.get_data())

            images1.append(color_image1.copy())
            images2.append(color_image2.copy())
            time_stamp.append(ts)

    finally:
        pipeline1.stop()
        pipeline2.stop()
        # save_camera_data_h5(save_h5_path, images1, images2, time_stamp)

    def shutdown_hook():
        save_camera_data_h5(save_h5_path, images1, images2, time_stamp)
    rospy.on_shutdown(shutdown_hook)

    rospy.spin()