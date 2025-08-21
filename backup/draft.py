import pyrealsense2 as rs
import numpy as np
import time
import cv2

if __name__ == "__main__":
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    pipeline.start()
    frames = pipeline.wait_for_frames()
    try:
        while True:
            start_time = time.time()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # 转换为 numpy 数组
            color_image = np.asanyarray(color_frame.get_data())
                            # 显示检测结果和帧率
            fps = 1.0 / (time.time() - start_time)
            cv2.putText(color_image, f'FPS: {fps:.2f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('YOLO Detection', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                exit(1)
    except KeyboardInterrupt:
        print('lalalalal')
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
