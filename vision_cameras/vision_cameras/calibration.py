import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from vision_cameras.stereo_camera import StereoCamera
import numpy as np
import cv2

class Calibration(Node):
    def __init__(self):
        # node settings
        self.node_name = 'calibration'
        self.config_path = get_package_share_directory('vision_cameras')
        super().__init__(self.node_name)

        # cameras
        self.stereo = StereoCamera(3, 1, "stereo_example")

        # variables
        self.cmd_start = False

    def run_calibration(self):
        # start stereo camera
        self.get_logger().info("Starting Calibration process...")
        self.stereo.start()

        # process frames


        # example 2
        frame = np.ndarray(self.stereo.get_resolution())
        while True:
            cv2.imshow("camera 1", self.stereo.get_frame(1))
            cv2.imshow("camera 2", self.stereo.get_frame(2))
            if cv2.waitKey(1) & 0xFF == 27:
                self.stereo.stop()
                break #27 is ESC key.

        cv2.destroyAllWindows()
        self.stereo.join()


def main(args=None):
    # start ros2 node
    rclpy.init(args=args)
    node = Calibration()
    try:
        node.run_calibration()
    except KeyboardInterrupt:
        # Graceful shutdown on Ctrl+C
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
