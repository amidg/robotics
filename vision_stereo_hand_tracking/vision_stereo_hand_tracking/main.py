import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from msg_hands.msg import Hand

import numpy as np
from multiprocessing.shared_memory import SharedMemory
from vision_stereo_hand_tracking.vision import HandTracking

class HandPublisher(Node):
    def __init__(self):
        # node settings
        self.node_name = 'vision_stereo_hand_tracking'
        self.config_path = get_package_share_directory(self.node_name)
        super().__init__(self.node_name)

        # shared memory
        shape = (21, 3)
        dtype=np.float32
        size = np.prod(shape)*np.dtype(dtype).itemsize
        self.shared_mem = SharedMemory(create=True, size=size)
        self.hand_data = np.ndarray(shape, dtype=dtype, buffer=self.shared_mem.buf)
        self.hand_data[:] = 0.0

        # variables
        self.hand_tracker = HandTracking(3, 1, self.config_path)
        self.publisher_ = self.create_publisher(Hand, '/hand', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # start tracker
        self.hand_tracker.start()

    def timer_callback(self):
        # read hand data
        if not self.hand_tracker.get_data(self.shared_mem.name):
            return

        #for kpt in self.hand_data:
        #    print(f"X: {kpt[0]}")
        #    print(f"Y: {kpt[1]}")
        #    print(f"Z: {kpt[2]}")

        # process data as Hand.msg
        msg = Hand()
        msg.side = "right"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, kpt in enumerate(self.hand_data):
            msg.fingers[i].x = float(kpt[0])
            msg.fingers[i].y = float(kpt[1])
            msg.fingers[i].z = float(kpt[2])
            #print(f"X{i}: {self.hand_data[i][0]}")

        # publish data
        self.publisher_.publish(msg)

def main(args=None):
    # start ros2 node
    rclpy.init(args=args)
    publisher = HandPublisher()
    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
