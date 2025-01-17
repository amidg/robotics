import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from msg_hands.msg import Hand
from visualization_msgs.msg import Marker, MarkerArray

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
        self.rviz_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # start tracker
        self.hand_tracker.start()

    def publish_raw_data(self):
        # process data as Hand.msg
        msg = Hand()
        msg.side = "right"
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, kpt in enumerate(self.hand_data):
            msg.fingers[i].x = float(kpt[0])/100
            msg.fingers[i].y = float(kpt[1])/100
            msg.fingers[i].z = float(kpt[2])/100
        # publish data
        self.publisher_.publish(msg)

    def publish_rviz_markers(self):
        marker_array = MarkerArray()
        for i, kpt in enumerate(self.hand_data):
            marker = Marker()
            marker.header.frame_id = "map"  # Set the frame of reference
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "hand_tracking"
            marker.id = i  # Unique ID for each marker in the namespace
            marker.type = Marker.SPHERE  # Shape of the marker
            marker.action = Marker.ADD  # Action: add the marker

            # Position the marker
            marker.pose.position.x = float(kpt[0])/100
            marker.pose.position.y = float(kpt[1])/100
            marker.pose.position.z = float(kpt[2])/100

            # Orientation (identity quaternion, no rotation)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Scale the marker (uniform scaling)
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02

            # Color of the marker
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Alpha: 1.0 for fully opaque

            # Add the marker to the array
            marker_array.markers.append(marker)
        # Publish the MarkerArray
        self.rviz_publisher_.publish(marker_array)

    def timer_callback(self):
        # read hand data
        if not self.hand_tracker.get_data(self.shared_mem.name):
            return

        # publish raw data
        self.publish_raw_data()

        # publish RViz data
        self.publish_rviz_markers()
        

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
