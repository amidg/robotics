import rclpy
from rclpy.node import Node

from msg_hands.msg import Hand

import numpy as np
from vision_stereo_hand_tracking.vision import HandTracking

class HandPublisher(Node):
    def __init__(self, hand_tracker):
        super().__init__('HandPublisher')
        self.hand_tracker = hand_tracker
        self.hand_data = np.empty((21, 3))
        self.publisher_ = self.create_publisher(Hand, '/hand', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # read hand data
        self.hand_data = self.hand_tracker.get_data()
        
        # process data as Hand.msg
        msg = Hand()
        msg.side = "right"
        for i in range(21):
            msg.fingers[i].x = self.hand_data[i][0]
            msg.fingers[i].y = self.hand_data[i][1]
            msg.fingers[i].z = self.hand_data[i][2]

        # publish data
        self.publisher_.publish(msg)

def main(args=None):
    # initialize hand tracker
    tracker = HandTracking(3, 1)
    tracker.start()

    # start ros2 node
    rclpy.init(args=args)
    publisher = HandPublisher(tracker)
    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
