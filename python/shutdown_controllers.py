import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('arena_notifications')

        self.publisher_ = self.create_publisher(String, '/arena/notifications', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'shutdown'
        self.publisher_.publish(msg)
        self.counter += 1
        if self.counter > 10:
            rclpy.utilities.try_shutdown()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.utilities.try_shutdown()


if __name__ == '__main__':
    main()