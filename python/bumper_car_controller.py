import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2


class BumperCarController(Node):
    def __init__(self, namespace_name, team_name):
        super().__init__('controller', namespace=namespace_name)

        self.active = False
        self.car_name = namespace_name
        self.team_name = team_name
        self.current_frontal_frame = None

        self.br = CvBridge()

        # Subscribe to arena notifications
        self.notification_subscription = self.create_subscription(
            String,
            '/arena/notifications',
            self.notification_callback,
            10)
        self.notification_subscription
        self.get_logger().info('Subscribed to /arena/notifications')

        # Subscribe to the frontal camera
        self.camera1_subscription = self.create_subscription(
            Image,
            'camera1/image_raw',
            self.camera1_callback,
            qos_profile_sensor_data)
        self.camera1_subscription
        self.get_logger().info('Subscribed to camera1/image_raw')

    # Called when the arena sends a notification to all bumper cars
    def notification_callback(self, msg):
        if msg.data == 'begin_game':
            self.active = True
        elif msg.data == 'end_game':
            self.active = False
            # TODO - stop the cart and stop processing new frames
        elif msg.data == 'shutdown':
            self.get_logger().info('Shutting Down!')
            rclpy.utilities.try_shutdown()

    # Called when the frontal camera receives a frame
    def camera1_callback(self, data):
        self.current_frontal_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_RGB2BGR)
        
        # Process the frame
        if self.active == True:
            self.process()
    
    # Process the current camera frame(s) and take action
    def process(self):
        #self.get_logger().info('Processing Camera Frame')
        return


def main(args=None):
    car_name = sys.argv[1]
    team_name = sys.argv[2]

    rclpy.init(args=args)
    
    bumper_car = BumperCarController(car_name, team_name)

    rclpy.spin(bumper_car)

    print('Destroying Node')
    bumper_car.destroy_node()
    rclpy.utilities.try_shutdown()


if __name__ == '__main__':
    main()
