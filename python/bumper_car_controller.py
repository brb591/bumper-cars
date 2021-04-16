import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import random


class BumperCarController(Node):
    def __init__(self, namespace_name, team_name):
        super().__init__('controller', namespace=namespace_name)

        self.active = False
        self.car_name = namespace_name
        self.team_name = team_name
        self.current_frontal_frame = None
        self.x = 0.0
        self.z = 0.0

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

        # Create a publisher for sending movement instructions
        self.movement_publisher = self.create_publisher(Twist, 'cmd_car', 10)

    # Called when the arena sends a notification to all bumper cars
    def notification_callback(self, msg):
        if msg.data == 'begin_game':
            if self.active == False:
                self.get_logger().info('Activating!')
            self.active = True
        elif msg.data == 'end_game':
            if self.active == True:
                self.get_logger().info('Deactivating!')
            self.active = False
            # Stop the bumper car, if it is moving
            self.update_movement(0.0, 0.0)
        elif msg.data == 'shutdown':
            self.get_logger().info('Shutting Down!')
            self.active = False
            # Stop the bumper car, if it is moving
            self.update_movement(0.0, 0.0)
            rclpy.utilities.try_shutdown()

    # Called when the frontal camera receives a frame
    def camera1_callback(self, data):
        self.current_frontal_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_RGB2BGR)
        
        # Process the frame
        if self.active == True:
            self.process()
    
    # Process the current camera frame(s) and take action
    def process(self):
        # This is placeholder code until we have actual processing code
        output = random.randrange(100)
        if output < 5:
            # With 5% probability, we will actually do something

            # Favor forward movement
            speed = (random.randrange(100) - 25) * 0.02

            # Left or Right should be equally distributed
            direction = (random.randrange(100) - 50) * 0.02
            self.get_logger().info('Movement X: %f Z: %f'%(speed, direction))
            self.update_movement(speed, direction)
        # End of placeholder code
            
    
    # Change the movement of the bumper car
    def update_movement(self, forward_speed, drive_angle):
        self.x = forward_speed
        self.z = drive_angle

        msg = Twist()
        msg.linear.x = self.x
        msg.angular.z = self.z
        self.movement_publisher.publish(msg)



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
