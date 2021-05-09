import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import random
import torch
from torchvision import transforms
from PIL import Image


class BumperCarController(Node):
    #
    #   Class initializer
    #
    def __init__(self, namespace_name, team_name):
        super().__init__('controller', namespace=namespace_name)

        self.active = False
        self.car_name = namespace_name
        self.team_name = team_name
        self.current_front_frame = None
        self.current_right_frame = None
        self.current_left_frame = None
        self.x = 0.0
        self.z = 0.0
        self.cuda = torch.cuda.is_available()
        self.model = None
        self.classes_names = [False , True]
        self.transforms = transforms.Compose([transforms.Resize(255),
                                    transforms.CenterCrop(224),
                                    transforms.ToTensor(),
                                    transforms.Normalize([0.485, 0.456, 0.406],
                                    [0.229, 0.224, 0.225])])

        self.br = CvBridge()

        # Subscribe to arena notifications
        self.notification_subscription = self.create_subscription(
            String,
            '/arena/notifications',
            self.notification_callback,
            10)
        self.notification_subscription
        self.get_logger().info('Subscribed to /arena/notifications')

        # Subscribe to the front camera
        self.front_camera_subscription = self.create_subscription(
            ImageMsg,
            'camera_front_sensor/image_raw',
            self.front_camera_callback,
            qos_profile_sensor_data)
        self.front_camera_subscription
        self.get_logger().info('Subscribed to camera_front_sensor/image_raw')

        # Subscribe to the right camera
        self.right_camera_subscription = self.create_subscription(
            ImageMsg,
            'camera_right_sensor/image_raw',
            self.right_camera_callback,
            qos_profile_sensor_data)
        self.right_camera_subscription
        self.get_logger().info('Subscribed to camera_right_sensor/image_raw')

        # Subscribe to the left camera
        self.left_camera_subscription = self.create_subscription(
            ImageMsg,
            'camera_left_sensor/image_raw',
            self.left_camera_callback,
            qos_profile_sensor_data)
        self.left_camera_subscription
        self.get_logger().info('Subscribed to camera_left_sensor/image_raw')

        # Create a publisher for sending movement instructions
        self.movement_publisher = self.create_publisher(Twist, 'cmd_car', 10)

    #
    # Called when the arena sends a notification to all bumper cars
    #
    def notification_callback(self, msg):
        if msg.data == 'prepare_cart':
            if self.model is None:
                self.load_model()
                self.get_logger().info('Loaded model')
        if msg.data == 'begin_game':
            if not self.active:
                self.get_logger().info('Activating!')
                if self.model is None:
                    self.load_model()
                    self.get_logger().info('Loaded model')
            self.active = True
        elif msg.data == 'end_game':
            if self.active:
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

    #
    # Called when the front camera receives a frame
    #
    def front_camera_callback(self, data):
        self.current_front_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_RGB2BGR)
        #self.current_front_frame = Image.fromarray(cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_RGB2BGR))
        # Only if currently active
        if self.active == True:
            self.process()
    
    #
    # Called when the right camera receives a frame
    #
    def right_camera_callback(self, data):
        self.current_right_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_RGB2BGR)
    
    #
    # Called when the left camera receives a frame
    #
    def left_camera_callback(self, data):
        self.current_left_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_RGB2BGR)
    
    #
    # Process the current camera frames and take action
    #
    def process(self):
        if random.randrange(100) < 25:
            # With 25% probability, we will actually do something

            front_ok = self.check_direction('front')
            left_ok = self.check_direction('left')
            right_ok = self.check_direction('right')
            self.get_logger().info("Front: %r  Left: %r Right: %r" % (front_ok, left_ok, right_ok))

            # At the present time, a negative speed means go forward and a positive direction means go right

            # Choose a random forward movement speed
            forward_speed = random.randrange(100) * 0.02 * -1.0

            # The order that we check determines the movement priority
            if front_ok:
                # Move forward
                direction = 0.0
            elif right_ok:
                # Move to the right
                direction = random.randrange(100) * 0.02
            elif left_ok:
                # Move to the left
                direction = random.randrange(100) * 0.02 * -1.0
            else:
                # Nothing is safe, go backwards!
                forward_speed = forward_speed * -1.0
                # Left or Right should be equally distributed
                direction = (random.randrange(100) - 50) * 0.02

            self.get_logger().info('Movement X: %f Z: %f'%(forward_speed, direction))
            self.update_movement(forward_speed, direction)

    #
    #   Apply our model to the requested image and return True
    #       if it is Ok to move in that direction, and False
    #       otherwise.
    #
    def check_direction(self, direction):
        if direction == 'right':
            image = Image.fromarray(self.current_right_frame)
        elif direction == 'left':
            image = Image.fromarray(self.current_left_frame)
        else:
            image = Image.fromarray(self.current_front_frame)
        
        image = self.transforms(image)
        image = image.unsqueeze(0)
        data = image.cuda()
        output = self.model.forward(data)
        _, preds = torch.max(output, 1)
        return self.classes_names[preds.item()]
            
    #
    #   Change the movement of the bumper car by sending
    #       a message via ROS2
    #
    def update_movement(self, forward_speed, drive_angle):
        self.x = forward_speed
        self.z = drive_angle

        msg = Twist()
        msg.linear.x = self.x
        msg.angular.z = self.z
        self.movement_publisher.publish(msg)
    
    #
    #   Load the model
    #
    #
    def load_model(self):
        self.model = torch.load('model/model-ok-notok.pt')
        if self.cuda:
            self.model.cuda()
            self.model.eval()



def main(args=None):
    car_name = sys.argv[1]
    team_name = sys.argv[2]

    # Initialize the ROS2 interface
    rclpy.init(args=args)
    
    # Initialize the bumper car controller
    bumper_car = BumperCarController(car_name, team_name)

    # Allow ROS2 to manage data into and out of this program
    rclpy.spin(bumper_car)

    print('Destroying Node')
    bumper_car.destroy_node()
    rclpy.utilities.try_shutdown()


if __name__ == '__main__':
    main()
