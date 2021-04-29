import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
    def __init__(self, car_name, camera_name):
        super().__init__(car_name + '_' + camera_name, namespace=car_name)

        self.car_name = car_name
        self.camera_name = camera_name

        self.subscription = self.create_subscription(Image,
            camera_name + '/image_raw',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription
        self.br = CvBridge()

    def listener_callback(self, data):
        current_frame = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_RGB2BGR)
        cv2.imshow(self.car_name + "_" + self.camera_name, current_frame)
        cv2.waitKey(1)


def main(args=None):
    car_name = sys.argv[1]
    camera_name = sys.argv[2]

    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber(car_name, camera_name)

    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
