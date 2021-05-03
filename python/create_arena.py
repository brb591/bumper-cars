import os
import sys
import rclpy

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

def main():
    # Get input arguments from user
    argv = sys.argv[1:]

    # Get the model directory
    model_dir = os.environ['BUMPER_CAR_MODEL_PATH']

    # Start node
    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")
    
    sdf_file_path = os.path.join(model_dir, "arena2", "model.sdf")
    print(sdf_file_path)
    
    request = SpawnEntity.Request()
    request.name = 'create-arena'
    request.xml = open(sdf_file_path, 'r').read()

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Finished Creating Arena.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()