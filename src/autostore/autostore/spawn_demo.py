import os # Operating system library
import sys # Python runtime environment library
import rclpy # ROS Client Library for Python

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_msgs.msg import Int32MultiArray

# Package Management Library
from ament_index_python.packages import get_package_share_directory 

# Gazebo's service to spawn a robot
from gazebo_msgs.srv import SpawnEntity

class RobotControl(Node):
	def __init__(self, robot_name):
		super().__init__(robot_name + '_control')
		namespace = '/'+robot_name
		self.planar_publisher = self.create_publisher(Twist, namespace+'/cmd_vel', 10)
		self.odom_subscriber = self.create_subscription(Odometry, namespace+'/odom', self.get_cur_pos, 10)
		self.arr_flag_publisher = self.create_publisher(Empty, namespace+'/arrive', 10)
		self.cmd_subscriber = self.create_subscription(Int32MultiArray, namespace+'/cmd_moveto', self.get_tar_pos, 10)
		self.tar_pos = [1.0, 1.0, 0.1, 0.0, 0.0, 0.0]
		self.cur_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.timer = self.create_timer(0.05, self.timer_callback)

	def get_cur_pos(self, msg):
		self.cur_pos[0] = msg.pose.pose.position.x
		self.cur_pos[1] = msg.pose.pose.position.y
		self.cur_pos[2] = msg.pose.pose.position.z
		self.cur_pos[3] = msg.pose.pose.orientation.x
		self.cur_pos[4] = msg.pose.pose.orientation.y
		self.cur_pos[5] = msg.pose.pose.orientation.z

	def get_tar_pos(self, msg):
		self.tar_pos[0] = msg.data[0]
		self.tar_pos[1] = msg.data[1]

	def timer_callback(self):
		diff = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		diff[0] = self.tar_pos[0] - self.cur_pos[0]
		diff[1] = self.tar_pos[1] - self.cur_pos[1]
		diff[2] = self.tar_pos[2] - self.cur_pos[2]
		diff[3] = self.tar_pos[3] - self.cur_pos[3]
		diff[4] = self.tar_pos[4] - self.cur_pos[4]
		diff[5] = self.tar_pos[5] - self.cur_pos[5]
		self.publish_arrive(diff)
		self.publish_control(diff)

	def publish_arrive(self, diff):
		margin = 0.05
		if (diff[0] < margin) & (diff[0] > -margin) & (diff[1] < margin) & (diff[1] > -margin):
			msg = Empty()
			self.arr_flag_publisher.publish(msg)

	def publish_control(self, diff):
		max_vel = 0.2
		p_gain = 3
		msg = Twist()
		msg.linear.x = self.clamp(p_gain * diff[0], max_vel)
		msg.linear.y = self.clamp(p_gain * diff[1], max_vel)
		msg.linear.z = self.clamp(p_gain * diff[2], max_vel)
		msg.angular.x = self.clamp(p_gain * diff[3], max_vel)
		msg.angular.y = self.clamp(p_gain * diff[4], max_vel)
		msg.angular.z = self.clamp(p_gain * diff[5], max_vel)
		self.planar_publisher.publish(msg)

	def clamp(self, value, max_val):
		if value > max_val:
			return max_val
		elif value < -max_val:
			return -max_val
		return value

def main():

    """ Main for spawning a robot node """
    # Get input arguments from user
    argv = sys.argv[1:]

    # Start node
    rclpy.init()

    # Get the file path for the robot model
    sdf_file_path = os.path.join(
        get_package_share_directory("autostore"), "models",
        "blackline", "model.sdf")
        
    # Create the node
    node = rclpy.create_node("entity_spawner")

    # Show progress in the terminal window
    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    # Get the spawn_entity service
    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the robot
    sdf_file_path = os.path.join(
        get_package_share_directory("autostore"), "models",
        "blackline", "model.sdf")

    # Show file path
    print(f"robot_sdf={sdf_file_path}")
    
    # Set data for request
    request = SpawnEntity.Request()
    request.name = argv[0]
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    my_robot_control = RobotControl(argv[1])
    rclpy.spin(my_robot_control)
    rclpy.shutdown()


if __name__ == "__main__":
    main()