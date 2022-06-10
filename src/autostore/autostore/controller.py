import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_msgs.msg import Int32MultiArray

class Controller(Node):
	def __init__(self):
		super().__init__('controller')
		self.robot_enable = [True]
		self.robot_position = [1, 1]
		self.robot_waypoints = []
		self.order_list = []
		self.robot_cmd_publisher = self.create_publisher(Int32MultiArray, '/blackline_1/cmd_moveto', 10)
		self.robot_arr_subscriber = self.create_subscription(Empty, '/blackline_1/arrive', self.set_enable, 10)
		self.order_subscriber = self.create_subscription(Int32MultiArray, '/controller/order', self.add_order, 10)
		self.timer = self.create_timer(0.5, self.timer_callback)

	def set_enable(self, msg):
		self.robot_enable = True

	def add_order(self, msg):
		print('order to ', msg.data)
		self.order_list.insert(0, msg.data)

	def timer_callback(self):
		if self.robot_enable:
			if len(self.robot_waypoints) > 0:
				msg = Int32MultiArray()
				msg.data = self.robot_waypoints.pop()
				print('move to ', msg.data)
				self.robot_cmd_publisher.publish(msg)
				self.robot_position = msg.data
				self.robot_enable = False
			elif len(self.order_list) > 0:
				self.robot_waypoints = self.generate_waypoint(self.order_list.pop(), self.robot_position)

	def generate_waypoint(self, dst, src):
		waypoints = []
		if (src[1] != dst[1]):
			waypoints.insert(0, [src[0], dst[1]])
		if (src[0] != dst[0]):
			waypoints.insert(0, [dst[0], dst[1]])
		return waypoints

def main():
	rclpy.init()
	my_controller = Controller()

	rclpy.spin(my_controller)

	my_controller.destroy_node()
	rclpy.shutdown