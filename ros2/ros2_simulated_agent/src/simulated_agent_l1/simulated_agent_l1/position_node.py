import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs import msg
from geometry_msgs.msg import Point
import builtins


class SendPosition(Node):

    def __init__(self):
        super().__init__('send_position')

        #Declare parameters that are set in config.yaml file
        self.declare_parameters(
            namespace='',
            parameters= [
                ('lon', rclpy.Parameter.Type.DOUBLE),
                ('lat', rclpy.Parameter.Type.DOUBLE),
                ('alt', rclpy.Parameter.Type.DOUBLE)
            ]
        )

        #Create ros publisher to talk to other node
        self.publisher = self.create_publisher(Point, 'cmd_vel', 10)

        self.position = Point()
        self.position.x = self.get_parameter('lon').value
        self.position.y = self.get_parameter('lat').value
        self.position.z = self.get_parameter('alt').value
        
        #Create timer to publish to the other node one time per second
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Publishing: "%s"' % self.position)
        self.publisher.publish(self.position)

def main():
    rclpy.init()
    node = SendPosition()
    rclpy.spin(node)

if __name__ == '__main__':
    main()