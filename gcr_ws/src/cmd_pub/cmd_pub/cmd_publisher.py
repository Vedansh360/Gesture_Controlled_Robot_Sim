import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class CMDPublisher(Node):

    def __init__(self):
        super().__init__('cmd_publisher')
        
        self.publisher_ = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Int32,
            'finger_count',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, fcount):

        count = fcount.data

        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0

        if count == 5:
            msg.linear.x = 8.0
            msg.angular.z = 0.0
        elif count == 4:
            msg.linear.x = -8.0
            msg.angular.z = 0.0
        elif count == 0:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        elif count == 2:
            msg.linear.x = 2.0
            msg.angular.z = -1.0
        elif count == 1:
            msg.linear.x = 2.0
            msg.angular.z = 1.0
        else:
            msg.linear.x = 0.0
            msg.linear.z = 0.0

        # Log and publish the message
        self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}')
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    publisher = CMDPublisher()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()