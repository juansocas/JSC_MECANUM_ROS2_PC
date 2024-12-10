import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist

class ConstantOdometryPublisher(Node):
    def __init__(self):
        super().__init__('constant_odometry_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        timer_period = 0.1  # Publica a 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_constant_odom)

    def publish_constant_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Posición constante (0, 0, 0)
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0

        # Orientación (sin rotación)
        odom_msg.pose.pose.orientation.w = 1.0  # Esto representa una rotación nula

        # Velocidad cero en todos los ejes
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Publica el mensaje en /odom
        self.publisher_.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConstantOdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
