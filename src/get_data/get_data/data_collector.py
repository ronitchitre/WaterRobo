import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Wrench

pos_file = open("test_pos.txt", "+w")
twist_file = open("test_twist.txt", "+w")
j_vec_file = open("test_j_world.txt", "+w")
fore_file = open("test_fore.txt", "+w")
star_file = open("test_stat.txt", "+w")
port_file = open("test_port.txt", "+w")


class DataCollector(Node):

    def __init__(self):
        super().__init__('get_data')
        self.pos_sub = self.create_subscription(
            Point,
            '/cur_pos',
            self.record_pos,
            10)
        self.twist_sub = self.create_subscription(
            Twist,
            "/cur_twist",
            self.record_twist,
            10)
        self.j_world_sub = self.create_subscription(
            Vector3,
            "/j_world",
            self.record_j_world,
            10)
        self.fore_sub = self.create_subscription(
            Wrench,
            "/thrust/fore",
            self.record_fore,
            10)
        self.star_sub = self.create_subscription(
            Wrench,
            "/thrust/starboard",
            self.record_star,
            10)
        self.port_sub = self.create_subscription(
            Wrench,
            "/thrust/port",
            self.record_port,
            10)
        self.pos_sub
        self.twist_sub
        self.j_world_sub
        self.fore_sub
        self.star_sub
        self.port_sub

    def record_pos(self, msg):
        pos_file.write(f"{msg.x} {msg.y} {msg.z} \n")
    
    def record_twist(self, msg):
        twist_file.write(f"{msg.linear.x} {msg.linear.y} {msg.linear.z} {msg.angular.x} {msg.angular.y} {msg.angular.z} \n")
    
    def record_j_world(self, msg):
        j_vec_file.write(f"{msg.x} {msg.y} {msg.z} \n")

    def record_fore(self, msg):
        fore_file.write(f"{msg.force.z} \n")
    
    def record_star(self, msg):
        star_file.write(f"{msg.force.z} \n")
    
    def record_port(self, msg):
        port_file.write(f"{msg.force.z} \n")
        


def main(args=None):
    rclpy.init(args=args)

    data_collector = DataCollector()

    rclpy.spin(data_collector)
    pos_file.close()
    twist_file.close()
    j_vec_file.close()
    fore_file.close()
    star_file.close()
    port_file.close()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()