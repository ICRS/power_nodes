import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Joy
import time
from rclpy.executors import MultiThreadedExecutor
import time


class PowerNode(Node):

    def __init__(self):
        super().__init__('power_coordinator')

        self.drill_sub = self.create_subscription(Float32, "drill/current",self.update_drill, 1)
        self.comms_sub = self.create_subscription(Float32,"comms/current",self.update_comms,1)
        self.arm_sub = self.create_subscription(Float32,"arm/current",self.update_arm,1)
        self.drive_l_sub = self.create_subscription(Float32,"drive/l_current",self.update_drive_l,1)
        self.drive_r_sub = self.create_subscription(Float32,"drive/r_current",self.update_drive_r,1)
        self.heartbeat_sub = self.create_subscription(Joy, "drive/joy", self.heartbeat,1)

        self.green_pub = self.create_publisher(Bool, "power/indicator_green", 1)
        self.yellow_pub = self.create_publisher(Bool, "power/indicator_yellow", 1)
    
        
        self.time = self.create_timer(1,self.handle_indicator)
        self.currents = {"drill":0, "comms":0 ,"arm":0, "drive_l":0, "drive_r":0}
        self.yellow_status = False
        self.green_status = False
        self.timeout_s = 5
        self.last_joy_s = 0

        

    def update_drill(self, msg: Float32):
        self.currents['drill']=msg.data

    def update_arm(self, msg: Float32):
        self.currents['arm']=msg.data

    def update_comms(self, msg: Float32):
        self.currents['comms']=msg.data

    def update_drive_l(self, msg: Float32):
        self.currents['drive_l']=msg.data

    def update_drive_r(self, msg: Float32):
        self.currents['drive_r']=msg.data

    def publish_total(self, msg: Float32):
        total = sum(self.currents.values)
        msg = Float32()
        msg.data = total
        self.total_pub.publish(msg)

    def heartbeat(self, msg:Joy):
        self.last_joy_s = time.time()

    def handle_indicator(self):
        time_now = time.time()
        if time_now < (self.last_joy_s + self.timeout_s):
            self.get_logger().info("Drive joy detected, flashing green")
            msg = Bool()
            msg.data = not self.green_status
            self.green_pub.publish(msg)
            msg = Bool()
            msg.data = False
            self.yellow_pub.publish(msg)
        else:
            self.get_logger().info("Drive joy not detected, flashing yellow")
            msg = Bool()
            msg.data = not self.yellow_status
            self.yellow_pub.publish(msg)
            msg = Bool()
            msg.data = False
            self.green_pub.publish(msg)
    


def main(args=None):
    rclpy.init(args=args)
    node = PowerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
