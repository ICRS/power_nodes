import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import glob
import serial
import time
import re
import threading

COMMS_CURRENT = re.compile(r"<I_COMMS:([0-9]+)>")
ARM_CURRENT = re.compile(r"<I_ARM:([0-9]+)>")
DRILL_CURRENT = re.compile(r"<I_DRILL:([0-9]+)>")
BATTERY_VOLTAGE = re.compile(r"<V_BAT:([0-9]+)>")


class PowerBridgeNode(Node):
    def __init__(self):
        super().__init__('power_bridge')
        ports = sorted(glob.glob('/dev/ttyACM*'))
        if not ports:
            self.get_logger().error("No /dev/ttyACM* devices found.")
            raise RuntimeError("No ACM serial device found.")


        # Setup serial connection
        self.ser = None
        for port in ports:
            try:
                self.get_logger().info(f"Trying serial port: {port}")
                self.ser = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)  # give device time to reset if needed
                self.get_logger().info(f"Connected to {port}")
                break
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to open {port}: {e}")

        self.comms_current = self.create_publisher(Float32,"comms/current",1)
        self.drill_current = self.create_publisher(Float32,"drill/current",1)
        self.arm_current = self.create_publisher(Float32,"arm/current",1)
        self.battery_voltage = self.create_publisher(Float32,"vbat",1)
        

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        self.indicator_yellow_sub = self.create_subscription(Bool,"power/indicator_yellow",self.send_orange,1)
        self.indicator_green_sub = self.create_subscription(Bool,"power/indicator_green",self.send_green,1)


    def destroy_node(self):
        super().destroy_node()
        if self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")

    def read_loop(self):
        buffer = ""
        while rclpy.ok():
            if self.ser.in_waiting:
                buffer += self.ser.read(self.ser.in_waiting).decode(errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.parse_line(line.strip())
        
    def parse_line(self, line):
        if m := COMMS_CURRENT.match(line):
            msg = Float32()
            msg.data = float(m.group(1))/1000
            self.comms_current.publish(msg)

        elif m := ARM_CURRENT.match(line):
            msg = Float32()
            msg.data = float(m.group(1))/1000
            self.arm_current.publish(msg)

        elif m := DRILL_CURRENT.match(line):
            msg = Float32()
            msg.data = float(m.group(1))/1000
            self.drill_current.publish(msg)
        
        elif m := BATTERY_VOLTAGE.match(line):
            msg = Float32()
            msg.data = float(m.group(1))/1000
            self.battery_voltage.publish(msg)
        else:
            self.get_logger().warn(f"Unknown line: {line}")
    
    def send_yellow(self,msg:Bool):
            out = f"<LED_Y:{int(msg.data)}>\n"
            out_bytes = bytes(out.encode("utf-8"))
            self.ser.write(out_bytes)
    
    def send_green(self,msg:Bool):
            out = f"<LED_G:{int(msg.data)}>\n"
            out_bytes = bytes(out.encode("utf-8"))
            self.ser.write(out_bytes)

   

def main(args=None):
    rclpy.init(args=args)
    node = PowerBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
