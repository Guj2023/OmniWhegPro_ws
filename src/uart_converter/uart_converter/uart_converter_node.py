import rclpy
import serial
import time
from rclpy.node import Node
from std_msgs.msg import String

from omniwheg_interfaces.msg import IMU, Wheel



serial_port = '/dev/ttyUSB1'
baud_rate = 9600
robot_data = '000'

class UartConverter(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.imuPublisher = self.create_publisher(IMU, 'pose', 10)
        self.wheelStatusPublisher = self.create_publisher(Wheel, 'wheelStatus', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Establish a connection to the serial port
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
    def imuInfoSeprate(self, raw_data):
        imuMsg = IMU()
        temArray = raw_data.split("\t")
        for item in temArray:
            if item[0] == 'x':
                imuMsg.angle_x = float(item[2:].strip())
            elif item[0] == 'y':
                imuMsg.angle_y = float(item[2:].strip())
            elif item[0] == 'z':
                imuMsg.angle_z = float(item[2:].strip())
            else:
                self.get_logger().info("imu message error")
        return imuMsg

    def wheelInfoSeprate(self, raw_data):
        wheelMsg = Wheel()
        wheelMsg.circle = 0
        temArray = raw_data.split("\t")
        for item in temArray:
            if item[0] == 'i':
                wheelMsg.id = int(item[2:].strip())
            elif item[0] == 's':
                wheelMsg.speed = float(item[2:].strip())
            elif item[0:3] == 'PWM':
                wheelMsg.pwm = float(item[4:].strip())
            elif item[0:1] == 'c:':
                wheelMsg.current = float(item[2:].strip())
            elif item[0] == 'p':
                wheelMsg.position = float(item[2:].strip())
            elif item[0:1] == 'ci':
                wheelMsg.circle = int(item[3:].strip())
            else:
                self.get_logger().info("wheel status message error")
        return wheelMsg


    def timer_callback(self):
        # Read data from the serial port
        robot_data = self.ser.readline().decode('utf-8').strip()
        if robot_data:
            self.get_logger().info('raw message: "%s" ' % robot_data)
            if robot_data[0] == 'x':
                imuMsg = self.imuInfoSeprate(robot_data)
                self.imuPublisher.publish(imuMsg)
            elif robot_data[0] == 'i':
                wheelMsg = self.wheelInfoSeprate(robot_data)
                self.wheelStatusPublisher.publish(wheelMsg)
            else:
                self.get_logger().info("useless message: %s, throw away" % robot_data)
        else:
            self.get_logger().info("No data")



    

def main(args=None):
    print('Hi from uart_converter_node.')
    rclpy.init(args=args)
    uart_converter = UartConverter()

    rclpy.spin(uart_converter)

    uart_converter.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
