import rclpy
import serial
import time
from rclpy.node import Node
from std_msgs.msg import String

from omniwheg_interfaces.msg import IMU, Wheel



serial_port = '/dev/ttyUSB2'
baud_rate = 9600
robot_data = '000'

class UartConverter(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.imuPublisher = self.create_publisher(IMU, 'pose', 10)
        self.wheelStatusPublisher = self.create_publisher(Wheel, 'wheelStatus', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Establish a connection to the serial port
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)

    def verifyCheckLength(self, data):
        if data:
            lengthInfo = data.split(" ")[-1]
            if lengthInfo:
                if len(lengthInfo) >= 1 and lengthInfo.isdigit():
                    if len(data[:-len(lengthInfo)].strip()) == int(lengthInfo):
                        return True
                    else:
                        return False
                else:
                    return False
            else:   
                return False
        else:   
            return False


    def imuInfoSeprate(self, raw_data):
        imuMsg = IMU()
        temArray = raw_data.split(" ")
        try:
            imuMsg.angle_x = float(temArray[1])
            imuMsg.angle_y = float(temArray[2])
            imuMsg.angle_z = float(temArray[3])
        except:
            self.get_logger().info("imu message error")
            return None
        return imuMsg

    def wheelInfoSeprate(self, raw_data):
        wheelMsg = Wheel()
        temArray = raw_data.split(" ")
        try:
            wheelMsg.id = int(temArray[1])
            wheelMsg.speed = float(temArray[2])
            wheelMsg.pwm = float(temArray[3])
            wheelMsg.current = float(temArray[4])
            wheelMsg.position = float(temArray[5])
            wheelMsg.circle = int(temArray[6])
        except:
            self.get_logger().info("wheel message error")
            return None
        return wheelMsg


    def timer_callback(self):
        # Read data from the serial port
        try:
            robot_data = self.ser.readline().decode('utf-8').strip()
        except:
            self.get_logger().info("uart read error")
            return None
        if robot_data:
            self.get_logger().info('raw message: "%s" ' % robot_data)
            if self.verifyCheckLength(robot_data):
                robot_data = robot_data[:-len(robot_data.split(" ")[-1])]
                if robot_data[0] == 'i':
                    imuMsg = self.imuInfoSeprate(robot_data)
                    self.imuPublisher.publish(imuMsg)
                elif robot_data[0] == 'w':
                    wheelMsg = self.wheelInfoSeprate(robot_data)
                    self.wheelStatusPublisher.publish(wheelMsg)
                else:
                    self.get_logger().info("useless message: %s, throw away" % robot_data)
            else:
                self.get_logger().info("message length error")
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
