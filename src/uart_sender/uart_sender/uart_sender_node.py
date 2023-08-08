import rclpy
import serial
import time
from rclpy.node import Node
import struct

serial_port = '/dev/ttyUSB2'
baud_rate = 9600
wheg_command = [40, 40, 40, 40, 0, 0, 0, 0]


def main(args=None):
    print('Hi from uart_sender_node.')
    stm32 = serial.Serial(serial_port, baud_rate, timeout=1)
    while True:
        try:
            stm32.write(wheg_command)
            print('send: ', wheg_command)
            time.sleep(0.1)
        except KeyboardInterrupt:
            print('send error')
            return

if __name__ == '__main__':
    main()
