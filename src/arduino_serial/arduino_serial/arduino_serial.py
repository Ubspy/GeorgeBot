import serial
import rclpy

from georgebot_msgs.msg import Direction 
from georgebot_msgs.msg import IMUData
from rclpy.node import Node

class ArduinoSerial(Node):
    def __init__(self):
        super().__init__('arduino_serial')

        # Set subscriber
        self.subscription = self.create_subscription(
                Direction,
                'move_direction',
                self.arduino_data_send,
                10)

        # Suppress unused warning
        self.subscription

        # Set publisher
        self.publisher = self.create_publisher(IMUData, 'imu_data', 10)

        # Initialize serial and get rid of bogus data
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
        self.ser.flush()

    def arduino_data_recv(self):
        # Check if there's any data waiting
        if(self.ser.in_waiting > 0):
            # Read everything from the serial until a new line character as a string
            ser_data = self.ser.readline().decode('utf-8').rstrip()

            # Split the data by a common delimiter (in this case, a comma)
            ser_data = ser_data.split(',')

            # Logging
            self.get_logger().info('Received from arduino: %s' % ser_data)

            to_pub = IMUData()
            to_pub.yaw = float(ser_data[0])
            to_pub.x_encoder_left = int(ser_data[1])
            to_pub.x_encoder_right = int(ser_data[2])
            to_pub.y_encoder_left = int(ser_data[3])
            to_pub.y_encoder_right = int(ser_data[4])
            to_pub.x_accel = float(ser_data[5])
            to_pub.y_accel = float(ser_data[6])

    def arduino_data_send(self, msg):
        # Create variable to store serial wait value
        currently_waiting = self.ser.out_waiting

        # Wait for serial to be availible
        while(currently_waiting <= 0):
            # Sleep while waiting for serial output buffer to be empty
            currently_waiting = self.ser.out_waiting

        # Create a string to send the direction info to the arduino via serial
        send_data = "%f,%f,%f\n" % (msg.x, msg.y, msg.theta)

        # Logging
        self.get_logger().info('Sending to arduino: %s' % send_data)

        # Write the data to the serial
        self.ser.write(send_data.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    arduino_serial = ArduinoSerial()

    rclpy.spin(arduino_serial)

    arduino_serial.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
