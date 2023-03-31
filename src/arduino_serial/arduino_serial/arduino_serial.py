import os.path
import rclpy
import serial

from georgebot_msgs.msg import Direction 
from georgebot_msgs.msg import IMUData
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

class ArduinoSerial(Node):
    def __init__(self):
        super().__init__('arduino_serial')

        # Add parameter for serial port (with a default value) with a description
        pc_port = ParameterDescriptor(description="The location of the serial interface for the arduino")
        self.declare_parameter('serial_port', '/dev/ttyACM0', pc_port)

        # Add parameter for the baud rate
        pc_rate = ParameterDescriptor(description="The baud rate of the arduino serial communication")
        self.declare_parameter('baud_rate', 9600, pc_rate)

        # Set subscriber
        self.subscription = self.create_subscription(
                Direction,
                'move_direction',
                self.arduino_data_send,
                10
        )

        # Suppress unused warning
        self.subscription

        # Timer for checking serial input
        self.period = 0.1

        # Set publisher
        self.publisher = self.create_publisher(IMUData, 'imu_data', 10)

        # Create timer callback for arduino receive
        self.timer = self.create_timer(self.period, self.arduino_data_recv)

        # Get parameter values and store them locally
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            # Initialize serial and get rid of bogus data
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.ser.flush()

            # Log that serial has been established
            self.get_logger().info("Serial communication has been established on port %s with baud rate %i" % (self.serial_port, self.baud_rate))
        except serial.serialutil.SerialException:
            self.serial_error_catch()

    def serial_error_catch(self):
        # Print serial error
        self.get_logger().error("Could not find serial device at %s, waiting for device..." % self.serial_port.value)
        
        # Wait for there to be a file at the serial location
        serial_port_present = False

        while(not serial_port_present):
            # Wait for this serial port to appear
            serial_port_present = os.path.isfile(self.serial_port.value)
        
        # Once that's done send an info message
        self.get_logger().info("Serial reconnected at port %s" % self.serial_port.value)

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

            self.publisher.publish(to_pub)

    def arduino_data_send(self, msg):
        # Create variable to store serial wait value
        currently_waiting = self.ser.out_waiting

        # Wait for serial to be availible
        while(currently_waiting > 0):
            # Sleep while waiting for serial output buffer to be empty
            currently_waiting = self.ser.out_waiting

        # Create a string to send the direction info to the arduino via serial
        send_data = "%f,%f,%f\n" % (msg.x, msg.y, msg.theta)

        # Logging
        self.get_logger().info('Sending serial data to the arduino: %s' % send_data)

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
