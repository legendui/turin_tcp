import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import xml.etree.ElementTree as ET
import threading

class JointStateListener(Node):
    def __init__(self):
        super().__init__('turin_joint_state')
        # Declare and retrieve necessary ROS parameters for operation
        self.declare_parameter('server_ip', '172.16.20.32')
        self.declare_parameter('server_port', 8527)
        self.declare_parameter('update_interval', 0.5)  # Frequency of sending requests in seconds

        server_ip = self.get_parameter('server_ip').value
        server_port = self.get_parameter('server_port').value

        # Setup the socket connection to the specified IP and port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((server_ip, server_port))
            self.get_logger().info(f"Connected to {server_ip} on port {server_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {server_ip} on port {server_port}: {str(e)}")
            self.sock = None
            return

        # Initialize publisher for publishing joint states as a simple string
        self.publisher = self.create_publisher(String, 'turin_joint_states', 10)

        # Setup a timer to send requests at specified intervals
        self.timer = self.create_timer(self.get_parameter('update_interval').value, self.send_request)

        # Thread to listen for responses from the server
        self.listener_thread = threading.Thread(target=self.listen_for_data, daemon=True)
        self.listener_thread.start()

    def send_request(self):
        # XML request structure
        request_xml = '''<?xml version="Turin.Robot.V2.0" encoding="UTF-8"?>
<Bodys>
<Cmd Name="GetCurrAllPos" CmdCont="0" Status="Send">
<Param Tool="1" User="0" />
</Cmd>
</Bodys>'''
        if self.sock:
            try:
                self.sock.sendall(request_xml.encode('utf-8'))
                self.get_logger().info("Data request sent to the server.")
            except Exception as e:
                self.get_logger().error(f"Failed to send data request: {str(e)}")

    def listen_for_data(self):
        # Continuously listen for data while the node is active
        while rclpy.ok():
            if not self.sock:
                return
            try:
                data = self.sock.recv(1024)  # Adjust buffer size based on expected data volume
                if data:
                    self.process_response(data.decode())
                else:
                    self.get_logger().info('No data received, connection might be closed')
            except Exception as e:
                self.get_logger().error(f"Error receiving data: {str(e)}")
                break

    def process_response(self, response):
        # Process the XML response and extract joint data
        try:
            root = ET.fromstring(response)
            data_element = root.find('.//Data')
            if data_element is None:
                self.get_logger().error('Data element not found in the response XML.')
                return

            data_text = data_element.text.strip()
            joint_line = next((line for line in data_text.splitlines() if line.strip().startswith('Joint:')), None)
            if joint_line is None:
                self.get_logger().error('Joint data not found in the Data element.')
                return

            joint_data_str = joint_line.split('Joint:')[1].strip()
            joint_values = [int(float(val)) for val in joint_data_str.split(',')]
            joint_values_str = ', '.join(map(str, joint_values))

            self.get_logger().info(f"Publishing Joint Values: {joint_values_str}")
            self.publisher.publish(String(data=joint_values_str))
        except Exception as e:
            self.get_logger().error(f'Error processing response: {str(e)}')

    def __del__(self):
        # Ensure the socket is closed cleanly
        if self.sock:
            self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    joint_state_listener = JointStateListener()
    rclpy.spin(joint_state_listener)
    joint_state_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
