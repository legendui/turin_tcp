import rclpy
from rclpy.node import Node
import socket

class TCPClientNode(Node):
    def __init__(self):
        super().__init__('tcp_client_node')
        
        # Declare parameters for server address and port with default values
        self.declare_parameter('server_address', '172.16.20.32')
        self.declare_parameter('server_port', 8527)

        # Get the parameters
        server_address = self.get_parameter('server_address').get_parameter_value().string_value
        server_port = self.get_parameter('server_port').get_parameter_value().integer_value

        self.get_logger().info('TCP Client Node has started.')
        self.get_logger().info(f'Connecting to {server_address}:{server_port}')

        # TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the server
        server = (server_address, server_port)
        self.sock.connect(server)
        self.get_logger().info(f'Connected to {server}')

        # Send and receive data as per your requirement
        # Make sure to handle exceptions and close the socket properly

def main(args=None):
    rclpy.init(args=args)
    node = TCPClientNode()
    rclpy.spin(node)
    
    # Clean up
    node.sock.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
