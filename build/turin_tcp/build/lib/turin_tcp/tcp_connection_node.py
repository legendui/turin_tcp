import rclpy
from rclpy.node import Node
import socket
import threading
from std_msgs.msg import String

class TCPConnectionNode(Node):
    def __init__(self):
        super().__init__('tcp_connection_node')
        self.declare_parameter('server_ip', '172.16.20.32')
        self.declare_parameter('server_port', 8527)
        
        server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((server_ip, server_port))
        self.get_logger().info(f'Connected to {server_ip}:{server_port}')

        # Send the login command after connecting
        self.send_login_command()

        self.publisher = self.create_publisher(String, 'tcp_receive_data', 10)
        self.subscription = self.create_subscription(
            String,
            'tcp_send_data',
            self.listener_callback,
            10)

        # Start the listener thread
        self.listener_thread = threading.Thread(target=self.receive_data)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def send_login_command(self):
        login_xml = '''<?xml version="Turin.Robot.V2.0" encoding="UTF-8"?>
<Bodys>
<Cmd Name="Login" CmdCont="0" Status="Send">
<Param UserName="administrator" Password="12345678"/>
</Cmd>
</Bodys>'''
        try:
            self.sock.sendall(login_xml.encode('utf-8'))
            self.get_logger().info('Login command sent')
        except Exception as e:
            self.get_logger().error('Failed to send login command: ' + str(e))
            self.sock.close()  # Consider reconnection logic here

    def listener_callback(self, msg):
        try:
            self.sock.sendall(msg.data.encode('utf-8'))
            self.get_logger().info(f'Sent data: {msg.data}')
        except Exception as e:
            self.get_logger().error('Failed to send data: ' + str(e))
            self.sock.close()  # Consider reconnection logic here

    def receive_data(self):
        while True:
            try:
                data = self.sock.recv(1024)  # Buffer size can be adjusted
                if data:
                    self.get_logger().info(f'Received data: {data.decode()}')
                    self.publisher.publish(String(data=data.decode()))
                else:
                    break
            except Exception as e:
                self.get_logger().error(f'Failed to receive data: {e}')
                break
        self.sock.close()

    def __del__(self):
        if self.sock:
            self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    node = TCPConnectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
