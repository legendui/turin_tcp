import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import xml.etree.ElementTree as ET
import time
import math  # 用于弧度到角度的转换

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('update_interval', 0.5)  # Interval in seconds for sending data

        joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.update_interval = self.get_parameter('update_interval').get_parameter_value().double_value

        self.publisher = self.create_publisher(String, 'tcp_send_data', 10)
        self.subscription = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10)

        self.last_sent_time = time.time()
        self.latest_joint_state = None

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg  # Update the latest state
        self.maybe_send_update()

    def maybe_send_update(self):
        current_time = time.time()
        if (current_time - self.last_sent_time) >= self.update_interval:
            if self.latest_joint_state is not None:
                xml_data = self.convert_to_xml(self.latest_joint_state)
                self.publisher.publish(String(data=xml_data))
                self.last_sent_time = current_time  # Update the last sent time

    def convert_to_xml(self, joint_state):
        positions_in_degrees = [math.degrees(pos) for pos in joint_state.position]
        
        # 创建XML字符串
        xml_str = f'''<?xml version="Turin.Robot.V2.0" encoding="UTF-8"?>
<Bodys>
<Cmd Name="MotionStart" CmdCont="0" Status="Send">
<Param UseInThread = "false" IsGcode = "false" MainFileName = "test.txt"
StartFileName = "" StartLine = "1" ExeLines = "0" ExeLoops = "0" />
<Data>MoveJ(0,0,0,0,0,0,0.000,0.002,0.000,{positions_in_degrees[0]},{positions_in_degrees[1]},{positions_in_degrees[3]},{positions_in_degrees[4]},{positions_in_degrees[2]},{positions_in_degrees[5]},0.000,0.002,0.000,10,0,1,0,01,00)

##关节运动 S=10% T=01

</Data>
</Cmd>
</Bodys>'''
        
        return xml_str

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)

    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()