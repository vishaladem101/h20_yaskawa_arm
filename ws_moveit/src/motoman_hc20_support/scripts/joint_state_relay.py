#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states_gui',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/hc20_arm_controller/commands', 10)
        self.joint_names = [
            'joint_1_s', 'joint_2_l', 'joint_3_u', 
            'joint_4_r', 'joint_5_b', 'joint_6_t'
        ]

    def listener_callback(self, msg):
        cmd = Float64MultiArray()
        # Map joint states to command positions in correct order
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        
        if len(positions) == 6:
            cmd.data = positions
            self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    relay = JointStateRelay()
    rclpy.spin(relay)
    relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
