import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class SimpleBotJointStatePublisher(Node):
    def __init__(self):
        super().__init__('simple_bot_joint_state_publisher')
        timer_period = 0.1
        self.joint_position = 0.0
        self.direction = 'left'
        self.lower_limit = -0.15
        self.upper_limit = 0.15
        self.position_delta = 0.005
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(timer_period, self.publish_joint_states)

    def publish_joint_states(self):
        self.update_joint_position()
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["right_wheel_link_joint"]
        joint_state.position = [self.joint_position]
        self.publisher_.publish(joint_state)

    def update_joint_position(self):
        self.update_joint_position_to_left()
        self.update_joint_position_to_right()

    def update_joint_position_to_right(self):
        if self.lower_limit <= self.joint_position and self.direction == 'right':
            self.joint_position -= self.position_delta
        else:
            self.direction = 'left'

    def update_joint_position_to_left(self):
        if self.upper_limit >= self.joint_position and self.direction == 'left':
            self.joint_position += self.position_delta
        else:
            self.direction = 'right'

def main(args=None):
    rclpy.init(args=args)
    simple_bot_joint_state_publisher = SimpleBotJointStatePublisher()
    rclpy.spin(simple_bot_joint_state_publisher)
    simple_bot_joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()