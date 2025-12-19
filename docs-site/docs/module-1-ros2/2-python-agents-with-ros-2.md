---
sidebar_position: 2
---

# Python Agents with ROS 2

This chapter explores how to build intelligent agents using Python and ROS 2. We will cover the `rclpy` library, how to publish and subscribe to topics, and how to bridge AI agents with robot controllers.

## Using `rclpy`

`rclpy` is the official Python client library for ROS 2. It provides a Pythonic interface to all of the ROS 2 concepts like nodes, topics, services, and actions.

### Publishing and Subscribing

The most common way for nodes to communicate is by publishing and subscribing to topics. A publisher sends messages to a topic, and a subscriber receives those messages.

Here is a minimal example of a publisher and subscriber using `rclpy`.

#### Publisher

This node publishes a string message to the `topic` every half a second.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber

This node subscribes to the `topic` and prints the received message to the console.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bridging AI Agents to Controllers

A common pattern in robotics is to have a high-level AI agent that makes decisions, and a low-level controller that executes those decisions. For example, an AI agent might decide to move the robot's arm to a certain position, and the controller would be responsible for moving the arm's motors to reach that position.

ROS 2 topics are a great way to bridge the gap between the AI agent and the controller. The AI agent can publish desired states (e.g., target joint angles) to a topic, and the controller can subscribe to that topic and actuate the robot accordingly.

Here is a conceptual code example:

```python
# AI Agent (e.g., a reinforcement learning agent)
class AIAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')
        self.publisher_ = self.create_publisher(JointState, 'target_joint_states', 10)
        # ... AI logic to decide on the next action ...
        self.timer = self.create_timer(0.1, self.publish_action)

    def publish_action(self):
        joint_state_msg = JointState()
        # ... fill in the joint state message based on the AI's decision ...
        self.publisher_.publish(joint_state_msg)

# Controller
class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.subscription = self.create_subscription(
            JointState,
            'target_joint_states',
            self.state_callback,
            10)

    def state_callback(self, msg):
        # ... actuate the robot's motors to match the target joint states ...
        pass
```