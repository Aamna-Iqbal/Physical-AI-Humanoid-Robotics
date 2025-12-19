---
sidebar_position: 1
---

# ROS 2 Fundamentals

Welcome to the first chapter of our course on the Robotic Nervous System (ROS 2). This chapter introduces the fundamental concepts of ROS 2, providing the building blocks for creating complex and robust robotic applications.

## What is ROS 2?

ROS 2 (Robot Operating System 2) is a set of software libraries and tools that help you build robot applications. It is a flexible framework for writing robot software, from low-level device drivers to high-level algorithms. ROS 2 is not an operating system in the traditional sense, but a meta-operating system that provides services like hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.

## Core Concepts

### Nodes

A **node** is the smallest unit of computation in ROS 2. It is a process that performs a specific task. For example, a node might control a wheel motor, read data from a laser range-finder, or plan a path for the robot. A well-designed robot system is composed of many nodes, each with a specific purpose. This modularity makes the system easier to debug, maintain, and scale.

### Topics

**Topics** are named buses over which nodes exchange messages. Topics have anonymous publish/subscribe semantics, which means that a node that publishes a message does not know which nodes will receive it, and a node that subscribes to a topic does not know which node is publishing the messages. This decoupling of nodes is a powerful feature of ROS 2 that allows for great flexibility and reusability.

### Services

While topics are used for continuous data streams, **services** are used for request/reply communication. A service is defined by a pair of messages: one for the request and one for the reply. One node (the client) sends a request message to another node (the server) and waits for a reply. This is a synchronous communication model, in contrast to the asynchronous nature of topics.

### Actions

**Actions** are similar to services, but they are designed for long-running tasks. An action provides feedback on the progress of the task and can be preempted or canceled. This is useful for tasks like navigation, where it is important to get feedback on the robot's progress and to be able to stop the robot if necessary.

### DDS (Data Distribution Service)

ROS 2 is built on top of DDS, a middleware standard for data-centric publish-subscribe messaging. DDS provides a robust and efficient communication layer for ROS 2, with features like automatic discovery of nodes, quality of service (QoS) settings for reliable communication, and support for real-time systems.

## Code Examples

Here are some minimal, runnable code examples to illustrate the core concepts.

### ROS 2 Node (Python)

This example shows a simple "Hello World" node in Python.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Hello, ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ROS 2 Topic (Python)

This example shows a simple publisher and subscriber for a string message.

#### Publisher

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

### ROS 2 Service (Python)

This example shows a simple service client and server.

#### Service Server

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (1, 2, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
