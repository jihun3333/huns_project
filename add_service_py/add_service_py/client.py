import sys
import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node

class Client(Node):
    def __init__(self):
        super().__init__('client')
        self.cli = self.create_client(AddTwoInts, 'service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    client = Client()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e, )
                )
            else:
                client.get_logger().info(
                    'Result of ROS2 add_two_ints: for %d + %d = %d' %
                    (client.req.a, client.req.b, response.sum)
                )
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()