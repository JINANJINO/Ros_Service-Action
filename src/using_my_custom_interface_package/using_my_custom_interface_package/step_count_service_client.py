import sys
import rclpy
from rclpy.node import Node
from msg_interface_example.srv import TwoStringService

class MinimalClientAsync(Node):
    
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(TwoStringService, 'jinhan_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, wating again...')
        self.req = TwoStringService.Request()
        
    def send_request(self):
        self.req.text_a = str(sys.argv[1])
        self.req.text_b = str(sys.argv[2])
        self.future = self.cli.call_async(self.req)
        
def main(args = None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    minimal_client.send_request()
    
    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info('Service call failed %r'%(e, ))
            else:
                minimal_client.get_logger().info('Result of add_three_ints: for %s + %s = %s'%(minimal_client.req.text_a, minimal_client.req.text_b, response.sum))
            break