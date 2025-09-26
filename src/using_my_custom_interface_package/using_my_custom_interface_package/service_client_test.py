import sys
import rclpy
from rclpy.node import Node
from msg_interface_example.srv import PlusService

class MinimalClientAsync(Node):
    
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(PlusService, 'plus')
        
        # Service Server가 켜져있는지 1.0초마다 체크하는 문장 
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, wating again...')
        # srv 파일의 request 부분 가져오기
        self.req = PlusService.Request()
        
    def send_request(self):
        # Command Line에서 Argument를 넣어줌.
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        # Service 요청을 보내는 부분
        self.future = self.cli.call_async(self.req)
        
def main(args = None):
    rclpy.init(args=args)
    
    minimal_client = MinimalClientAsync()
    minimal_client.send_request() # 서비스 요청 보내는 부분
    
    while rclpy.ok(): # Service request가 들어오면 응답함.
        # Service 요청은 한 번이기 때문에 spin 대신에 spin_once 함수 사용
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                # 서비스의 결과를 받아옴.
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call faild %r'%(e,)
                )
            else:
                minimal_client.get_logger().info('Result of add_three_ints: for %d + %d = %d'%(minimal_client.req.a, minimal_client.req.b, response.sum))
                
            break