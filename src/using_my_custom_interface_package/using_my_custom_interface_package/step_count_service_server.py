import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from msg_interface_example.srv import TwoStringService
from msg_interface_example.msg import TwoNumSum


class MinimalService(Node):
    
    def __init__(self):
        super().__init__('minimal_service')
        qos_profile = QoSProfile(depth=10)

        # 서비스 서버 생성
        self.srv = self.create_service(
            TwoStringService,
            'jinhan_service',
            self.add_two_callback
        )

        # 구독자 생성 (TwoNumSum에서 num_a만 받음)
        self.subscriber = self.create_subscription(
            TwoNumSum,
            'step_count',
            self.subscriber_callback,
            qos_profile
        )

        # 최신 구독 데이터를 저장할 변수
        self.latest_num = None

    # 서비스 콜백
    def add_two_callback(self, request, response):
        if self.latest_num is not None:
            response.sum = request.text_a + request.text_b + str(self.latest_num)
            self.get_logger().info(
                f"Incoming request: text_a={request.text_a}, text_b={request.text_b}, "
                f"num_a(from topic)={self.latest_num} -> sum={response.sum}"
            )
        else:
            response.sum = request.text_a + request.text_b
            self.get_logger().warn(
                "No step_count message received yet, using only text fields."
            )
        return response

    # 구독 콜백
    def subscriber_callback(self, msg):
        self.latest_num = msg.num_a
        


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
