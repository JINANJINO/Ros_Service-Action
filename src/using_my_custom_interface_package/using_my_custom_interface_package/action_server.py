import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from msg_interface_example.action import Fibonacci

class FibonacciActionServer(Node):
    
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Excuting goal...')
        
        # 정의한 Action Interface의 Feedback 자료형 선언
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        
        # goal_handle.request.order -> Clinet에서 보내는 목표
        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i -1]
            )
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg) # 중간목표
            time.sleep(1)
        
        goal_handle.succeed() # 결과반환
        
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
    
def main(args = None):
    rclpy.init(args=args)
        
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
        
if __name__ == '__main__':
    main()    
