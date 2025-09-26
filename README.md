# Ros Service&Action
## 1. Ros Service
Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model versus the publisher-subscriber model of topics. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

![Service-SingleServiceClient](https://github.com/user-attachments/assets/5e012ca1-8fbe-4f38-8919-8c20e279d0ed)

Before creating a service server and service client, we need to create ```srv```.
You can check out ```srv``` in my repository under ```msg_interface_example```. If you create it yourself, you'll need to create the srv directory after creating the package.

- **PlusService.srv**

```bash
int64 a
int64 b
---
int64 sum
```
In the interface file contents, the upper part is the ```Request``` contents and the lower part is the ```Response``` contents.

After that, you need to go into ```CMakelists.list``` and edit the contents.
```CMake
set(srv_files
  "srv/PlusService.srv"
  "srv/TwoStringService.srv"
)
```
```CMake
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)
```
Now you need to write the code related to the ```service server``` and ```service client```.

- **service_server_test.py**
```python
def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d'%(request.a, request.b))
        
        return response
```
Here, the author combined two values ​​when they entered the callback function from the service server.

- **service_client_test.py**
```python
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
```
In the service client file, it is set to receive two integer values ​​from the command line.

Finally, add what we wrote to the ```entry point``` of ```setup.py```, build it, and then run Node.

---

## 2. Ros Service Assignment
- Create a publisher that issues integer topics that increase by 1 at each step.
  > Topic name : ```step_count```

- Create a service server
  > Receive two strings (create ```TwoStringService.srv```)
  > Concatenate the two input strings with the most recently received step_count topic and return the result.

- Create a service client
  > Send ```TwoStringService.srv``` to the server
  > The user enters two strings on the command line.

My result: 
- Publisher Code : ```step_count.py```
- Service Server : ```step_count_service_server.py```
- Service Client : ```step_count_service_client.py```
  
---

## 3. Ros Action

Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

Actions are built on topics and services. Their functionality is similar to services, except actions are preemptable (you can cancel them while executing). They also provide steady feedback, as opposed to services which return a single response.

Actions use a client-server model, similar to the publisher-subscriber model. An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

![Action-SingleActionClient](https://github.com/user-attachments/assets/7c3ae291-05f8-4851-ac83-aabaedd6419b)

Before designing the Action server and Action Client nodes, you must create the messages associated with the Action.
Find the ```action``` directory in my repository. Inside that directory, I have defined the action in the ```Fibonacci.action``` file.

- Fibonacci.action
```bash
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

Afterwards, you need to go into the ```CMakeLists.txt``` file and configure it.
```CMake
set(action_files
  "action/Fibonacci.action"
)
```

Since we added ```action``` to dependencies when setting up srv earlier, we will not add any code separately.

Now we need to create the Action server and Action Client nodes. Check out the following files in my repository: 

```action_client.py```, ```action_server.py```

-  **action_server.py**

```python
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
```
This is the part that processes feedback in the ```callback function```. Like the Fibonacci sequence, it starts from [0,1], adds the two requested values, and appends them to the list.

```python
goal_handle.succeed() # 결과반환
        
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
```
And below the callback function, there is a code that returns the result. In other words, it is the content that ultimately returns the appended result value.

- **action_client.py**

```python
def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        # Action Server Waiting
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
```
First, declare the data type by importing the Goal from the Action message you previously wrote. Then, wait for the server to respond and send the goal asynchronously. Additionally, specify the ```goal_response_callback``` function to be used.

```python
def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')
        
        self._get_result_future = goal_handle.get_result_async()        
        self._get_result_future.add_done_callback(self.get_result_callback)
```

This function checks whether the goal has been properly delivered. 

```python
  def get_result_callback(self, future):
        action_status = future.result().status
        result = future.result().result
        
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action succeed!')
            self.get_logger().info('Result: {0}'.format(result.sequence))
            
            rclpy.shutdown()
```

The function above is a function that receives results.

```python
def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

```

The last function defined is a function that receives feedback from the Action Server from time to time.

---
