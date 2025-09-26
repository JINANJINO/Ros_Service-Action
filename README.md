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
