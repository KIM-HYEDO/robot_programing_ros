from beagle_msgs.msg import BuzzerControl, LidarData, MotorControl, CameraDetection, ServoControl
from beagle_msgs.srv import RobotMotor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from roboid import *


class BeagleRobot(Node):

    def __init__(self):
        super().__init__('beagle_robot') # 노드 이름 저장
        self.beagle = Beagle()
        qos_profile = QoSProfile(depth=10) # 퍼블리시 할 데이터를 버퍼에 10개 저장
       
        self.lidar_publisher = self.create_publisher(
            LidarData, '/lidar_data', qos_profile)

        self.buzzer_subscriber = self.create_subscription(
            BuzzerControl,
            'buzzer_control',
            self.BuzzerControl,#buzzer_callback,
            qos_profile)
        self.servo_subscriber = self.create_subscription(
            ServoControl,
            'servo_control',
            self.ServoControl,#servo_callback,
            qos_profile)
        
        self.encoder_srv = self.create_service(
            RobotMotor, '/encoder_callback', self.encoder_callback)        

    def ConnectBeagle(self):
        self.beagle = Beagle()
    def ResetBeagle(self):
        self.beagle.reset()
    def DisConnectBeagle(self):
        self.beagle.dispose()

    def LidarData(self):
        self.beagle.start_lidar()
        self.beagle.wait_until_lidar_ready()
        print('lidar is ready')
        self.beagle.lidar_chart()
        wait(-1)    

    def BuzzerControl(self, msg):
        if len(msg.data) >= 2:
            hz = msg.data[0]
            sec = msg.data[1]
        else:
            hz = msg.data[0]
            sec = 1
        self.beagle.buzzer(hz)
        wait(sec * 1000)

    def ServoControl(self, msg):
        if msg.num == 1:
            self.beagle.servo_output_a_until_done(msg.data[0])
            if len(msg.data) >= 2:
                self.beagle.servo_speed_a(msg.data[1])
            else:
                self.beagle.servo_speed_a(5)
        if msg.num == 1:
            self.beagle.servo_output_b_until_done(msg.data[0])
            if len(msg.data) >= 2:
                self.beagle.servo_speed_b(msg.data[1])
            else:
                self.beagle.servo_speed_b(5)
        if msg.num == 1:
            self.beagle.servo_output_c_until_done(msg.data[0])
            if len(msg.data) >= 2:
                self.beagle.servo_speed_c(msg.data[1])
            else:
                self.beagle.servo_speed_c(5)                

    def encoder_callback(self, request, response):
        response.encoder_l = self.beagle.left_encoder()
        response.encoder_r = self.beagle.right_encoder()
        self.get_logger().info('Incoming request\na: %d b: %d' 
        % (response.encoder_l, response.encoder_r))

        return response

    def MotorControl(self, request, response):#request와 response의 형태로
        motor_l = request.motor_l
        motor_r = request.motor_r
        self.beagle.wheels(motor_l, motor_r)

    def ServoControl(self, num, degree, speed):
        if num == 1:
            self.beagle.servo_speed_a(speed)
            self.beagle.servo_output_a_until_done(degree)
        elif num == 2:
            self.beagle.servo_speed_b(speed)
            self.beagle.servo_output_b_until_done(degree)
        elif num == 3:
            self.beagle.servo_speed_c(speed)
            self.beagle.servo_output_c_until_done(degree)

def main(args=None):
    rclpy.init(args=args) # 초기화
    node = BeagleRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node() # 노드 소멸
        rclpy.shutdown() # 함수 종료


if __name__ == '__main__':
    main()
