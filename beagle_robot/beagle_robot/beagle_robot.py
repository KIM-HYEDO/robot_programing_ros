from beagle_msgs.msg import BuzzerControl, LidarData, MotorControl, CameraDetection, ServoControl
from beagle_msgs.srv import RobotMotor
from rcl_interfaces.msg import SetParametersResult
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from roboid import *


class BeagleRobot(Node):

    def __init__(self):
        super().__init__('beagle_robot') # 노드 이름 저장
        self.beagle = Beagle()
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초마다 타이머 콜백 함수 호출
        qos_profile = QoSProfile(depth=10) # 퍼블리시 할 데이터를 버퍼에 10개 저장

        self.motor_go_publisher = self.create_publisher(
            MotorControl, '/motor_control', qos_profile)
        self.buzzer_publisher = self.create_publisher(
            BuzzerControl, '/buzzer_control', qos_profile)
        self.servo_publisher = self.create_publisher(
            ServoControl, '/servo_control', qos_profile)
        
        self.lidar_subscriber = self.create_subscription(
            LidarData,
            '/lidar_data',
            self.lidar_callback,
            qos_profile)
        self.camera_subscriber = self.create_subscription(
            CameraDetection,
            '/camera_detection',
            self.camera_callback,
            qos_profile)    
        self.encoder_subscriber = self.create_subscription(
            MotorControl,
            'motor_control',
            self.encoder_callback,
            qos_profile)

    def ConnectBeagle(self):
        self.beagle = Beagle()

    def ResetBeagle(self):
        self.beagle.reset()

    def DisConnectBeagle(self):
        self.beagle.dispose()

    def BuzzerControl(self, hz, sec):
        self.beagle.buzzer(hz)
        wait(sec * 1000)

    def LidarData(self):
        self.beagle.start_lidar()
        self.beagle.wait_until_lidar_ready()
        print('lidar is ready')
        self.beagle.lidar_chart()
        wait(-1)
    
    def encoder_callback(self,msg):
        encoder_l = self.beagle.left_encoder()
        encoder_r = self.beagle.right_encoder()

    def MotorControl(self, request, response):#request와 response의 형태로
        motor_l = request.motor_l
        motor_r = request.motor_r
        sec = request.sec
        
        self.beagle.wheels(motor_l, motor_r)
        wait(sec * 1000)

        encoder_l = self.beagle.left_encoder()
        encoder_r = self.beagle.right_encoder()
        
        response.encoder_l = encoder_l
        response.encoder_r = encoder_r

        return response

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

    def timer_callback(self):
        self.Motor_Control()
        self.motor_timer.cancel()
    
    def start_motor(self, motor_l, motorR, sec):
        self.Motor_Control()
        self.motor_timer = self.create_timer(sec, self.timer_callback)

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

    subscription = node.

if __name__ == '__main__':
    main()


#서비스로 엔코더값 보내기
