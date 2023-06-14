from beagle_msgs.msg import BuzzerControl, LidarData, MoterControl, CameraDetection, ServoControl
from rcl_interfaces.msg import SetParametersResult
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
        qos_profile = QoSProfile(depth=10) # 퍼블리시 할 데이터를 버퍼에 10개 저장

        self.moter_go_publisher = self.create_publisher(
            MoterControl, '/moter_control', qos_profile)
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

    def MoterControl(self, moter_l, moter_r, sec):#request와 response의 형태로
        self.beagle.wheels(moter_l, moter_r)
        
        encoderl = self.beagle.left_encoder()
        encoderr = self.beagle.right_encoder()
        
        responseL = 0#(requestL)
        responseR = 0#(requestR)

        wait(sec * 1000)

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

#서비스로 엔코더값 보내기
