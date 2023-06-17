from beagle_msgs.msg import BuzzerControl, LidarData,  CameraDetection, ServoControl
from beagle_msgs.srv import RobotMotor, ConnectChecker
from rcl_interfaces.msg import SetParametersResult
import rclpy
import sys
import argparse
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from roboid import *


class BeagleRobot(Node):

    def __init__(self):
        super().__init__('beagle_robot')  # 노드 이름 저장

        while True:
            if self.beagle_check():
                break
            else:
                self.get_logger().warning('check connect beagle')
                wait(1000)

        qos_profile = QoSProfile(depth=10)  # 퍼블리시 할 데이터를 버퍼에 10개 저장

        self.declare_parameter('lidar_use', False)  # lidar 사용할 경우에만 퍼블리시
        self.lidar_use = self.get_parameter('lidar_use').value
        self.lidar_publisher = self.create_publisher(
            LidarData, '/lidar_data', qos_profile)
        if self.lidar_use:
            self.beagle.start_lidar()
            self.get_logger().info('ready lidar')
            self.beagle.wait_until_lidar_ready()
            self.lidar_pub_timer = self.create_timer(0.1, self.lidar_pub)
        self.add_on_set_parameters_callback(self.param_callback)
        self.callback_group = ReentrantCallbackGroup()

        self.buzzer_subscriber = self.create_subscription(
            BuzzerControl,
            '/buzzer_control',
            self.BuzzerControl,  # buzzer_callback,
            qos_profile)
        self.servo_subscriber = self.create_subscription(
            ServoControl,
            '/servo_control',
            self.ServoControl,  # servo_callback,
            qos_profile)
        self.lidar_publisher = self.create_publisher(
            LidarData, '/lidar_data', qos_profile)

        self.encoder_srv = self.create_service(
            RobotMotor, '/robot_motor', self.encoder_callback,
            callback_group=self.callback_group)
        self.service_connect_checker = self.create_service(
            ConnectChecker, '/connect_checker', self.checkConnection)

    def set_lidar_mode(self, mode):
        if mode == "zero" or mode == "trunc":
            self.beagle.lidar_mode(mode)
        else:
            self.beagle.lidar_mode("raw")

    def param_callback(self, params):
        for param in params:
            if param.name == 'lidar_use':
                pre_lidar_use = self.lidar_use
                self.lidar_use = param.value
                if not pre_lidar_use and self.lidar_use:
                    self.beagle.start_lidar()
                    self.get_logger().info('ready lidar')
                    self.beagle.wait_until_lidar_ready()
                    self.lidar_pub_timer = self.create_timer(
                        0.1, self.lidar_pub)
                elif pre_lidar_use and not self.lidar_mode:
                    self.lidar_pub_timer.cancel()
                    self.beagle.stop_lidar()
        return SetParametersResult(successful=True)

    def ConnectBeagle(self):
        self.beagle = Beagle()

    def checkConnection(self, req, res):  # 비글 연결 됐는지 확인
        if req.try_connect:
            res.connected = self.beagle_check()
            return res
        res.connected = False
        return res
    def beagle_check(self):
        self.beagle = Beagle()
        num = self.beagle.timestamp_basic()
        if num == 0:
            return False
        else:
            return True

    def ResetBeagle(self):
        self.beagle.reset()

    def DisConnectBeagle(self):
        self.beagle.dispose()

    def LidarData(self):
        self.beagle.start_lidar()
        self.beagle.wait_until_lidar_ready()
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
        wait(sec*1000.0)
        self.beagle.buzzer(0)

    def ServoControl(self, msg):
        if msg.num == 1:
            self.beagle.servo_output_a(msg.data[0])
            if len(msg.data) >= 2:
                self.beagle.servo_speed_a(msg.data[1])
            else:
                self.beagle.servo_speed_a(5)
        if msg.num == 1:
            self.beagle.servo_output_b(msg.data[0])
            if len(msg.data) >= 2:
                self.beagle.servo_speed_b(msg.data[1])
            else:
                self.beagle.servo_speed_b(5)
        if msg.num == 1:
            self.beagle.servo_output_c(msg.data[0])
            if len(msg.data) >= 2:
                self.beagle.servo_speed_c(msg.data[1])
            else:
                self.beagle.servo_speed_c(5)

    def encoder_callback(self, request, response):
        self.beagle.wheels(request.motor_l, request.motor_r)
        response.encoder_l = self.beagle.left_encoder()
        response.encoder_r = self.beagle.right_encoder()
        return response

    def MotorControl(self, request, response):  # request와 response의 형태로
        motor_l = request.motor_l
        motor_r = request.motor_r
        self.beagle.wheels(motor_l, motor_r)

    def lidar_pub(self):
        msg = LidarData()
        msg.data = list(map(float, self.beagle.lidar()))
        self.lidar_publisher.publish(msg)


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-L', '--lidar_mode', type=str,
                        default="raw", help='lidar mode set (raw, zero, trunc)')
    args, unknown = parser.parse_known_args()
    rclpy.init(args=argv)  # 초기화

    node = BeagleRobot()
    node.set_lidar_mode(args.lidar_mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown()  # 함수 종료


if __name__ == '__main__':
    main()
