
from beagle_msgs.msg import BuzzerControl, LidarData, MoterControl, CameraDetection, ServoControl
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class CodingNode(Node):

    def __init__(self):
        super().__init__('coding_node')
        qos_profile = QoSProfile(depth=10)
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

    def block_code(self):
        #code_start
        #middle_code_start
        msg = MoterControl()
        msg.mode = 0
        msg.data.append(10)
        msg.data.append(10)
        self.moter_go_publisher.publish(msg)
        msg = MoterControl()
        msg.mode = 1
        msg.data.append(10)
        msg.data.append(10)
        msg.data.append(10)
        self.moter_go_publisher.publish(msg)
        msg = MoterControl()
        msg.mode = 2
        msg.data.append(10)
        msg.data.append(10)
        self.moter_go_publisher.publish(msg)
        msg = MoterControl()
        msg.mode = 3
        msg.data.append(10)
        msg.data.append(10)
        self.moter_go_publisher.publish(msg)
        msg = MoterControl()
        msg.mode = 4
        msg.data.append(10)
        msg.data.append(10)
        self.moter_go_publisher.publish(msg)
        msg = ServoControl()
        msg.num = 1
        msg.data.append(0)
        self.servo_publisher.publish(msg)
        msg = ServoControl()
        msg.num = 1
        msg.data.append(0)
        msg.data.append(0)
        self.servo_publisher.publish(msg)
        msg = BuzzerControl()
        msg.data.append(0)
        msg.data.append(0)
        self.buzzer_publisher.publish(msg)
        rclpy.spin_once(self,timeout_sec=0.1)
        #middle_code_end
        #code_end

    def lidar_callback(self, lidar_msg):
        print('lidar')
        self.lidar = lidar_msg.data

    def camera_callback(self, camera_msg):
        print('camera')
        self.camera = []
        i=0
        for rects in camera_msg.rects:
            self.camera.append([])
            for rect in rects.rect:
                self.camera[i].append(rect.data)
            i+=1



def main(args=None):
    rclpy.init(args=args)
    node = CodingNode()
    try:
        node.block_code()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
