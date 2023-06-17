
from beagle_msgs.msg import BuzzerControl, LidarData,  CameraDetection, ServoControl
from beagle_msgs.srv import ConnectChecker
from beagle_msgs.action import MotorControl
from rcl_interfaces.msg import SetParametersResult
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class CodingNode(Node):

    def __init__(self):
        super().__init__('coding_node')
        self.lidar_data=[0 for i in range(360)]
        self.callback_group = ReentrantCallbackGroup()
        qos_profile = QoSProfile(depth=10)

        #모터 컨트롤 노드
        self.motor_control_client = ActionClient(
            self,
            MotorControl,
            '/motor_control', callback_group=self.callback_group)
        while not self.motor_control_client.wait_for_server(timeout_sec=1):
            self.get_logger().warning('wait open motor control node')

        #비글 로봇 체커 노드
        self.connect_check_cli = self.create_client(
            ConnectChecker, "/connect_checker")
        while not self.connect_check_cli.wait_for_service(timeout_sec=1):
            self.get_logger().warning('wait open beagle robot node')

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
        self.encoder_l = 0.0
        self.encoder_r = 0.0
        self.end_time = None

    def block_code(self):
        #시작전 비글 연결 됐는지 확인
        while rclpy.ok():
            if not self.beagle_robot_check():
                self.get_logger().warning('check beagle robot')
            else :
                break

        # code_start

        # code_end
        # stop
        if self.send_goal_motor_control(0, (0, 0)):
            while self.end_time is None and rclpy.ok():
                rclpy.spin_once(self)
                self.cancel_motor_control()
                break
            self.end_time = None
        else:
            return False
        return True

    def lidar_callback(self, lidar_msg):
        self.lidar_data = lidar_msg.data

    def camera_callback(self, camera_msg):
        self.camera = []
        i = 0
        for rects in camera_msg.rects:
            self.camera.append([])
            for rect in rects.rect:
                self.camera[i].append(rect.data)
            i += 1

    def cancel_motor_control(self):
        self.action_result_future.cancel()
        future_ = self.goal_handle.cancel_goal_async()
        future_.add_done_callback(self.goal_canceled_callback)
        rclpy.spin_until_future_complete(self, future_)

    def goal_canceled_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Cancelling of goal complete')
            self.cancel_complate = True
        else:
            self.get_logger().warning('Goal failed to cancel')

    def send_goal_motor_control(self, mode, data):
        wait_count = 1
        while not self.motor_control_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                self.get_logger().warning('motor control action server is not available.')
                return False
            wait_count += 1
        goal_msg = MotorControl.Goal()
        goal_msg.mode = mode
        goal_msg.data = tuple(map(float, data))
        self.send_goal_future = self.motor_control_client.send_goal_async(
            goal_msg,
            feedback_callback=self.motor_control_action_feedback)
        self.send_goal_future.add_done_callback(
            self.motor_control_action_goal)
        return True

    def motor_control_action_goal(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warning('Action goal rejected.')
            self.goal_handle = None  # 파괴된 핸들을 None으로 설정
            return
        # self.get_logger().info('Action goal accepted.')
        self.action_result_future = self.goal_handle.get_result_async()
        self.action_result_future.add_done_callback(
            self.motor_control_action_result)

    def motor_control_action_feedback(self, feedback_msg):
        self.encoder_l = feedback_msg.feedback.encoder_l
        self.encoder_r = feedback_msg.feedback.encoder_r

    def motor_control_action_result(self, future):
        if future.cancelled():
            self.get_logger().warning('future canceled')
            return
        action_status = future.result().status
        self.end_time = future.result().result.end_time
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            # self.get_logger().info('Action succeeded!')
            pass
        else:
            self.get_logger().warning(
                'Action failed with status: {0}'.format(action_status))

    def beagle_robot_check(self):
        srv_req = ConnectChecker.Request()
        srv_req.try_connect = True
        srv_future=self.connect_check_cli.call_async(srv_req)
        while not srv_future.done():
            rclpy.spin_once(self)
        srv_res=srv_future.result()

        if  srv_res is None:
            return False
        elif srv_res.connected == True:
            return True
        else :
            return False



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
