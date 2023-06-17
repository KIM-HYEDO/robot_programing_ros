from roboid import *

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from beagle_msgs.msg import Forward
from beagle_msgs.srv import ConnectChecker

# beagle = Beagle()

# #비글 초기화
# beagle.reset()

# wait(5000)

class BeagleBringUp(Node):

    def __init__(self):
        super().__init__('beagle_bringup')
        #비글 객체
        self.beagle = Beagle()
        print(self.beagle.charge_state())
        print(self.beagle.battery_state())
        print(self.beagle.timestamp_basic())

        # timer_period sec마다 한번씩 돌게한다
        timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        qos_profile = QoSProfile(depth=10)

        #Publisher
        self.helloworld_publisher = self.create_publisher(String, 'helloworld', qos_profile)

        #Subscriber
        self.forward_sub = self.create_subscription(Forward,"toBeagle", self.ForwardCallback ,qos_profile)

        # srv-Service
        self.service_connect_checker = self.create_service(ConnectChecker,'connect_checker',self.checkConnection)


        # Variables
        self.leftEncoder = None
        self.rightEncoder = None

        self.isLidarReady = False
        self.scanData = None
        self.leftScanData = None
        self.rightScanData = None
        self.frontScanData = None
        self.rearScanData = None

    ## Init / Reset
    def checkConnection(self, req, res):
        print("Check")
        if req.try_connect:
            self.beagle = Beagle()
            num = self.beagle.timestamp_basic()
            if num==0:
                res.connected = False
            else: res.connected = True
            print(res)
            return res
        res.connected = False
        return res


    def ResetEncoder(self):
        self.beagle.reset_encoder()

    ## Setter

    # def timer_callback(self):
    #     self.moveFoward(0.1, 50)

    def moveFoward(self, time, speed) :
        self.beagle.move_forward(time,speed)

    def MoveWheel(self, speed_l, speed_r) :
        self.beagle.wheels(speed_l,speed_r)

    def MoveWheel(self, speed) :
        self.beagle.wheels(speed)

    def moveLeftWheel(self,speed) :
        self.beagle.left_wheel(speed)

    def moveRightWheel(self,speed) :
        self.beagle.right_wheel(speed)

    def StartLidar(self):
        self.beagle.start_lidar()

    def StopLidar(self):
        self.beagle.stop_lidar()

    def CutOff_Beagle(self):
        dispose()


    ## Callback Method

    def ForwardCallback(self,msg):
        self.get_logger().info('sub message: {0}'.format(msg.speed))
        self.MoveWheel(msg.speed)


    ## Getter

    def IsLidarReady(self):
        self.isLidarReady = self.beagle.is_lidar_ready()

    def Get_LeftEncoder(self):
        self.leftEncoder = self.beagle.left_encoder
        print("left encoder : " + self.leftEncoder)

    def Get_RightEncoder(self):
        self.rightEncoder = self.beagle.right_encoder
        print("right encoder : " + self.rightEncoder)

    def Get_Lidar(self):
        self.scanData = self.beagle.lidar()  # 0 ~ 65534

    def Get_LeftLidar(self):
        self.LeftScanData = self.beagle.left_lidar()  # 0 ~ 65534

    def Get_RightLidar(self):
        self.RightScanData = self.beagle.right_lidar()  # 0 ~ 65534

    def Get_FrontLidar(self):
        self.FrontScanData = self.beagle.front_lidar()  # 0 ~ 65534

    def Get_RearLidar(self):
        self.RightScanData = self.beagle.rear_lidar()  # 0 ~ 65534


def main(args=None):
    rclpy.init(args=args)
    node = BeagleBringUp()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
