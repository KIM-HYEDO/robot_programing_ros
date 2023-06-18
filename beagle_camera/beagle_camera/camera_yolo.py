import roboidai as ai
import rclpy
import sys
import argparse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from beagle_msgs.msg import CameraDetection, Rect


class camera_yolo(Node):

    def __init__(self,camera_view,yolo_use):
        super().__init__('camera_yolo')
        qos_profile = QoSProfile(depth=10)
        self.camera_publisher = self.create_publisher(
            CameraDetection, '/camera_detection', qos_profile)
        self.count = 0
        self.cam = ai.Camera('ip0')
        self.yolo_use=yolo_use=='True'
        self.camera_view=camera_view=='True'
        if self.yolo_use:
            self.dt = ai.ObjectDetector(multi=True, lang='en')
            self.dt.download_model()
            self.dt.load_model()
            self.labels = {"person": 0, "bicycle": 1, "car": 2, "motorcycle": 3, "airplane": 4, "bus": 5, "train": 6, "truck": 7, "boat": 8, "traffic light": 9, "fire hydrant": 10, "stop sign": 11, "parking meter": 12, "bench": 13, "bird": 14, "cat": 15, "dog": 16, "horse": 17, "sheep": 18, "cow": 19, "elephant": 20, "bear": 21, "zebra": 22, "giraffe": 23, "backpack": 24, "umbrella": 25, "handbag": 26, "tie": 27, "suitcase": 28, "frisbee": 29, "skis": 30, "snowboard": 31, "sports ball": 32, "kite": 33, "baseball bat": 34, "baseball glove": 35, "skateboard": 36, "surfboard": 37, "tennis racket": 38,
                        "bottle": 39, "wine glass": 40, "cup": 41, "fork": 42, "knife": 43, "spoon": 44, "bowl": 45, "banana": 46, "apple": 47, "sandwich": 48, "orange": 49, "broccoli": 50, "carrot": 51, "hot dog": 52, "pizza": 53, "donut": 54, "cake": 55, "chair": 56, "couch": 57, "potted plant": 58, "bed": 59, "dining table": 60, "toilet": 61, "tv": 62, "laptop": 63, "mouse": 64, "remote": 65, "keyboard": 66, "cell phone": 67, "microwave": 68, "oven": 69, "toaster": 70, "sink": 71, "refrigerator": 72, "book": 73, "clock": 74, "vase": 75, "scissors": 76, "teddy bear": 77, "hair drier": 78, "toothbrush": 79}
        self.timer = self.create_timer(0.1, self.publish_camera_yolo)

    def publish_camera_yolo(self):
        msg = CameraDetection()
        image = self.cam.read()
        if self.yolo_use:
            if self.dt.detect(image):
                image = self.dt.draw_result(image)
                print(self.dt.get_label())
                print(self.dt.get_box())
            for box, label in zip(self.dt.get_box(), self.dt.get_label()):
                rect = Rect()
                rect.data = box
                msg.data[self.labels[label]].rect.append(rect)
            self.camera_publisher.publish(msg)
        if self.camera_view:
            self.cam.show(image)
        key = self.cam.check_key()


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-C', '--camera_use', type=str,
                        default='False', help='camera use mode')
    parser.add_argument('-CY', '--yolo_use', type=str,
                        default='False', help='yolo use mode')
    parser.add_argument('-CV', '--camera_view', type=str,
                        default='False', help='camera view mode')
    args, unknown = parser.parse_known_args()
    if not args.camera_use=='True':
        return
    rclpy.init(args=argv)  # 초기화
    node = camera_yolo(args.camera_view,args.yolo_use)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
