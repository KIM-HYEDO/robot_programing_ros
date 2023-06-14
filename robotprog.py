from roboid import *
#import roboidai as ai

def ConnectBeagle():
    beagle = Beagle()

def ResetBeagle():
    beagle.reset()

def DisConnectBeagle():
    beagle.dispose()

def BuzzerControl(hz, sec):
    beagle.buzzer(hz)
    wait(sec * 1000)

#def CameraDetection(list, cnt, pos):
    #while True:
        

def LidarData():
    beagle.start_lidar()
    beagle.wait_until_lidar_ready()
    print('lidar is ready')
    beagle.lidar_chart()
    wait(-1)

def MoterControl(left_speed, right_speed, sec):
    beagle.wheels(left_speed, right_speed)
    wait(sec * 1000)

def ServoControl(num, degree, speed):
    if num == 1:
        beagle.servo_speed_a(speed)
        beagle.servo_output_a_until_done(degree)
    elif num == 2:
        beagle.servo_speed_b(speed)
        beagle.servo_output_b_until_done(degree)
    elif num == 3:
        beagle.servo_speed_c(speed)
        beagle.servo_output_c_until_done(degree)
    
    print('servo_a : ', beagle.servo_input_a(),
          'servo_b : ', beagle.servo_input_b(),
          'servo_c : ', beagle.servo_input_c(), sep="\n")


#cam = ai.Camera('usb0')
#dt = ai.ObjectDetector(multi=True, lang='ko')
#dt.download_model()
#dt.load_model()

beagle = Beagle()

while True:


    dispose()