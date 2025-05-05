# Mary Cottier
# 5-5-25
import rospy
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import OverrideRCIn

class FinalProject:
    def __init__(self):
        self.apriltag_data = None
        self.apriltag_detection = False

        # Initialize ROS node and wait for services
        rospy.init_node('final_project_node', anonymous=True)
        rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
        rospy.wait_for_service('/minihawk_SIM/mavros/cmd/arming')

        self.set_mode_service = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)

        # Set AUTO mode and arm
        self.set_mode('AUTO')
        self.arm_motors()

        # Start AprilTag detection
        rospy.Subscriber('/minihawk_SIM/MH_usb_camera_link_optical/tag_detections',
                         AprilTagDetectionArray, self.apriltag_callback)

        # Wait for tag detection
        while not self.apriltag_detection and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.sleep(3)  # stabilize before PID

        # Adjust position
        self.finetune_position()

        # Land after alignment
        self.set_mode('QLAND')

    def set_mode(self, mode):
        try:
            self.set_mode_service(0, mode)
            print(f'Mode set to {mode}')
        except rospy.ServiceException as e:
            print(f'Error setting mode to {mode}: {e}')

    def arm_motors(self):
        try:
            self.arm_service(True)
            print('Motors armed.')
        except rospy.ServiceException as e:
            print(f'Error arming motors: {e}')

    def apriltag_callback(self, msg):
        if msg.detections:
            self.apriltag_data = msg.detections[0]
            self.apriltag_detection = True

    def get_position(self):
        pose = self.apriltag_data.pose
        while hasattr(pose, "pose"):
            pose = pose.pose
        return pose.position

    def finetune_position(self):
        self.set_mode('QLOITER')
        pub = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size=10)

        Kp, Ki, Kd = 10.0, 0.1, 5.0
        ix = iy = ex_prev = ey_prev = 0.0
        last_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if not self.apriltag_data:
                continue

            pos = self.get_position()
            dx, dy = pos.x, pos.y
            print(f'AprilTag offset: dx={dx:.2f}, dy={dy:.2f}')

            if abs(dx) < 1 and abs(dy) < 1:
                print('Target aligned, initiating landing.')
                break

            now = rospy.Time.now().to_sec()
            dt = max(now - last_time, 1e-3)
            last_time = now

            # PID Roll (Y-axis)
            iy += dy * dt
            dy_d = (dy - ey_prev) / dt
            roll = int(max(1000, min(2000, 1500 + Kp * dy + Ki * iy + Kd * dy_d)))
            ey_prev = dy

            # PID Pitch (X-axis)
            ix += dx * dt
            dx_d = (dx - ex_prev) / dt
            pitch = int(max(1000, min(2000, 1500 + Kp * dx + Ki * ix + Kd * dx_d)))
            ex_prev = dx

            control = OverrideRCIn()
            control.channels = [roll, pitch, 1500, 1500] + [0]*14
            pub.publish(control)

            rospy.sleep(0.5)

if __name__ == '__main__':
    FinalProject()
