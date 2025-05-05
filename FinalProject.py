# Mary Cottier
# 5-5-25
import rospy
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import OverrideRCIn

class FinalProject:
    def __init__(self):
        # info variables
        self.apriltag_data = None
        self.apriltag_detection = False

        # nodes & services
        rospy.init_node('final_project_node', anonymous=True)
        rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
        rospy.wait_for_service('/minihawk_SIM/mavros/cmd/arming')

        # service calls
        self.set_mode = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        self.arm = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)

        try:
            self.set_mode(0, 'AUTO')
            self.arm(True)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')

        rospy.Subscriber('/minihawk_SIM/MH_usb_camera_link_optical/tag_detections', AprilTagDetectionArray, self.apriltag_callback)
        while not self.apriltag_detection and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.sleep(3)

        self.finetune_position()

        try:
            self.set_mode(0, 'QLAND')
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')

    def apriltag_callback(self, msg):
        if msg.detections:
            self.apriltag_data = msg.detections[0]
            self.apriltag_detection = True

    # sets to qloiiter
    def finetune_position(self):
        try:
            self.set_mode(0, 'QLOITER')
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')

        pub = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size=10)
        Kp, Ki, Kd = 10.0, 0.1, 5.0 # pid variables
        ix = iy = ex_prev = ey_prev = 0.0
        last_time = rospy.Time.now().to_sec()

        # position adjustments
        while not rospy.is_shutdown():
            if not self.apriltag_data: # take data
                continue

            pos = self.apriltag_data.pose.pose.position
            dx, dy = pos.x, pos.y
            rospy.loginfo((dx, dy))

            if abs(dx) < 1 and abs(dy) < 1:
                break

            now = rospy.Time.now().to_sec()
            dt = max(now - last_time, 1e-3)
            last_time = now

            # roll
            iy += dy * dt
            dy_d = (dy - ey_prev) / dt
            roll = int(max(1000, min(2000, 1500 + Kp * dy + Ki * iy + Kd * dy_d)))
            ey_prev = dy

            # pitch
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
