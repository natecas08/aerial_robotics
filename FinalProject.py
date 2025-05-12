#!/usr/bin/env python
# Mary Cottier
# 5-5-25
import rospy
from mavros_msgs.srv import SetMode, CommandBool
from apriltag_ros.msg import AprilTagDetectionArray
from mavros_msgs.msg import OverrideRCIn

class FinalProject:
    def __init__(self):
        # info storage variable
        self.apriltag_data = None
        self.apriltag_detection = False

        # create nodes and services
        rospy.init_node('final_project_node', anonymous=True)
        rospy.wait_for_service('/minihawk_SIM/mavros/set_mode')
        rospy.wait_for_service('/minihawk_SIM/mavros/cmd/arming')

        # call services
        self.mode_service = rospy.ServiceProxy('/minihawk_SIM/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/minihawk_SIM/mavros/cmd/arming', CommandBool)

        self.set_mode('AUTO')
        self.arm_motors()

        rospy.Subscriber('/minihawk_SIM/MH_usb_camera_link_optical/tag_detections',
                         AprilTagDetectionArray, self.apriltag_callback)

        self.wait_for_apriltag()
        self.finetune_position()
        self.set_mode('QLAND')

    def set_mode(self, mode_name):
        try:
            self.mode_service(0, mode_name)
            print('Mode set to {}'.format(mode_name))
        except rospy.ServiceException as e:
            print('Errored while setting mode to {}: {}'.format(mode_name, e))

    def arm_motors(self):
        try:
            self.arm_service(True)
            print('Motors armed.')
        except rospy.ServiceException as e:
            print('Errored while arming: {}'.format(e))

    def apriltag_callback(self, msg):
        if msg.detections:
            self.apriltag_data = msg.detections[0]
            self.apriltag_detection = True

    def get_apriltag_position(self, pose):
        while hasattr(pose, "pose"):
            pose = pose.pose
        return pose.position

    def wait_for_apriltag(self):
        while not self.apriltag_detection and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.sleep(3)

    def finetune_position(self):
        self.set_mode('QLOITER')
        pub = rospy.Publisher('/minihawk_SIM/mavros/rc/override', OverrideRCIn, queue_size=10)

        # PID constants
        Kp, Ki, Kd = 10.0, 0.1, 5.0
        ix = iy = ex_prev = ey_prev = 0.0
        last_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if not self.apriltag_data:
                continue

            # Position
            pos = self.get_apriltag_position(self.apriltag_data.pose)
            dx, dy = pos.x, pos.y
            print('AprilTag offset: dx=%.2f, dy=%.2f' % (dx, dy))

            # Orientation (Quaternion)
            orientation = self.apriltag_data.pose.pose.pose.orientation
            print('AprilTag orientation: x=%.2f, y=%.2f, z=%.2f' % (
                orientation.x, orientation.y, orientation.z))

            if abs(dx) < 1.5 and abs(dy) < 1.5:
                print('Target aligned, initiating landing.')
                break

            now = rospy.Time.now().to_sec()
            dt = max(now - last_time, 1e-3)
            last_time = now

            # Roll (Y axis control)
            iy += dy * dt
            dy_d = (dy - ey_prev) / dt
            roll = int(max(1000, min(2000, 1500 + Kp * dy + Ki * iy + Kd * dy_d)))
            ey_prev = dy

            # Pitch (X axis control)
            ix += dx * dt
            dx_d = (dx - ex_prev) / dt
            pitch = int(max(1000, min(2000, 1500 + Kp * dx + Ki * ix + Kd * dx_d)))
            ex_prev = dx

            throttle = 1500
            yaw = 1500

            control = OverrideRCIn()
            control.channels = [roll, pitch, throttle, yaw] + [0] * 14
            pub.publish(control)

            print('Control signals: roll=%.2f, pitch=%.2f, throttle=%.2f, yaw=%.2f' %
                  (roll, pitch, throttle, yaw))

            rospy.sleep(0.5)


if __name__ == '__main__':
    FinalProject()
