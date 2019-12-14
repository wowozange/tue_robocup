from sensor_msgs.msg import Image
import smach
import math
import rospy


class ShowOpenPose(smach.State):
    def __init__(self, robot, timeout = 5.0):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        self._image_openpose = None
        self._openpose_subscriber = rospy.Subscriber('openpose/result_image', Image, self._callback)
        self._timeout = timeout

    def execute(self, userdata):
        # call openpose service
        if self._image_openpose:
            self._robot.hmi.show_image_from_msg(self._image_openpose, self._timeout)
        return "done"

    def _callback(self, data):
        rospy.logdebug('Received openpose result image')
        self._image_openpose = data
