# ROS
from threading import Event

import message_filters
import rospy
import smach
import std_msgs.msg
from sensor_msgs.msg import CameraInfo, Image

TRIGGER_TOPIC = "/trigger"


class LightSaber(smach.State):
    def __init__(self, robot):
        """
        State that registers an image subscriber and continuously calls the people detection service. As soon as a
        trigger is received over a topic, the state exits

        :param robot: (Robot) api object
        """
        smach.State.__init__(self, outcomes=["done"])

        self._robot = robot
        self._ts = None
        self._event = Event()
        self._camera_base_ns = "{}/head_rgbd_sensor".format(robot.robot_name)

    def execute(self, ud=None):

        self._robot.head.reset()

        self._robot.speech.speak("Let's show what I can do")
        self._event = Event()
        self._register_subscribers()
        rate = rospy.Rate(10.0)
        rospy.loginfo("Starting main loop")
        while not rospy.is_shutdown() and not self._event.is_set():
            rate.sleep()

        self._deregister_subscribers()

    def _image_callback(self, rgb, depth, depth_info):
        """
        Callback for received image data. Uses image data to call people detection service.

        :param rgb:
        :param depth:
        :param depth_info:
        """
        try:
            rospy.logdebug('Received rgb, depth, cam_info')
            image_data = (rgb, depth, depth_info)
            if any(image_data):
                self._robot.perception.detect_person_3d(rgb, depth, depth_info)
        except Exception as e:
            rospy.logerr(e)

    def _trigger_callback(self, _):
        """
        Callback for the trigger topic. Sets the event to signal the main loop to stop.
        """
        self._event.set()

    def _register_subscribers(self):
        """
        Registers the image subscribers. N.B.: this is based on robot_skills.perception.
        """
        rospy.loginfo("Registering image subscribers")
        # camera topics
        self._depth_info_sub = message_filters.Subscriber(
            '{}/depth_registered/camera_info'.format(self._camera_base_ns), CameraInfo)
        self._depth_sub = message_filters.Subscriber(
            '{}/depth_registered/image'.format(self._camera_base_ns), Image)
        self._rgb_sub = message_filters.Subscriber(
            '{}/rgb/image_raw'.format(self._camera_base_ns), Image)

        self._ts = message_filters.ApproximateTimeSynchronizer([self._rgb_sub, self._depth_sub, self._depth_info_sub],
                                                               queue_size=1,
                                                               slop=10)

        self._ts.registerCallback(self._image_callback)

        # trigger sub
        self._trigger_sub = rospy.Subscriber(TRIGGER_TOPIC, std_msgs.msg.Empty, self._trigger_callback, queue_size=1)

    def _deregister_subscribers(self):
        """
        Clears the callbacks and deletes the subcribers
        """
        rospy.loginfo("Deregistering image subscribers")
        self._ts.callbacks.clear()
        del self._ts, self._depth_info_sub, self._depth_sub, self._rgb_sub
        rospy.loginfo("Deregistration done")
