import rospy
import smach


class CheckTimeOut(smach.State):
    def __init__(self, robot, time_out_seconds):
        smach.State.__init__(self, outcomes=["not_yet", "time_out"])
        self.robot = robot
        self.time_out_seconds = time_out_seconds

        self.start = None

    def execute(self, userdata=None):
        current_seconds = rospy.Time.now().to_sec()

        if self.start is None:
            self.start = current_seconds

        dt = current_seconds - self.start

        if dt > self.time_out_seconds:
            return "time_out"

        return "not_yet"

    def reset(self):
        self.start = None
