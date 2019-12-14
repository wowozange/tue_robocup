import rospy
import PyKDL as kdl
from robot_skills.hero import Hero
from robot_skills.util.kdl_conversions import FrameStamped

if __name__ == "__main__":

    rospy.init_node("motion_test_node")
    hero = Hero()
    arm = hero.arms.values()[0]
    goal_pose = kdl.Frame(kdl.Vector(0.5, 0.0, 0.5))
    goal = FrameStamped(goal_pose, "base_link")
    arm.send_goal(goal)
