# ROS
import smach

# TU/e Robotics
from robot_smach_states.utility import Initialize

class English(object):
    HI_MY_NAME_IS = "Hi, my name is {}"
    IM_A_SERVICE_ROBOT = "I am one of the service robots of the Eindhoven University of Technology"
    PURPOSE = "My purpose is to help people in domestic or care environments."
    IM_OMNIDIR = "I have an omnidirectional base instead of legs"
    EXPLAIN_BASE = "With this base, I can instantly move in any direction and turn around whenever I want"
    TWO_ARMS = "I have two arms. These have the dimensions and degrees of freedom of human arms"
    HUMAN_ARMS = "So I can move my arms just like you would move your arms"
    END_OF_ARMS = "At the end of my arms, I have two grippers with which I can grasp objects"
    GRIPPERS = "My grippers can be opened and closed when I need to."
    TORSO = "My arms are mounted on a moveable torso. This way, I can grasp higher and lower"
    HEAD = "As a head, I have a 3D camera. I use this to detect and recognize objects and people"
    CAMERA = "My 3D camera is mounted on top of my torso and I can move my camera just like a human head."
    TWO_LRFs = "Furthermore, I have two laser range finders to help me to see where I am"
    LRF_LOCS = "One laser is mounted on my torso and the other one is at the bottom of my base"
    MICROPHONE = "Finally, I have a microphone on my head so that I can hear what you are saying"
    END_OF_INTRO = "This was my introduction, I hope that you like what you see and please enjoy the presentation."

class Dutch(object):
    HI_MY_NAME_IS = "Hoi, mijn naam is {}"
    IM_A_SERVICE_ROBOT = "Ik ben een van de zorgrobots van de Technische Universiteit Eindhoven"
    PURPOSE = "Mijn doel is mensen te helpen in huis- en zorgomgevingen"
    IM_OMNIDIR = "In plaats van benen heb ik een omni-directioneel onderstel"
    EXPLAIN_BASE = "Met dit onderstel kan ik direct alle kanten op bewegen en omdraaien wanneer ik wil"
    TWO_ARMS = "Ik heb ook twee armen. Deze hebben dezelfde afmetingen en bewegingsmogelijkheden als mensenarmen"
    HUMAN_ARMS = "Dus ik kan mijn armen net zo bewegen als jij de jouwe"
    END_OF_ARMS = "Aan het eind van mijn armen zitten grijpers waarmee ik dingen kan vastpakken"
    GRIPPERS = "Mijn grijpers kunnen open en dicht als ik dat wil"
    TORSO = "Mijn armen zitten aan een beweegbare torso, zodat ik hoger en lager kan pakken"
    HEAD = "Als hoofd heb ik een 3D camera. Dat gebruik ik om mensen en dingen te herkennen"
    CAMERA = "Mijn hoofd zit bovenop mijn torso en ik kan rondkijken net als een mens"
    TWO_LRFs = "Verder heb ik twee lezer afstandsmeters, waarmee ik beter kan zien waar ik ben" # laser = lezer :-)
    LRF_LOCS = "1 lezer zit op mijn torso en de andere onderaan mijn onderstel"
    MICROPHONE = "Als laatste heb ik een microfoon waarmee ik kan horen wat mensen zeggen"
    END_OF_INTRO = "Dit was mijn introductie, ik hoop dat leuk vind wat je ziet en ik wens je nog een fijne dag"


class Presentation(smach.State):
    """ Smach state to have the robot present itself to an audience as a demo """
    def __init__(self, robot, language_name='en'):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=["done"])

        self.robot = robot
        self.language_name = language_name
        self.lang = {"en":English, "nl":Dutch}[language_name]

    def execute(self, userdata=None):
        """ Execute function

        :param userdata:
        :return:
        """
        # ToDo: here we can have the robot do awesome stuff
        # Introduction
        self.robot.speech.speak(self.lang.HI_MY_NAME_IS.format(self.robot.robot_name), language=self.language_name, block=True)
        self.robot.speech.speak(self.lang.IM_A_SERVICE_ROBOT, language=self.language_name, block=True)
        self.robot.speech.speak(self.lang.PURPOSE, language=self.language_name, block=True)

        # Base
        self.robot.speech.speak(self.lang.IM_OMNIDIR, language=self.language_name, block=True)
        self.robot.speech.speak(self.lang.EXPLAIN_BASE, language=self.language_name, block=False)
        self.robot.base.force_drive(0.1, 0, 0, 1.0)  # Forward
        self.robot.base.force_drive(0, 0.1, 0, 1.0)  # Left
        self.robot.base.force_drive(-0.1, 0, 0, 1.0)  # Backwards
        self.robot.base.force_drive(0, -0.1, 0, 1.0)  # Right
        self.robot.base.force_drive(0, 0, 1.0, 6.28)  # Turn around

        # Arms
        self.robot.speech.speak(self.lang.TWO_ARMS, language=self.language_name, block=False)
        self.robot.speech.speak(self.lang.HUMAN_ARMS, language=self.language_name, block=False)
        self.robot.leftArm.send_joint_trajectory("wave_front")
        self.robot.speech.speak(self.lang.END_OF_ARMS, language=self.language_name, block=False)
        self.robot.speech.speak(self.lang.GRIPPERS, language=self.language_name, block=False)
        self.robot.leftArm._send_joint_trajectory([[0, 0, 0, 1.7, 0, 0, 0]])
        self.robot.rightArm._send_joint_trajectory([[0, 0, 0, 1.7, 0, 0, 0]])
        self.robot.leftArm.send_gripper_goal("open")
        self.robot.leftArm.send_gripper_goal("close")
        self.robot.rightArm.send_gripper_goal("open")
        self.robot.rightArm.send_gripper_goal("close")
        self.robot.leftArm.reset()
        self.robot.rightArm.reset()

        # Torso
        self.robot.speech.speak(self.lang.TORSO, language=self.language_name, block=False)
        self.robot.torso.medium()
        self.robot.torso.wait_for_motion_done(5.0)
        self.robot.torso.reset()
        self.robot.torso.wait_for_motion_done(5.0)

        # Kinect
        self.robot.speech.speak(self.lang.HEAD, language=self.language_name, block=False)
        self.robot.rightArm.send_joint_trajectory("point_to_kinect")
        self.robot.speech.speak(self.lang.CAMERA, language=self.language_name, block=False)
        self.robot.head.look_at_hand("right")
        self.robot.head.wait_for_motion_done()
        self.robot.head.look_at_hand("left")
        self.robot.head.wait_for_motion_done()
        self.robot.head.reset()

        # Lasers
        self.robot.speech.speak(self.lang.TWO_LRFs, language=self.language_name, block=True)
        self.robot.speech.speak(self.lang.LRF_LOCS, language=self.language_name, block=False)
        self.robot.leftArm.send_joint_trajectory("point_to_laser")

        # Microphone
        self.robot.speech.speak(self.lang.MICROPHONE, language=self.language_name, block=True)

        # Final
        self.robot.speech.speak(self.lang.END_OF_INTRO, language=self.language_name, block=True)

        return "done"


class PresentationMachine(smach.StateMachine):
    def __init__(self, robot):
            """ Contains the Initialize state and the Presentation state
            :param robot: Robot to use
            :return:
            """
            smach.StateMachine.__init__(self, outcomes=["done", "aborted"])

            with self:
                smach.StateMachine.add("INITIALIZE", Initialize(robot=robot),
                                       transitions={"initialized": "PRESENT",
                                                    "abort": "aborted"})

                smach.StateMachine.add("PRESENT", Presentation(robot=robot),
                                       transitions={"done": "done"})

class PresentationMachineDutch(smach.StateMachine):
    def __init__(self, robot):
            """ Contains the Initialize state and the Presentation state
            :param robot: Robot to use
            :return:
            """
            smach.StateMachine.__init__(self, outcomes=["done", "aborted"])

            with self:
                smach.StateMachine.add("INITIALIZE", Initialize(robot=robot),
                                       transitions={"initialized": "PRESENT",
                                                    "abort": "aborted"})

                smach.StateMachine.add("PRESENT", Presentation(robot=robot, language_name="nl"),
                                       transitions={"done": "done"})
