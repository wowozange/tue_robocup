"""
Contains a getter function for a singleton with the hsrb interface
"""
from .hsrb_interface_settings import update_hsrb_settings
from .hsrb_interface_joint_group import update_joint_group

_HSRB_INTERFACE = None


def _construct_robot_interface():
    global _HSRB_INTERFACE
    update_hsrb_settings()
    update_joint_group()
    from hsrb_interface import Robot
    _HSRB_INTERFACE = Robot()


def get_hsrb_interface():
    """
    Returns an instance of the hsrb_robot.robot.Robot class

    To do so, some changes are made to the hsrb_interface Python module and subsequently the Robot object is
    constructed.
    """
    if _HSRB_INTERFACE is None:
        _construct_robot_interface()

    return _HSRB_INTERFACE
