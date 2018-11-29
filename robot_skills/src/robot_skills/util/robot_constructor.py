

def robot_constructor(robot_name):
    """Construct a robot by it's name. Choices are amigo, sergio, mockbot
    :param robot_name str of robot name. Current options are amigo, sergio, mockbot
    :returns a Robot-instance"""

    if robot_name == "amigo":
        import robot_skills.amigo
        return robot_skills.amigo.Amigo(wait_services=True)
    elif robot_name == "sergio":
        import robot_skills.sergio
        return robot_skills.sergio.Sergio(wait_services=True)
    elif robot_name == "mockbot":
        import robot_skills.mockbot
        return robot_skills.mockbot.Mockbot(wait_services=True)
    else:
        return None
