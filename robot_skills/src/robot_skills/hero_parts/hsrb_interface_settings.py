"""
This file contains 'fixes' for the hsrb_interface.settings module.
"""

import json
from hsrb_interface import settings as hsrb_settings

_HSRB_SETTINGS = """
{
    "robot": {
        "hsrb": {
            "fullname": "HSR-B"
        }
    },
    "frame": {
        "map": {
            "frame_id": "map"
        },
        "odom": {
            "frame_id": "odom"
        },
        "base": {
            "frame_id": "base_footprint"
        },
        "hand": {
            "frame_id": "hand_palm_link"
        }
    },
    "trajectory": {
            "impedance_control": "/hero/impedance_control",
            "constraint_filter_service":
                "/trajectory_filter/filter_trajectory_with_constraints",
            "timeopt_filter_service": "/hsrb/omni_base_timeopt_filter",
            "whole_timeopt_filter_service": "/hero/filter_hsrb_trajectory",
            "caster_joint": "base_roll_joint",
            "filter_timeout": 30.0,
            "action_timeout": 30.0,
            "watch_rate": 30.0
    },
    "joint_group": {
        "whole_body": {
            "class":                        ["joint_group", "JointGroup"],
            "joint_states_topic":           "/hero/joint_states",
            "arm_controller_prefix":        "/hero/arm_trajectory_controller",
            "head_controller_prefix":       "/hero/head_trajectory_controller",
            "hand_controller_prefix":       "/hero/gripper_controller",
            "omni_base_controller_prefix":  "/hero/omni_base_controller",
            "plan_with_constraints_service":"/plan_with_constraints",
            "plan_with_hand_goals_service": "/hero/plan_with_hand_goals",
            "plan_with_hand_line_service":  "/plan_with_hand_line",
            "plan_with_joint_goals_service":"/plan_with_joint_goals",
            "timeout":                       1.0,
            "end_effector_frames": [
                "hand_palm_link",
                "hand_l_finger_vacuum_frame"
            ],
            "rgbd_sensor_frame": "head_rgbd_sensor_link",
            "passive_joints": [
                "hand_r_spring_proximal_joint",
                "hand_l_spring_proximal_joint"
            ],
            "looking_hand_constraint": {
                "plugin_name": "hsrb_planner_plugins/LookHand",
                "use_joints": ["head_pan_joint", "head_tilt_joint"]
            },
            "motion_planning_joints": [
                "wrist_flex_joint",
                "wrist_roll_joint",
                "arm_roll_joint",
                "arm_flex_joint",
                "arm_lift_joint",
                "hand_motor_joint",
                "head_pan_joint",
                "head_tilt_joint"
            ]
        }
    },
    "end_effector": {
        "gripper": {
            "class":        ["end_effector", "Gripper"],
            "joint_names":  ["hand_motor_joint"],
            "prefix":       "/hsrb/gripper_controller",
            "left_finger_joint_name":  "hand_l_spring_proximal_joint",
            "right_finger_joint_name": "hand_r_spring_proximal_joint"
        },
        "suction": {
            "class": ["end_effector", "Suction"],
            "action":                         "/hsrb/suction_control",
            "suction_topic":                  "/hsrb/command_suction",
            "pressure_sensor_topic":          "/hsrb/pressure_sensor",
            "timeout":                        1.0
        }
    },
    "mobile_base": {
        "omni_base": {
            "class": ["mobile_base", "MobileBase"],
            "move_base_action":          "/move_base/move",
            "follow_trajectory_action":  "/hsrb/omni_base_controller",
            "pose_topic":                "/global_pose",
            "goal_topic":                "/base_goal",
            "timeout":                   1.0
        }
    },
    "camera": {
        "head_l_stereo_camera": {
            "class":   ["sensors", "Camera"],
            "prefix":  "/hsrb/head_l_stereo_camera",
            "timeout": 3.0
        },
        "head_r_stereo_camera": {
            "class": ["sensors", "Camera"],
            "prefix":  "/hsrb/head_l_stereo_camera",
            "timeout": 3.0
        },
        "head_rgbd_sensor_rgb": {
            "class": ["sensors", "Camera"],
            "prefix": "/hsrb/head_rgbd_sensor/rgb",
            "timeout": 3.0
        },
        "head_rgbd_sensor_depth": {
            "class": ["sensors", "Camera"],
            "prefix": "hsrb/head_rgbd_sensor/depth",
            "timeout": 3.0
        }
    },
    "imu": {
        "base_imu": {
            "class": ["sensors", "IMU"],
            "topic": "/hsrb/base_imu/data",
            "timeout": 1.0
        }
    },
    "force_torque": {
        "wrist_wrench": {
            "class": ["sensors", "ForceTorque"],
            "raw_topic": "/hsrb/wrist_wrench/raw",
            "compensated_topic": "/hsrb/wrist_wrench/compensated",
            "reset_service": "/hsrb/wrist_wrench/readjust_offset",
            "timeout": 1.0
        }
    },
    "lidar": {
        "base_scan": {
            "class": ["sensors", "Lidar"],
            "topic": "/hsrb/base_scan",
            "timeout": 1.0
        }
    },
    "object_detection": {
        "marker": {
            "class": ["object_detection", "ObjectDetector"],
            "topic": "/recognized_object"
        }
    },
    "power_supply": {
        "battery": {
            "class": ["battery", "Battery"],
            "topic": "/hsrb/battery_state",
            "timeout": 2.0
        }
    },
    "text_to_speech": {
        "default_tts": {
            "class": ["text_to_speech", "TextToSpeech"],
            "topic": "/talk_request"
        }
    },
    "collision_world": {
        "global_collision_world": {
            "class": ["collision_world", "CollisionWorld"],
            "service": "/get_collision_environment",
            "control_topic": "/known_object",
            "listing_topic": "/known_object_ids"
        }
    }
}
"""


def update_hsrb_settings():
    """
    Updates the (hardcoded) settings of the hsrb_interface.settings module.
    """
    hsrb_settings._SETTINGS = json.loads(_HSRB_SETTINGS)
