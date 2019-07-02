# CLEAN UP KNOWLEDGE FILE ROBOTICS_TESTLABS
from robocup_knowledge import knowledge_loader
common = knowledge_loader.load_knowledge("common")

"""
Local knowledge info needed:
The room to search has to have enough (way)points to cover the complete area.
This means waypoints on the floor, and known (furniture) object points.
Exact coordinates of the locations are in ed_object_models.
"""

starting_point = "initial_pose"
starting_pose = "gpsr_meeting_point"

# required keys: entity_id (str), room_id (str), navigation_area (str), segment_areas (list)
cleaning_locations = [
    {'name': 'dinner_table',  'room': 'livingroom', 'navigation_area': 'in_front_of',   'segment_areas': ['on_top_of']},
    {'name': 'couch_table',   'room': 'livingroom', 'navigation_area': 'in_front_of',   'segment_areas': ['on_top_of']},
    {'name': 'cabinet',       'room': 'kitchen',    'navigation_area': 'in_front_of',   'segment_areas': ['on_top_of']},
    {'name': 'hallway_table', 'room': 'hallway',    'navigation_area': 'in_front_of',   'segment_areas': ['on_top_of']}
]

grammar_target = "T"

grammar = ""
for room in common.rooms:
    grammar += "\nT[{0}] -> {0}".format(room)