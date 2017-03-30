starting_point = "initial_pose"

# required keys: entity_id (str), room_id (str), navigation_area (str), segment_areas (list)
inspection_places = [
    {"entity_id": "cabinet",
     "room_id": "kitchen",
     "navigate_area": "near",
     "segment_areas": ["on_top_of"]},

    {"entity_id": "dinner_table",
     "room_id": "livingroom",
     "navigate_area": "near",
     "segment_areas": ["on_top_of", "under"]},
]

known_types = [
        "beer",
        "bifrutas",
        "coke",
        "deodorant",
        "fanta",
        "ice tea",
        "mentos",
#        "sponge",
        "tea",
        "water"]
