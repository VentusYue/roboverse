TRAIN_CONTAINERS = [
    'plate',
    'cube_concave',
    'table_top',
    'bowl_small',
    'tray',
    'open_box',
    'cube',
    'torus',
]

TEST_CONTAINERS = [
    'pan_tefal',
    'marble_cube',
    'basket',
    'checkerboard_table',
]

CONTAINER_CONFIGS = {
    'plate': {
        'container_position_low': (.50, 0.22, -.30),
        'container_position_high': (.70, 0.26, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.46,
        'container_position_z': -0.37,
        'place_success_height_threshold': -0.32,
        'place_success_radius_threshold': 0.04,
        'min_distance_from_object': 0.11,
    },
    'cube_concave': {
        'container_name': 'cube_concave',
        'container_position_low': (.50, 0.22, -.30),
        'container_position_high': (.70, 0.26, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.06,
        'container_position_z': -0.35,
        'place_success_height_threshold': -0.23,
        'place_success_radius_threshold': 0.04,
        'min_distance_from_object': 0.11,
    },
    'table_top': {
        'container_position_low': (.50, 0.22, -.30),
        'container_position_high': (.70, 0.26, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.13,
        'container_position_z': -0.37,
        'place_success_height_threshold': -0.32,
        'place_success_radius_threshold': 0.05,
        'min_distance_from_object': 0.11,
    },
    'bowl_small': {
        'container_position_low': (.5, 0.26, -.30),
        'container_position_high': (.7, 0.26, -.30),
        'container_position_z': -0.35,
        'place_success_height_threshold': -0.32,
        'place_success_radius_threshold': 0.04,
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.07,
        'min_distance_from_object': 0.11,
    },
    'tray': {
        'container_position_low': (.5, 0.25, -.30),
        'container_position_high': (.7, 0.25, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.18,
        'container_position_z': -0.37,
        'place_success_height_threshold': -0.32,
        'place_success_radius_threshold': 0.04,
        'min_distance_from_object': 0.11,
    },
    'open_box': {
        'container_position_low': (.5, 0.23, -.30),
        'container_position_high': (.7, 0.23, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.1,
        'container_position_z': -0.35,
        'place_success_height_threshold': -0.32,
        'place_success_radius_threshold': 0.04,
        'min_distance_from_object': 0.11,
    },
    'pan_tefal': {
        'container_position_low': (.50, 0.22, -.30),
        'container_position_high': (.70, 0.24, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.4,
        'container_position_z': -0.37,
        'place_success_height_threshold': -0.32,
        'place_success_radius_threshold': 0.04,
        'min_distance_from_object': 0.1,
    },
    'husky': {
        'container_position_low': (.50, 0.22, -.30),
        'container_position_high': (.70, 0.26, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.13,
        'container_position_z': -0.35,
        'place_success_height_threshold': -0.23,
        'place_success_radius_threshold': 0.04,
        'min_distance_from_object': 0.10,
    },
    'marble_cube': {
        'container_position_low': (.50, 0.22, -.30),
        'container_position_high': (.70, 0.26, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.07,
        'container_position_z': -0.35,
        'place_success_height_threshold': -0.23,
        'place_success_radius_threshold': 0.04,
        'min_distance_from_object': 0.10,
    },
    'basket': {
        'container_name': 'basket',
        'container_position_low': (.50, 0.22, -.30),
        'container_position_high': (.70, 0.26, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 1.68,
        'container_position_z': -0.37,
        'place_success_height_threshold': -0.28,
        'place_success_radius_threshold': 0.04,
        'min_distance_from_object': 0.11,
    },
    'checkerboard_table': {
        'container_name': 'checkerboard_table',
        'container_position_low': (.50, 0.22, -.30),
        'container_position_high': (.70, 0.26, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.08,
        'container_position_z': -0.37,
        'place_success_height_threshold': -0.23,
        'place_success_radius_threshold': 0.05,
        'min_distance_from_object': 0.11,
    },
    'torus': {
        'container_position_low': (.50, 0.22, -.30),
        'container_position_high': (.70, 0.26, -.30),
        'container_orientation': (1, 1, 1, 1),
        'container_scale': 0.15,
        'container_position_z': -0.37,
        'place_success_height_threshold': -0.32,
        'place_success_radius_threshold': 0.04,
        'min_distance_from_object': 0.1,
    },
    'cube': {
        'container_position_low': (.5, 0.22, -.30),
        'container_position_high': (.7, 0.24, -.30),
        'container_orientation': (0, 0, 0.707107, 0.707107),
        'container_scale': 0.05,
        'container_position_z': -0.35,
        'place_success_radius_threshold': 0.03,
        'place_success_height_threshold': -0.23,
        'min_distance_from_object': 0.1,
    }
}

TRAIN_OBJECTS = [
    'conic_cup',
    'ball',
    'sack_vase',
    'fountain_vase',
    'shed',
    'circular_table',
    'hex_deep_bowl',
    'smushed_dumbbell',
    'square_prism_bin',
    'narrow_tray',
    # New objects:
    'colunnade_top',
    'stalagcite_chunk',
    'bongo_drum_bowl',
    'pacifier_vase',
    'beehive_funnel',
    'crooked_lid_trash_can',
    'double_l_faucet',
    'toilet_bowl',
    'pepsi_bottle',
    'two_handled_vase',
    'tongue_chair',
    'oil_tanker',
    'thick_wood_chair',
    'modern_canoe',
    'pear_ringed_vase',
    'short_handle_cup',
    'curved_handle_cup',
    'bullet_vase',
    'glass_half_gallon',
    'flat_bottom_sack_vase',
    'teepee',
    'trapezoidal_bin',
    'vintage_canoe',
    'bathtub',
    'flowery_half_donut',
    't_cup',
    'cookie_circular_lidless_tin',
    'box_sofa',
    'baseball_cap',
    'two_layered_lampshade',
]

GRASP_TRAIN_OBJECTS = [
    'conic_cup',
    'fountain_vase',
    'circular_table',
    'hex_deep_bowl',
    'smushed_dumbbell',
    'square_prism_bin',
    'narrow_tray',
    'colunnade_top',
    'stalagcite_chunk',
    'bongo_drum_bowl',
    'pacifier_vase',
    'beehive_funnel',
    'crooked_lid_trash_can',
    'toilet_bowl',
    'pepsi_bottle',
    'tongue_chair',
    'modern_canoe',
    'pear_ringed_vase',
    'short_handle_cup',
    'bullet_vase',
    'glass_half_gallon',
    'flat_bottom_sack_vase',
    'trapezoidal_bin',
    'vintage_canoe',
    'bathtub',
    'flowery_half_donut',
    't_cup',
    'cookie_circular_lidless_tin',
    'box_sofa',
    'two_layered_lampshade',
    'conic_bin',
    'jar',
    'bunsen_burner',
    'long_vase',
    'ringed_cup_oversized_base',
    'aero_cylinder',
]

PICK_PLACE_TRAIN_OBJECTS = [
    'conic_cup',
    'fountain_vase',
    'circular_table',
    'hex_deep_bowl',
    'smushed_dumbbell',
    'square_prism_bin',
    'narrow_tray',
    'colunnade_top',
    'stalagcite_chunk',
    'bongo_drum_bowl',
    'pacifier_vase',
    'beehive_funnel',
    'crooked_lid_trash_can',
    'toilet_bowl',
    'pepsi_bottle',
    'tongue_chair',
    'modern_canoe',
    'pear_ringed_vase',
    'short_handle_cup',
    'bullet_vase',
    'glass_half_gallon',
    'flat_bottom_sack_vase',
    'trapezoidal_bin',
    'vintage_canoe',
    'bathtub',
    'flowery_half_donut',
    't_cup',
    'cookie_circular_lidless_tin',
    'box_sofa',
    'two_layered_lampshade',
    'conic_bin',
    'jar',
    'aero_cylinder',
]

OBJECT_SCALINGS = {
    'conic_cup': 0.6,
    'ball': 1.0,
    'sack_vase': 0.6,
    'fountain_vase': 0.4,
    'shed': 0.6,
    'circular_table': 0.4,
    'hex_deep_bowl': 0.4,
    'smushed_dumbbell': 0.6,
    'square_prism_bin': 0.7,
    'narrow_tray': 0.35,
    # New objects:
    'colunnade_top': 0.5,
    'stalagcite_chunk': 0.6,
    'bongo_drum_bowl': 0.5,
    'pacifier_vase': 0.5,
    'beehive_funnel': 0.6,
    'crooked_lid_trash_can': 0.5,
    'double_l_faucet': 0.6,
    'toilet_bowl': 0.4,
    'pepsi_bottle': 0.65,
    'two_handled_vase': 0.45,

    'tongue_chair': 0.5,
    'oil_tanker': 1.0,
    'thick_wood_chair': 0.4,
    'modern_canoe': 0.9,
    'pear_ringed_vase': 0.65,
    'short_handle_cup': 0.5,
    'curved_handle_cup': 0.5,
    'bullet_vase': 0.6,
    'glass_half_gallon': 0.6,
    'flat_bottom_sack_vase': 0.5,

    'teepee': 0.7,
    'trapezoidal_bin': 0.4,
    'vintage_canoe': 1.0,
    'bathtub': 0.4,
    'flowery_half_donut': 0.5,
    't_cup': 0.5,
    'cookie_circular_lidless_tin': 0.5,
    'box_sofa': 0.4,
    'baseball_cap': 0.5,
    'two_layered_lampshade': 0.6,

    'conic_bin': 0.4,
    'jar': 0.8,
    'gatorade': 0.7,
    'bunsen_burner': 0.6,
    'long_vase': 0.5,
    # New objects:
    'ringed_cup_oversized_base': 0.5,
    'square_rod_embellishment': 0.6,
    'elliptical_capsule': 0.6,
    'aero_cylinder': 0.5,
    'grill_trash_can': 0.5,
}

TEST_OBJECTS = [
    'conic_bin',
    'jar',
    'gatorade',
    'bunsen_burner',
    'long_vase',
    # New objects:
    'ringed_cup_oversized_base',
    'square_rod_embellishment',
    'elliptical_capsule',
    'aero_cylinder',
    'grill_trash_can',
]

GRASP_TEST_OBJECTS = [
    'square_rod_embellishment',
    'grill_trash_can',
    'shed',
    'sack_vase',
    'two_handled_vase',
    'thick_wood_chair',
    'curved_handle_cup',
    'baseball_cap',
    'elliptical_capsule',
]

PICK_PLACE_TEST_OBJECTS = [
    'square_rod_embellishment',
    'grill_trash_can',
    'shed',
    'sack_vase',
    'two_handled_vase',
    'thick_wood_chair',
    'curved_handle_cup',
    'baseball_cap',
    'elliptical_capsule',
]

OBJECT_ORIENTATIONS = {
    'conic_cup': (0, 0, 1, 0),
    'ball': (0, 0, 1, 0),
    'sack_vase': (0, 0.707, 0.707, 0),
    'fountain_vase': (0, 0, 1, 0),
    'shed': (0, 0, 1, 0),
    'circular_table': (0, 0, 1, 0),
    'hex_deep_bowl': (0, 0, 1, 0),
    'smushed_dumbbell': (0, 0, 1, 0),
    'square_prism_bin': (0, 0, 1, 0),
    'narrow_tray': (0, 0, 1, 0),
    # New objects:
    'colunnade_top': (0, 0, 1, 0),
    'stalagcite_chunk': (0, 0, 1, 0),
    'bongo_drum_bowl': (0, 0.707, 0.707, 0),
    'pacifier_vase': (0, 0, 1, 1),
    'beehive_funnel': (0, 0, 1, 0),
    'crooked_lid_trash_can': (0, 0, 1, 0),
    'double_l_faucet': (0, 0.707, 0, 0.707),
    'toilet_bowl': (0, 0, 1, 0),
    'pepsi_bottle': (0, 0, 1, 0),
    'two_handled_vase': (0, 0, 1, 0),
    'tongue_chair': (0, 0, 1, 0),
    'oil_tanker': (0, 0, 0, 0),
    'thick_wood_chair': (0, 0, 1, 0),
    'modern_canoe': (0, 0.707, 0.707, 0),
    'pear_ringed_vase': (0, 0, 1, 0),
    'short_handle_cup': (0, 0, 1, 0),
    'curved_handle_cup': (0, 0.707, 0.707, 0),
    'bullet_vase': (0, 0, 1, 0),
    'glass_half_gallon': (0, 0, 1, 0),
    'flat_bottom_sack_vase': (0, 0, 1, 0),
    'teepee': (0, 0, 1, 0),
    'trapezoidal_bin': (0, 0, 1, 0),
    'vintage_canoe': (0, 0.707, 0.707, 0),
    'bathtub': (0, 0, 1, 0),
    'flowery_half_donut': (0, 0.707, 0.707, 0),
    't_cup': (0, 0.707, 0.707, 0),
    'cookie_circular_lidless_tin': (0, 0, 1, 0),
    'box_sofa': (0, 0, 1, 0),
    'baseball_cap': (0, -0.707, 0.707, 0),
    'two_layered_lampshade': (0, 0.707, 0.707, 0),

    'conic_bin': (0, 0.707, 0.707, 0),
    'jar': (0, 0.707, 0, 0.707),
    'gatorade': (0, 0, 1, 0),
    'bunsen_burner': (0, 0, 1, 0),
    'long_vase': (0, 0, 1, 0),
    # New objects:
    'ringed_cup_oversized_base': (0, 0.707, 0.707, 0),
    'square_rod_embellishment': (0, 0, 1, 0),
    'elliptical_capsule': (0.5, 0.5, 0.5, 0.5),
    'aero_cylinder': (0, 0, 1, 0),
    'grill_trash_can': (0, 0.707, 0.707, 0),
}

GRASP_OFFSETS = {
    'bunsen_burner': (0, 0.01, 0.0),
    'double_l_faucet': (0.01, 0.0, 0.0),
    'pear_ringed_vase': (0.0, 0.01, 0.0),
    'teepee': (0.0, 0.04, 0.0),
    'long_vase': (0.0, 0.03, 0.0),
    'ball': (0, 0, 0.0)
}
