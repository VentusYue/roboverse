import gym
import numpy as np
from roboverse.assets.shapenet_object_lists \
    import GRASP_TRAIN_OBJECTS, GRASP_TEST_OBJECTS, PICK_PLACE_TRAIN_OBJECTS, \
    PICK_PLACE_TEST_OBJECTS, TRAIN_CONTAINERS, TEST_CONTAINERS, PICK_PLACE_DEMO_CONTAINERS, PICK_PLACE_DEMO_OBJECTS

ENVIRONMENT_SPECS = (
    {
        'id': 'Widow250Grasp-v0',
        'entry_point': 'roboverse.envs.widow250:Widow250Env',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'target_object': 'beer_bottle',
                   'load_tray': True,
                   'xyz_action_scale': 0.2,
                   }
    },
    {
        'id': 'Widow250GraspEasy-v0',
        'entry_point': 'roboverse.envs.widow250:Widow250Env',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'target_object': 'shed',
                   'object_names': ('shed',),
                   'object_scales': (0.7,),
                   'load_tray': False,
                   'xyz_action_scale': 0.2,
                   'object_position_high': (.7, .3, -.30),
                   'object_position_low': (.7, .3, -.30),
                   }
    },
    {
        'id': 'Widow250MultiTaskGrasp-v0',
        'entry_point': 'roboverse.envs.widow250:Widow250Env',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_high': (.68, .25, -.30),
                   'object_position_low': (.53, .15, -.30),
                   'xyz_action_scale': 0.2,
                   }
    },
    {
        'id': 'Widow250MultiObjectGraspTrain-v0',
        'entry_point': 'roboverse.envs.widow250:Widow250MultiObjectEnv',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'possible_objects': GRASP_TRAIN_OBJECTS,
                   'num_objects': 2,

                   'load_tray': False,
                   'object_position_high': (.68, .25, -.30),
                   'object_position_low': (.53, .15, -.30),
                   'xyz_action_scale': 0.2,

                   }
    },
    {
        'id': 'Widow250MultiObjectGraspTest-v0',
        'entry_point': 'roboverse.envs.widow250:Widow250MultiObjectEnv',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',

                   'possible_objects': GRASP_TEST_OBJECTS,
                   'num_objects': 2,

                   'load_tray': False,
                   'object_position_high': (.68, .25, -.30),
                   'object_position_low': (.53, .15, -.30),
                   'xyz_action_scale': 0.2,

                   }
    },
    {
        'id': 'Widow250MultiThreeObjectGraspTrain-v0',
        'entry_point': 'roboverse.envs.widow250:Widow250MultiObjectEnv',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'possible_objects': GRASP_TRAIN_OBJECTS,
                   'num_objects': 3,

                   'load_tray': False,
                   'object_position_high': (.7, .25, -.30),
                   'object_position_low': (.5, .15, -.30),
                   'xyz_action_scale': 0.2,


                   # Next three entries are ignored
                   'object_names': ('beer_bottle', 'gatorade', 'shed'),
                   'object_scales': (0.7, 0.6, 0.8),
                   'object_orientations': (
                       (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),
                   }
    },
    {
        'id': 'Widow250MultiThreeObjectGraspTest-v0',
        'entry_point': 'roboverse.envs.widow250:Widow250MultiObjectEnv',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',

                   'possible_objects': GRASP_TEST_OBJECTS,
                   'num_objects': 3,

                   'load_tray': False,
                   'object_position_high': (.7, .25, -.30),
                   'object_position_low': (.5, .15, -.30),
                   'xyz_action_scale': 0.2,


                   # Next three entries are ignored
                   'object_names': ('beer_bottle', 'gatorade', 'shed'),
                   'object_scales': (0.7, 0.6, 0.8),
                   'object_orientations': (
                        (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),
                   }
    },
    {
        'id': 'Widow250SingleObjGrasp-v0',
        'entry_point': 'roboverse.envs.widow250:Widow250Env',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'object_names': ('shed',),
                   'object_scales': (0.7,),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_high': (.68, .25, -.30),
                   'object_position_low': (.53, .15, -.30),
                   'xyz_action_scale': 0.2,
                   }
    },
    # Pick and place environments
    {
        'id': 'Widow250PickPlaceEasy-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed',),
                   'object_scales': (0.7,),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.6, .2, -.3),
                   'object_position_high': (.6, .2, -.3),

                   'container_name': 'bowl_small',
                   'fixed_container_position': True,

                   }
    },
    {
        'id': 'Widow250PickPlace-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed',),
                   'object_scales': (0.7,),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.59, .27, -.30),

                   'container_name': 'plate',


                   }
    },
    {
        'id': 'Widow250PickPlaceMultiObjectMultiContainerTrain-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace'
                       ':Widow250PickPlaceMultiObjectMultiContainerEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': False,
                   'num_objects': 2,

                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,
                   'possible_containers': TRAIN_CONTAINERS,

                   }
    },
    {
        'id': 'Widow250PickPlaceMultiObjectMultiContainerTest-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace'
                       ':Widow250PickPlaceMultiObjectMultiContainerEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': False,
                   'num_objects': 2,

                   'possible_objects': PICK_PLACE_TEST_OBJECTS,
                   'possible_containers': TEST_CONTAINERS,
                   }
    },


    {
        'id': 'Widow250PickTray-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed',),
                   'object_scales': (0.7,),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.75, .27, -.30),

                   'container_name': 'tray',
                   'fixed_container_position': True,

                   'use_neutral_action': True,
                   'neutral_gripper_open': False,
                   }
    },
    {
        'id': 'Widow250PlaceTray-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed',),
                   'object_scales': (0.7,),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.75, .27, -.30),

                   'container_name': 'tray',
                   'fixed_container_position': True,
                   'start_object_in_gripper': True,

                   'use_neutral_action': True,
                   'neutral_gripper_open': False,
                   }
    },
    {
        'id': 'Widow250SinglePutInBowl-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed',),
                   'object_scales': (0.7,),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.59, .27, -.30),

                   'container_name': 'bowl_small',


                   }
    },
    {
        'id': 'Widow250SinglePutInBowlRandomBowlPosition-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed',),
                   'object_scales': (0.7,),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.5, .18, -.30),
                   'object_position_high': (.7, .27, -.30),

                   'container_name': 'bowl_small',


                   }
    },
    {
        'id': 'Widow250PutInBowlRandomBowlPosition-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.5, .18, -.30),
                   'object_position_high': (.7, .27, -.30),

                   'container_name': 'bowl_small',


                   }
    },
    {
        'id': 'Widow250MultiObjectPutInBowlRandomBowlPositionTrain-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceMultiObjectEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,
                   'num_objects': 2,
                   'load_tray': False,
                   'object_position_low': (.5, .18, -.30),
                   'object_position_high': (.7, .27, -.30),

                   'container_name': 'bowl_small',


                   }
    },
    {
        'id': 'Widow250MultiObjectPutInBowlRandomBowlPositionTest-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceMultiObjectEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'possible_objects': PICK_PLACE_TEST_OBJECTS,
                   'num_objects': 2,
                   'load_tray': False,
                   'object_position_low': (.5, .18, -.30),
                   'object_position_high': (.7, .27, -.30),

                   'container_name': 'bowl_small',


                   }
    },
    {
        'id': 'Widow250MultiShedPutInBowlRandomBowlPositionTest-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceMultiObjectEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'possible_objects': PICK_PLACE_DEMO_OBJECTS,

                   'num_objects': 2,
                   'load_tray': False,
                   'object_position_low': (.5, .18, -.30),
                   'object_position_high': (.7, .27, -.30),

                   'container_name': 'bowl_small',


                   }
    },

    {
        'id': 'Widow250PutInTray-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.59, .27, -.30),

                   'container_name': 'tray',

                   }
    },
    {
        'id': 'Widow250PutInBox-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.59, .27, -.30),

                   'container_name': 'open_box',

                   }
    },
    {
        'id': 'Widow250PlaceOnCube-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.59, .27, -.30),

                   'container_name': 'cube',

                   }
    },
    {
        'id': 'Widow250PutInPanTefal-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.69, .27, -.30),

                   'container_name': 'pan_tefal',

                   }
    },
    {
        'id': 'Widow250PutInPanTefalTestRL1-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('square_rod_embellishment',
                                    'grill_trash_can'),
                   'object_scales': (0.6, 0.5),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'target_object': 'square_rod_embellishment',

                   'load_tray': False,
                   'container_name': 'pan_tefal',

                   }
    },
    {
        'id': 'Widow250PutInPanTefalFixedTestRL1-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('square_rod_embellishment',
                                    'grill_trash_can'),
                   'object_scales': (0.6, 0.5),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'target_object': 'square_rod_embellishment',

                   'load_tray': False,
                   'container_name': 'pan_tefal',
                   'fixed_container_position': True,
                   }
    },
    {
        'id': 'Widow250PutInPanTefalTestRL2-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'sack_vase'),
                   'object_scales': (0.6, 0.6),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'target_object': 'shed',

                   'load_tray': False,
                   'container_name': 'pan_tefal',
                   }
    },
    {
        'id': 'Widow250PutInPanTefalTestRL3-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('two_handled_vase',
                                    'thick_wood_chair',),
                   'object_scales': (0.45, 0.4),
                   'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0)),
                   'target_object': 'two_handled_vase',

                   'load_tray': False,
                   'container_name': 'pan_tefal',

                   }
    },
    {
        'id': 'Widow250PutInPanTefalRL4-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('curved_handle_cup',
                                    'baseball_cap',),
                   'object_scales': (0.5, 0.5),
                   'object_orientations': ((0, 0.707, 0.707, 0),
                                           (0, -0.707, 0.707, 0)),
                   'target_object': 'curved_handle_cup',

                   'load_tray': False,
                   'container_name': 'pan_tefal',
                   }
    },
    {
        'id': 'Widow250PutInTableTop-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.69, .27, -.30),

                   'container_name': 'table_top',

                   }
    },
    {
        'id': 'Widow250PutOnTorus-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.69, .27, -.30),

                   'container_name': 'torus',
                   }
    },
    {
        'id': 'Widow250PutOnCubeConcave-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.69, .27, -.30),

                   'container_name': 'cube_concave',

                   }
    },
    {
        'id': 'Widow250PutOnPlate-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.69, .27, -.30),

                   'container_name': 'plate',

                   }
    },
    {
        'id': 'Widow250PutOnHusky-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.69, .27, -.30),

                   'container_name': 'husky',

                   }
    },
    {
        'id': 'Widow250PutOnMarbleCube-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.69, .27, -.30),

                   'container_name': 'marble_cube',

                   }
    },
    {
        'id': 'Widow250PutOnMarbleCubeTestRL1-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('square_rod_embellishment',
                                    'grill_trash_can'),
                   'object_scales': (0.6, 0.5),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'target_object': 'square_rod_embellishment',

                   'load_tray': False,

                   'container_name': 'marble_cube',

                   }
    },
    {
        'id': 'Widow250PutOnMarbleCubeTestRL2-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'sack_vase'),
                   'object_scales': (0.6, 0.6),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'target_object': 'shed',

                   'load_tray': False,
                   'container_name': 'marble_cube',

                   }
    },
    {
        'id': 'Widow250PutOnMarbleCubeTestRL3-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('two_handled_vase',
                                    'thick_wood_chair',),
                   'object_scales': (0.45, 0.4),
                   'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0)),
                   'target_object': 'two_handled_vase',

                   'load_tray': False,
                   'container_name': 'marble_cube',

                   }
    },
    {
        'id': 'Widow250PutOnMarbleCubeTestRL4-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('curved_handle_cup',
                                    'baseball_cap',),
                   'object_scales': (0.5, 0.5),
                   'object_orientations': ((0, 0.707, 0.707, 0),
                                           (0, -0.707, 0.707, 0)),
                   'target_object': 'curved_handle_cup',

                   'load_tray': False,
                   'container_name': 'marble_cube',

                   }
    },
    {
        'id': 'Widow250PutOnMarbleCubeFixedTestRL4-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('curved_handle_cup',
                                    'baseball_cap',),
                   'object_scales': (0.5, 0.5),
                   'object_orientations': ((0, 0.707, 0.707, 0),
                                           (0, -0.707, 0.707, 0)),
                   'target_object': 'curved_handle_cup',

                   'load_tray': False,
                   'container_name': 'marble_cube',
                   'fixed_container_position': True,

                   }
    },
    {
        'id': 'Widow250PutInBasket-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',

                   'load_tray': False,
                   'container_name': 'basket',

                   }
    },
    {
        'id': 'Widow250PutInBasketTestRL1-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('square_rod_embellishment',
                                    'grill_trash_can'),
                   'object_scales': (0.6, 0.5),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'target_object': 'square_rod_embellishment',

                   'load_tray': False,
                   'container_name': 'basket',

                   }
    },
    {
        'id': 'Widow250PutInBasketTestRL2-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'sack_vase'),
                   'object_scales': (0.6, 0.6),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'target_object': 'shed',

                   'load_tray': False,
                   'container_name': 'basket',

                   }
    },
    {
        'id': 'Widow250PutInBasketFixedTestRL2-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'sack_vase'),
                   'object_scales': (0.6, 0.6),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'target_object': 'shed',

                   'load_tray': False,
                   'container_name': 'basket',
                   'fixed_container_position': True,

                   }
    },
    {
        'id': 'Widow250PutOnCheckerboardTable-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed', 'two_handled_vase'),
                   'object_scales': (0.7, 0.6),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.69, .27, -.30),

                   'container_name': 'checkerboard_table',

                   }
    },
    {
        'id': 'Widow250PutOnCheckerboardTableTestRL3-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('two_handled_vase',
                                    'thick_wood_chair',),
                   'object_scales': (0.45, 0.4),
                   'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0)),
                   'target_object': 'two_handled_vase',

                   'load_tray': False,
                   'container_name': 'checkerboard_table',

                   }
    },
    {
        'id': 'Widow250PutOnCheckerboardTableFixedTestRL3-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('two_handled_vase',
                                    'thick_wood_chair',),
                   'object_scales': (0.45, 0.4),
                   'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0)),
                   'target_object': 'two_handled_vase',

                   'load_tray': False,
                   'container_name': 'checkerboard_table',
                   'fixed_container_position': True,

                   }
    },
    {
        'id': 'Widow250PutOnCheckerboardTableTestRL4-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('curved_handle_cup',
                                    'baseball_cap',),
                   'object_scales': (0.5, 0.5),
                   'object_orientations': ((0, 0.707, 0.707, 0),
                                           (0, -0.707, 0.707, 0)),
                   'target_object': 'curved_handle_cup',

                   'load_tray': False,
                   'container_name': 'checkerboard_table',

                   }
    },
    # Drawer environments
    {
        'id': 'Widow250DrawerOpen-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DrawerEnv',
        'kwargs': {'reward_type': 'opening',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'start_opened': False,

                   }
    },
    {
        'id': 'Widow250DrawerOpenNeutral-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DrawerEnv',
        'kwargs': {'reward_type': 'opening',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'use_neutral_action': True
                   }
    },
    {
        'id': 'Widow250DrawerGrasp-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DrawerEnv',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'start_opened': True,
                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   }
    },
    {
        'id': 'Widow250DrawerGraspNeutral-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DrawerEnv',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'start_opened': True,
                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'use_neutral_action': True
                   }
    },
    {
        'id': 'Widow250DrawerOpenGrasp-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DrawerEnv',
        'kwargs': {'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'use_neutral_action': True
                   }
    },
    {
        'id': 'Widow250DrawerRandomizedOpen-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DrawerRandomizedEnv',
        'kwargs': {'reward_type': 'opening',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   }
    },
    {
        'id': 'Widow250DoubleDrawerOpenNeutral-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DoubleDrawerEnv',
        'kwargs': {'drawer_pos': (0.47, 0.2, -.35),
                   'reward_type': 'opening',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'use_neutral_action': True
                   }
    },
    {
        'id': 'Widow250DoubleDrawerOpenGraspNeutral-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DoubleDrawerEnv',
        'kwargs': {'drawer_pos': (0.47, 0.2, -.35),
                   'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'use_neutral_action': True
                   }
    },
    {
        'id': 'Widow250DoubleDrawerGraspNeutral-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DoubleDrawerEnv',
        'kwargs': {'drawer_pos': (0.47, 0.2, -.35),
                   'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'start_opened': True,
                   'use_neutral_action': True
                   }
    },
    {
        'id': 'Widow250DoubleDrawerPickPlaceOpenGraspNeutral-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DoubleDrawerEnv',
        'kwargs': {'drawer_pos': (0.47, 0.2, -.35),
                   'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'start_opened': False,
                   'use_neutral_action': True,
                   'blocking_object_in_tray': False,
                   }
    },
    {
        'id': 'Widow250DoubleDrawerCloseOpenNeutral-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DoubleDrawerEnv',
        'kwargs': {'drawer_pos': (0.35, 0.2, -.35),
                   'reward_type': 'opening',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'start_opened': True,
                   'start_top_opened': False,
                   'use_neutral_action': True,
                   }
    },
    {
        'id': 'Widow250DoubleDrawerCloseOpenGraspNeutral-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DoubleDrawerEnv',
        'kwargs': {'drawer_pos': (0.47, 0.2, -.35),
                   'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'start_opened': False,
                   'start_top_opened': True,
                   'use_neutral_action': True,
                   }
    },
    {

        'id': 'Widow250DrawerRandomizedOpenTwoObjGrasp-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DrawerRandomizedEnv',
        'kwargs': {'reward_type': 'opening',
                   'control_mode': 'discrete_gripper',

                   'object_names': ("shed", "sack_vase"),
                   'object_scales': (0.6, 0.6),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'target_object': "shed",
                   'load_tray': False,
                   }
    },
    # Button environments
    {
        'id': 'Widow250ButtonPress-v0',
        'entry_point': 'roboverse.envs.widow250_button:Widow250ButtonEnv',
        'kwargs': {'control_mode': 'discrete_gripper',
                   'load_tray': False,
                   }
    },
    {
        'id': 'Widow250ButtonPressTwoObjGrasp-v0',
        'entry_point': 'roboverse.envs.widow250_button:Widow250ButtonEnv',
        'kwargs': {'control_mode': 'discrete_gripper',

                   'object_names': ("shed", "sack_vase"),
                   'object_scales': (0.6, 0.6),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'object_position_high': (.75, .25, -.30),
                   'object_position_low': (.6, .1, -.30),
                   'target_object': "shed",
                   'load_tray': False,
                   }
    },
    {
        'id': 'Widow250RandPosButtonPressTwoObjGrasp-v0',
        'entry_point': 'roboverse.envs.widow250_button:Widow250ButtonEnv',
        'kwargs': {'control_mode': 'discrete_gripper',
                   'button_pos_low': (0.5, 0.25, -.34),
                   'button_pos_high': (0.55, 0.15, -.34),

                   'object_names': ("shed", "sack_vase"),
                   'object_scales': (0.6, 0.6),
                   'object_orientations': ((0, 0, 1, 0), (0, 0.707, 0.707, 0)),
                   'object_position_high': (.75, .25, -.30),
                   'object_position_low': (.65, .1, -.30),
                   'target_object': "shed",
                   'load_tray': False,
                   }
    },
    {
        'id': 'Widow250MultiShedPutInMultiBowlTest-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceMultiObjectMultiContainerEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',
                   'load_tray': False,
                   'num_objects': 2,
                   'possible_objects': PICK_PLACE_DEMO_OBJECTS,
                   'possible_containers': PICK_PLACE_DEMO_CONTAINERS,


                   }
    },
    {
        'id': 'Widow250DrawerPickPlaceOpenGraspNeutral-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DrawerEnv',
        'kwargs': {'drawer_pos': (0.47, 0.2, -.35),
                   'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   'start_opened': False,
                   'use_neutral_action': True,
                   'blocking_object_in_tray': False,
                   }
    },




    {
        'id': 'Widow250TableMultiObjectMultiContainerTrain-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableMultiObjectMultiContainerEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': False,
                   'num_objects': 2,
                   'num_containers': 2,

                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,
                   'possible_containers': TRAIN_CONTAINERS,

                   }
    },

    {
        'id': 'Widow250TableMultiObjectMultiContainerTest-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableMultiObjectMultiContainerEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',
                    'drawer_pos': (0.27, 0.27, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,

                    'object_names': ('gatorade', 'pepsi_bottle', 'shed', 'glass_half_gallon'),

                    'object_scales': (0.75, 0.75, 0.75, 0.7),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.65, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.55, .1, -.35),

                    'min_distance_drawer': 0.14,
                    'min_distance_container': 0.08,
                    'min_distance_obj': 0.12,

                   'load_tray': False,
                   'num_objects': 2,
                   'num_containers': 2,

                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,
                   'possible_containers': TRAIN_CONTAINERS,

                   }
    },


    {
        'id': 'Widow250DrawerTestPos-v0',
        'entry_point': 'roboverse.envs.widow250_drawer:Widow250DrawerEnv',
        'kwargs': {
                    # 'drawer_pos': (0.3, 0.2, -.35),
                    'drawer_pos': (0.1, 0.0, -.35),
                    # 'drawer_pos': (0.55, 0.3, -.35)

                   'reward_type': 'grasping',
                   'control_mode': 'discrete_gripper',
                   'start_opened': False,
                   'object_names': ('ball',),
                   'object_scales': (0.75,),
                   'target_object': 'ball',
                   'load_tray': False,
                   }
    },



    {
        'id': 'Widow250TableCleanTest-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': False,
                #    'num_objects': 2,
                #    'num_containers': 2,

                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },
    {
        'id': 'Widow250TableCleanTest-v1',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,


                    'drawer_pos': (0.2, 0.24, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 4,
                    'object_names': ('gatorade', 'pepsi_bottle', 'shed', 'glass_half_gallon'),
                    'object_targets': ('drawer_inside', 'container', 'tray', 'container'),
                    'object_scales': (0.75, 0.75, 0.75, 0.7),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.65, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.55, .1, -.35),

                    'min_distance_drawer': 0.16,
                    'min_distance_container': 0.1,
                    'min_distance_obj': 0.1,



                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },
    {
        'id': 'Widow250TableCleanTest-v2',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,


                    'drawer_pos': (0.3, 0.23, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 3,
                    'object_names': ('gatorade', 'shed', 'pepsi_bottle',  ),
                    'object_targets': ('drawer_inside', 'container', 'tray'),
                    'object_scales': (0.75, 0.75, 0.75),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.72, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.62, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.9, 0.0, -.37),
                    'random_shuffle_object': False,
                    'random_shuffle_target': False,


                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },

    {
        'id': 'TestWidow250PickPlaceTray-v0',
        'entry_point': 'roboverse.envs.widow250_pickplace:Widow250PickPlaceEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'object_names': ('shed',),
                   'object_scales': (0.7,),
                   'target_object': 'shed',
                   'load_tray': False,
                   'object_position_low': (.49, .18, -.30),
                   'object_position_high': (.75, .27, -.30),

                   'container_name': 'tray',
                   'fixed_container_position': True,
                   'start_object_in_gripper': False,

                   'use_neutral_action': True,
                   'neutral_gripper_open': False,
                   }
    },
    {
        'id': 'Widow250TableCleanObjects2Random-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                #    'num_objects': 2,
                #    'num_containers': 2,


                    'drawer_pos': (0.3, 0.23, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 2,
                    'object_names': ('gatorade', 'shed',  ),
                    'object_targets': ('drawer_inside', 'container',),
                    'object_scales': (0.75, 0.75,),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0),),

                    'object_position_high': (0.72, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.62, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.9, 0.0, -.37),
                    'load_tray': False,
                    'random_shuffle_object': True,
                    'random_shuffle_target': False,


                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },

    {
        'id': 'Widow250TableCleanObjects2RandomTray-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                #    'num_objects': 2,
                #    'num_containers': 2,


                    'drawer_pos': (0.3, 0.23, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 2,
                    'object_names': ('gatorade', 'shed',  ),
                    'object_targets': ('drawer_inside', 'tray',),
                    'object_scales': (0.75, 0.75,),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0),),

                    'object_position_high': (0.72, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.62, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.8, 0., -.37),
                    'load_tray': True,
                    'random_tray': True,
                    'tray_position_high': (0.9, 0., -.35), # (.7, .27, -.35)
                    'tray_position_low': (0.8, -0.2, -.35),
                    'random_shuffle_object': True,
                    'random_shuffle_target': False,


                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },
    {
        'id': 'Widow250TableCleanObjects3Random-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,


                    'drawer_pos': (0.3, 0.23, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 3,
                    'object_names': ('gatorade', 'shed', 'pepsi_bottle',  ),
                    'object_targets': ('container', 'tray', 'drawer_inside'),
                    'object_scales': (0.75, 0.75, 0.75),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.72, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.62, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.9, 0.0, -.37),
                    'random_shuffle_object': True,
                    'random_shuffle_target': False,


                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },

        {
        'id': 'Widow250TableCleanObjects3Random-v1',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,


                    'drawer_pos': (0.3, 0.23, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 3,
                    'object_names': ('gatorade', 'shed', 'pepsi_bottle',  ),
                    'object_targets': ('drawer_inside', 'container', 'tray'),
                    'object_scales': (0.75, 0.75, 0.75),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.72, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.62, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.9, 0.0, -.37),
                    'random_shuffle_object': True,
                    'random_shuffle_target': False,


                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },

    {
        'id': 'Widow250TableCleanObjects3Random-v2',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,
                    'observation_mode': 'pixels',

                    'drawer_pos': (0.3, 0.23, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 3,
                    'object_names': ('gatorade', 'shed', 'pepsi_bottle',  ),
                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    'object_scales': (0.75, 0.75, 0.75),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.56, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.9, 0.0, -.37),
                    'random_tray': True,
                    'tray_position_high': (0.9, 0., -.35), # (.7, .27, -.35)
                    'tray_position_low': (0.8, -0.15, -.35),
                    'random_shuffle_object': True,
                    'random_shuffle_target': False,


                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },

    {
        'id': 'Widow250TableCleanObjects3RandomNoimage-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,
                    'observation_mode': 'noimage',

                    'drawer_pos': (0.3, 0.23, -.35),

                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 3,
                    'object_names': ('gatorade', 'shed', 'pepsi_bottle',  ),
                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    'object_scales': (0.75, 0.75, 0.75),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.56, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.9, 0.0, -.37),
                    'random_tray': True,
                    'tray_position_high': (0.9, 0., -.35), # (.7, .27, -.35)
                    'tray_position_low': (0.8, -0.15, -.35),
                    'random_shuffle_object': True,
                    'random_shuffle_target': False,

                    'base_position_high': (0.63, 0.01, -0.4), # (.7, .27, -.35)
                    'base_position_low': (0.57, -0.01, -0.4),
                    'base_position': (0.6, 0.0, -0.4),
                    'random_base': False,
                    'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },

{
        'id': 'Widow250TableCleanObjects3FixedNoimage-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,
                    'observation_mode': 'noimage',

                    'drawer_pos': (0.3, 0.23, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 3,
                    'object_names': ('gatorade', 'pepsi_bottle', 'shed',  ),
                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    'object_scales': (0.75, 0.75, 0.75),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.56, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.9, 0.0, -.37),
                    'random_tray': False,
                    'tray_position_high': (0.9, 0., -.35), # (.7, .27, -.35)
                    'tray_position_low': (0.8, -0.15, -.35),
                    'random_shuffle_object': False,
                    'random_shuffle_target': False,

                    'fixed_init_pos': (
                        np.array([0.8, 0.23837455, -0.3]),       # container position
                        [np.array([0.5096847 ,  0.19509713, -0.35]),     # object positions
                         np.array([0.60079058,  0.2650282 , -0.35]),
                         np.array([0.69213048,  0.18794259, -0.35])]
                    ),


                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },

    {
        'id': 'Widow250TableCleanObjects4Random-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,


                    'drawer_pos': (0.3, 0.23, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'start_opened': False,
                    'num_objects': 4,
                    'object_names': ('gatorade', 'shed', 'pepsi_bottle', 'shed' ),
                    'object_targets': ('drawer_inside', 'container', 'tray', 'tray'),
                    'object_scales': (0.75, 0.75, 0.75, 0.75),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.78, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.56, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.3,
                    'tray_position': (.9, 0.0, -.37),
                    'random_tray': True,
                    'tray_position_high': (0.9, 0., -.35), # (.7, .27, -.35)
                    'tray_position_low': (0.8, -0.2, -.35),
                    'random_shuffle_object': True,
                    'random_shuffle_target': False,


                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },

    {
        'id': 'Widow250TableCleanFixedTest-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,
                    'observation_mode': 'noimage',

                    # 'drawer_pos': (0.3, 0.3, -.35),
                    'drawer_pos': (0.3, 0.23, -.35),
                    'start_opened': False,
                    'num_objects': 3,
                    'object_names': ('gatorade', 'pepsi_bottle', 'shed',  ),
                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    'object_scales': (0.75, 0.75, 0.75),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.56, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.9, 0.0, -.37),
                    'random_tray': False,
                    'tray_position_high': (0.9, 0., -.35), # (.7, .27, -.35)
                    'tray_position_low': (0.8, -0.15, -.35),
                    'random_shuffle_object': False,
                    'random_shuffle_target': False,

                    'fixed_init_pos': (
                        np.array([0.8, 0.23837455, -0.3]),       # container position
                        [np.array([0.5096847 ,  0.229713, -0.35]),     # object positions
                         np.array([0.60079058,  0.2650282 , -0.35]),
                         np.array([0.69213048,  0.18794259, -0.35])]
                    ),


                   'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },
{
        'id': 'Widow250TableCleanRandom-v0',
        'entry_point': 'roboverse.envs.widow250_tableclean'
                       ':Widow250TableEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,
                    'observation_mode': 'noimage',

                    # 'drawer_pos': (0.14, 0.1, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    'drawer_pos': (0.3, 0.23, -.35),

                    'start_opened': False,
                    'num_objects': 3,
                    'object_names': ('gatorade', 'shed', 'pepsi_bottle',  ),
                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    'object_scales': (0.75, 0.75, 0.75),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    # 'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    # 'object_position_low': (.56, .1, -.35),
                    
                    'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.3, .1, -.35),



                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (.9, 0.0, -.37),
                    'random_tray': True,
                    'tray_position_high': (0.9, 0., -.35), # (.7, .27, -.35)
                    'tray_position_low': (0.8, -0.15, -.35),
                    'random_shuffle_object': False,
                    'random_shuffle_target': False,

                    'base_position_high': (0.63, 0.02, -0.4), # (.7, .27, -.35)
                    'base_position_low': (0.57, -0.02, -0.4),
                    'base_position': (0.6, 0.0, -0.4),
                    'random_base': True,
                    'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },
    {
        'id': 'Widow250OfficePickPlaceFixed-v0',
        'entry_point': 'roboverse.envs.widow250_office'
                       ':Widow250OfficeEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,
                    'observation_mode': 'noimage',

                    'drawer_pos': (0.1, 0.0, -.35), # middle
                    'drawer_pos_low': (0.8, 0.0, -.35), # (.7, .27, -.35)
                    'drawer_pos_high': (.12, 0.1, -.35),

                    'start_opened': False,
                    'num_objects': 3, # 2 or 3, 
                    # 'object_names': ('eraser', 'gatorade', 'shed', 'pepsi_bottle', ),
                    # 'object_targets': ('tray', 'container', 'drawer_inside'),        'object_names': ('eraser', 'gatorade', 'shed', 'pepsi_bottle', ),
                    # 'object_names': ('eraser', 'shed', 'pepsi_bottle', 'gatorade', ),
                    'object_names': ('eraser', 'shed', 'pepsi_bottle', 'gatorade', ),

                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    # 'object_targets': ('container', 'tray', 'drawer_inside'),
                    
                    'object_scales': (0.8, 0.8, 0.8, 0.8),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),
                                   
                    'random_object_position': False,
                    'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.3, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (0.25,-0.19, -.39),
                    # 'tray_position': (0.25,-0.25, -.39),

                    'random_tray': False,
                    'tray_position_high': (0.25,-0.22, -.39), # (.7, .27, -.35)
                    'tray_position_low': (0.22,-0.25, -.39),
                    'random_shuffle_object': False,
                    'random_shuffle_target': False,

                    'base_position_high': (0.62, 0.02, -0.4), # (.7, .27, -.35)
                    'base_position_low': (0.58, -0.02, -0.4),
                    'base_position': (0.6, 0.0, -0.4),
                    'random_joint_values': False,
                    'random_base': False,
                    'random_base_orientation': False,
                    'base_orientation' : (0, 0, -180),
                    'base_orientation_high':(0, 0, -170),
                    'base_orientation_low' : (0, 0, -190),
                    'possible_objects': PICK_PLACE_TRAIN_OBJECTS,
                    'random_drawer': False,
                   }
    },
    {
        'id': 'Widow250OfficePickPlaceRandom-v0',
        'entry_point': 'roboverse.envs.widow250_office'
                       ':Widow250OfficeEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,
                    'observation_mode': 'noimage',

                    # 'drawer_pos': (0.14, 0.1, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    # 'drawer_pos': (0.15, 0.23, -.35),
                    # 'drawer_pos': (0.1, 0.0, -.35),
                    # 'drawer_pos': (0.1, -0.2, -.35), # upper right
                    # 'drawer_pos': (0.1, 0.0, -.35), # middle
                    'random_drawer' : True,
                    'drawer_pos': (0.1, 0.0, -.35), # middle
                    'drawer_pos_low': (0.08, -0.01, -.35), # (.7, .27, -.35)
                    'drawer_pos_high': (.12, 0.03, -.35),

                    # 'drawer_pos': (0.15, 0.2, -.35),

                    # 'drawer_pos': (0.6, -0.5, -.32),


                    'start_opened': False,
                    'num_objects': 3, # 2 or 3, 
                    'object_names': ('eraser', 'gatorade', 'shed', 'pepsi_bottle', ),
                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    'object_scales': (0.8, 0.8, 0.8, 0.8),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),
                                   
                    # 'num_objects': 2,
                    # 'object_names': ('shed','gatorade' ),
                    # 'object_targets': ('container','drawer_inside',),
                    # 'object_scales': (0.9, 0.9),
                    # 'object_orientations': ((0, 0, 1, 0),(0, 0, 1, 0)),
                   
                   
                   
                   
                    # 'object_orientations': ((0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0)),

                    # 'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    # 'object_position_low': (.56, .1, -.35),
                    'random_object_position' : True,
                    'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.3, .1, -.35),

                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.13,
                    'tray_position': (0.26,-0.2, -.39),
                    # 'tray_position': (0.25,-0.25, -.39),

                    'random_tray': True,
                    'tray_position_high': (0.28,-0.18, -.39), # (.7, .27, -.35)
                    'tray_position_low': (0.25,-0.2, -.39),
                    'random_shuffle_object': True,
                    'random_shuffle_target': True,

                    'base_position_high': (0.62, 0.02, -0.4), # (.7, .27, -.35)
                    'base_position_low': (0.58, -0.02, -0.4),
                    'base_position': (0.6, 0.0, -0.4),
                    'random_base': True,
                    'random_base_orientation': False,
                    'random_joint_values': True,
                    'base_orientation' : (0, 0, -180),
                    'base_orientation_high':(0, 0, -182),
                    'base_orientation_low' : (0, 0, -188),
                    'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },

    {
        'id': 'Widow250OfficeManyObjectsRandom-v0',
        'entry_point': 'roboverse.envs.widow250_officeManyObjects'
                       ':Widow250OfficeEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',
                    'observation_mode': 'noimage',

                    #'num_objects': 3,
                    #'object_names': ('eraser', 'gatorade', 'shed', 'pepsi_bottle', ),
                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    #'object_scales': (0.8, 0.8, 0.8, 0.8),
                    #'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                    'random_shuffle_object': True,
                    'random_shuffle_target': True,

                    'possible_objects': PICK_PLACE_TRAIN_OBJECTS,
                   }
    },

    {
        'id': 'Widow250OfficeManyObjectsFixed-v0',
        'entry_point': 'roboverse.envs.widow250_officeManyObjects'
                       ':Widow250OfficeEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',
                    'observation_mode': 'noimage',

                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    'original_object_positions': (
                        (0.33620103,  0.12358467, -0.35),
                        (0.55123888, -0.17699107, -0.35),
                        (0.84287004, -0.1479069 , -0.35),
                        (0.75200037, -0.14383595, -0.35),
                        (0.42755662, -0.13711447, -0.35),
                        (0.39866522,  0.18929185, -0.35),
                        (0.46422192, -0.23138137, -0.35),
                    ),

                    'random_shuffle_object': True,
                    'random_shuffle_target': True,
                    'random_object_position': False,
                    'object_jitter': 0.05,

                    'possible_objects': PICK_PLACE_TRAIN_OBJECTS,
                   }
    },

    {
        'id': 'Widow250OfficeManyObjectsFixed5Obj-v0',
        'entry_point': 'roboverse.envs.widow250_officeManyObjects'
                       ':Widow250OfficeEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',
                    'observation_mode': 'noimage',

                    'object_targets': ('tray', 'container', 'tray', 'container', 'drawer_inside'),
                    'original_object_positions': (
                        (0.33620103,  0.12358467, -0.35),
                        (0.55123888, -0.17699107, -0.35),
                        (0.84287004, -0.1479069 , -0.35),
                        (0.75200037, -0.14383595, -0.35),
                        (0.42755662, -0.13711447, -0.35),
                        (0.39866522,  0.18929185, -0.35),
                        (0.46422192, -0.23138137, -0.35),
                    ),

                    'random_shuffle_object': True,
                    'random_shuffle_target': True,
                    'random_object_position': False,
                    'object_jitter': 0.05,

                    'possible_objects': PICK_PLACE_TRAIN_OBJECTS,
                   }
    },

    {
        'id': 'Widow250OfficePickPlaceSimpleFixed-v0',
        'entry_point': 'roboverse.envs.widow250_office'
                       ':Widow250OfficeEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,
                    'observation_mode': 'noimage',

                    'drawer_pos': (0.1, 0.0, -.35), # middle
                    'drawer_pos_low': (0.8, 0.0, -.35), # (.7, .27, -.35)
                    'drawer_pos_high': (.12, 0.1, -.35),

                    'start_opened': False,
                    'num_objects': 2, # 2 or 3, 
                    # 'object_names': ('eraser', 'gatorade', 'shed', 'pepsi_bottle', ),
                    # 'object_targets': ('tray', 'container', 'drawer_inside'),        'object_names': ('eraser', 'gatorade', 'shed', 'pepsi_bottle', ),
                    # 'object_names': ('eraser', 'shed', 'pepsi_bottle', 'gatorade', ),
                    'object_names': ('eraser', 'shed', 'pepsi_bottle', 'gatorade', ),

                    'object_targets': ('tray', 'drawer_inside', 'container'),
                    # 'object_targets': ('container', 'tray', 'drawer_inside'),
                    
                    'object_scales': (0.8, 0.8, 0.8, 0.8),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),
                                   
                    'random_object_position': False,
                    'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.3, .1, -.35),
                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.11,
                    'tray_position': (0.25,-0.19, -.39),
                    # 'tray_position': (0.25,-0.25, -.39),

                    'random_tray': False,
                    'tray_position_high': (0.25,-0.22, -.39), # (.7, .27, -.35)
                    'tray_position_low': (0.22,-0.25, -.39),
                    'random_shuffle_object': False,
                    'random_shuffle_target': False,

                    'base_position_high': (0.62, 0.02, -0.4), # (.7, .27, -.35)
                    'base_position_low': (0.58, -0.02, -0.4),
                    'base_position': (0.6, 0.0, -0.4),
                    'random_joint_values': False,
                    'random_base': False,
                    'random_base_orientation': False,
                    'base_orientation' : (0, 0, -180),
                    'base_orientation_high':(0, 0, -170),
                    'base_orientation_low' : (0, 0, -190),
                    'possible_objects': PICK_PLACE_TRAIN_OBJECTS,
                    'random_drawer': False,
                   }
    },

    {
        'id': 'Widow250OfficePickPlaceSimpleRandom-v0',
        'entry_point': 'roboverse.envs.widow250_office'
                       ':Widow250OfficeEnv',
        'kwargs': {'reward_type': 'pick_place',
                   'control_mode': 'discrete_gripper',

                   'load_tray': True,
                #    'num_objects': 2,
                #    'num_containers': 2,
                    'observation_mode': 'noimage',

                    # 'drawer_pos': (0.14, 0.1, -.35),
                    # 'drawer_pos': (0.35, 0.2, -.35),
                    # 'drawer_pos': (0.15, 0.23, -.35),
                    # 'drawer_pos': (0.1, 0.0, -.35),
                    # 'drawer_pos': (0.1, -0.2, -.35), # upper right
                    # 'drawer_pos': (0.1, 0.0, -.35), # middle
                    'random_drawer' : True,
                    'drawer_pos': (0.1, 0.0, -.35), # middle
                    'drawer_pos_low': (0.08, -0.01, -.35), # (.7, .27, -.35)
                    'drawer_pos_high': (.12, 0.03, -.35),

                    # 'drawer_pos': (0.15, 0.2, -.35),

                    # 'drawer_pos': (0.6, -0.5, -.32),


                    'start_opened': False,
                    'num_objects': 2, # 2 or 3, 
                    'object_names': ('eraser', 'gatorade', 'shed', 'pepsi_bottle', ),
                    'object_targets': ('tray', 'container', 'drawer_inside'),
                    'object_scales': (0.8, 0.8, 0.8, 0.8),
                    'object_orientations': ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),
                                   
                    # 'num_objects': 2,
                    # 'object_names': ('shed','gatorade' ),
                    # 'object_targets': ('container','drawer_inside',),
                    # 'object_scales': (0.9, 0.9),
                    # 'object_orientations': ((0, 0, 1, 0),(0, 0, 1, 0)),
                   
                   
                   
                   
                    # 'object_orientations': ((0, 0, 0, 0), (0, 0, 0, 0), (0, 0, 0, 0)),

                    # 'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    # 'object_position_low': (.56, .1, -.35),
                    'random_object_position' : True,
                    'object_position_high': (0.75, .9, -.35), # (.7, .27, -.35)
                    'object_position_low': (.3, .1, -.35),

                    'xyz_action_scale': 0.7,
                    'min_distance_drawer': 0.2,
                    'min_distance_container': 0.11,
                    'min_distance_obj': 0.13,
                    'tray_position': (0.26,-0.2, -.39),
                    # 'tray_position': (0.25,-0.25, -.39),

                    'random_tray': True,
                    'tray_position_high': (0.28,-0.18, -.39), # (.7, .27, -.35)
                    'tray_position_low': (0.25,-0.2, -.39),
                    'random_shuffle_object': True,
                    'random_shuffle_target': True,

                    'base_position_high': (0.62, 0.02, -0.4), # (.7, .27, -.35)
                    'base_position_low': (0.58, -0.02, -0.4),
                    'base_position': (0.6, 0.0, -0.4),
                    'random_base': False,
                    'random_base_orientation': False,
                    'random_joint_values': False,
                    'base_orientation' : (0, 0, -180),
                    'base_orientation_high':(0, 0, -182),
                    'base_orientation_low' : (0, 0, -188),
                    'possible_objects': PICK_PLACE_TRAIN_OBJECTS,

                   }
    },


    
)


def register_environments():
    for env in ENVIRONMENT_SPECS:
        gym.register(**env)

    gym_ids = tuple(
        environment_spec['id']
        for environment_spec in ENVIRONMENT_SPECS)

    return gym_ids


def make(env_name, *args, **kwargs):
    env = gym.make(env_name, *args, **kwargs)
    return env
