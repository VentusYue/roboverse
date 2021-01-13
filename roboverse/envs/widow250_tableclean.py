from roboverse.envs.widow250 import Widow250Env
from roboverse.envs.widow250_drawer import Widow250DrawerEnv
from roboverse.envs.widow250_pickplace import Widow250PickPlaceEnv, Widow250PickPlaceMultiObjectEnv
from roboverse.bullet import object_utils
import roboverse.bullet as bullet
from roboverse.envs import objects
from roboverse.envs.multi_object import MultiObjectEnv, MultiObjectMultiContainerEnv
from roboverse.assets.shapenet_object_lists import CONTAINER_CONFIGS, TRAIN_OBJECTS
import os.path as osp
import numpy as np
import random

OBJECT_IN_GRIPPER_PATH = osp.join(osp.dirname(osp.dirname(osp.realpath(__file__))),
                'assets/bullet-objects/bullet_saved_states/objects_in_gripper/')


class Widow250TableEnv(Widow250PickPlaceEnv):
    def __init__(self,
                container_name='plate',    
                # num_objects=3, 
                # object_names=('gatorade', 'pepsi_bottle', 'shed'),
                # target_object='gatorade',
                # object_scales=(0.75, 0.75, 0.75),
                # object_orientations=((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),

                num_objects=4, 
                object_names=('gatorade', 'pepsi_bottle', 'shed', 'glass_half_gallon'),
                object_targets=('container', 'container', 'container', 'container'),
                target_object='gatorade',
                object_scales=(0.75, 0.75, 0.75, 0.7),
                object_orientations=((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 0)),
                
                object_position_high=(0.65, .9, -.35), # (.7, .27, -.35)
                object_position_low=(.55, .1, -.35),
                
                possible_objects=TRAIN_OBJECTS[:10],            
                drawer_pos=(0.35, 0.2, -.35),
                drawer_quat=(0, 0, 0.707107, 0.707107),
                left_opening=True,  # False is not supported
                start_opened=False,
                reward_type='pick_place',
                min_distance_from_object = 0.12,
                min_distance_drawer=0.14,
                min_distance_container=0.08,
                min_distance_obj=0.08,
                # tray_position = (.8, 0.0, -.37),
                tray_position = (.8, 0.0, -.37),

                xyz_action_scale = 0.3,
                random_shuffle_object = True,
                random_shuffle_target = True,
                **kwargs):
        
        self.tray_position = tray_position
        self.drawer_pos = drawer_pos
        self.drawer_quat = drawer_quat
        self.left_opening = left_opening
        self.start_opened = start_opened
        self.drawer_opened_success_thresh = 0.95
        self.drawer_closed_success_thresh = 0.05     
        self.possible_objects = np.asarray(possible_objects) 
        self.random_shuffle_object = random_shuffle_object
        if self.random_shuffle_object:
            self.object_names = random.sample(object_names, len(object_names))
            print(self.object_names)
            self.object_targets = object_targets
        else:
            self.object_names = object_names
            self.object_targets = object_targets



        self.num_objects = num_objects
        self.xyz_action_scale = xyz_action_scale
        assert self.num_objects == len(object_names) == len(self.object_targets)



        self.inside_drawer_position = np.array(self.drawer_pos[:2] + (-.2,)) + np.array((0.12, 0, 0))
        self.top_drawer_position = np.array(self.drawer_pos[:2] + (0.1,))

        self.min_distance_drawer = min_distance_drawer
        self.min_distance_obj = min_distance_obj
        self.min_distance_container = min_distance_container

        super(Widow250TableEnv, self).__init__(
            object_names=object_names,
            target_object=target_object,
            object_orientations=object_orientations,
            object_scales=object_scales,
            container_name=container_name,
            **kwargs,
        )

        self.reward_type = reward_type
        self.num_objects = num_objects
    
    def _load_meshes(self):
        # super(Widow250TableEnv, self)._load_meshes()

        # TODO temporal defaults, need to change later


        self.table_id = objects.table()
        self.robot_id = objects.widow250()
        self.objects = {}
        if self.load_tray:
            self.tray_id = objects.tray(base_position=self.tray_position, scale=0.3)

        self.objects["drawer"] = object_utils.load_object(
            "drawer", self.drawer_pos, self.drawer_quat, scale=0.1)
        # Open and close testing.
        closed_drawer_x_pos = object_utils.open_drawer(
            self.objects['drawer'])[0]

        opened_drawer_x_pos = object_utils.close_drawer(
            self.objects['drawer'])[0]

        if self.left_opening:
            self.drawer_min_x_pos = closed_drawer_x_pos
            self.drawer_max_x_pos = opened_drawer_x_pos
        else:
            self.drawer_min_x_pos = opened_drawer_x_pos
            self.drawer_max_x_pos = closed_drawer_x_pos

        if self.start_opened:
            object_utils.open_drawer(self.objects['drawer'])

        if self.num_objects == 1:
            self.container_position, self.original_object_positions = \
                object_utils.generate_object_positions_single(
                    self.object_position_low, self.object_position_high,
                    self.container_position_low, self.container_position_high,
                    min_distance_large_obj=self.min_distance_from_object,
                )
        else: 
            self.container_position, self.original_object_positions = \
                object_utils.generate_multiple_object_positions(
                    self.object_position_low, self.object_position_high,
                    self.container_position_low, self.container_position_high,
                    drawer_pos=self.drawer_pos,
                    num_objects=self.num_objects,
                    min_distance_drawer = self.min_distance_drawer,
                    min_distance_container=self.min_distance_container,
                    min_distance_obj=self.min_distance_obj
                )
        self.container_position[-1] = self.container_position_z
        self.container_id = object_utils.load_object(self.container_name,
                                                     self.container_position,
                                                     self.container_orientation,
                                                     self.container_scale)  
        
        bullet.step_simulation(self.num_sim_steps_reset)
        
        for object_name, object_position in zip(self.object_names,
                                                self.original_object_positions):
            self.objects[object_name] = object_utils.load_object(
                object_name,
                object_position,
                object_quat=self.object_orientations[object_name],
                scale=self.object_scales[object_name])
            bullet.step_simulation(self.num_sim_steps_reset)
    
    def get_drawer_handle_pos(self):
        handle_pos = object_utils.get_drawer_handle_pos(
            self.objects["drawer"])
        return handle_pos


    def reset(self):
        if self.random_shuffle_object:
            self.object_names = random.sample(self.object_names, len(self.object_names))
            # print(f"reset: {self.object_names}")

        bullet.reset()
        bullet.setup_headless()
        self._load_meshes()
        bullet.reset_robot(
            self.robot_id,
            self.reset_joint_indices,
            self.reset_joint_values)
        self.is_gripper_open = True  # TODO(avi): Clean this up

        return self.get_observation()

    # def get_inside_drawer_pos(self):
    #     obj_pos_high = np.array(self.drawer_pos[:2] + (-.2,)) \
    #                    + (1 - 2 * (not self.left_opening)) * np.array((0.12, 0, 0))
    #     obj_pos_low = np.array(self.drawer_pos[:2] + (-.2,)) \
    #         - (1 - 2 * (not self.left_opening)) * np.array((-0.12, 0, 0))
       
        return 0.5 * (obj_pos_high + obj_pos_low)

    def is_drawer_open(self):
        # refers to bottom drawer in the double drawer case
        info = self.get_info()
        return info['drawer_opened_success']

    def get_drawer_opened_percentage(self, drawer_key="drawer"):
        # compatible with either drawer or upper_drawer
        drawer_x_pos = self.get_drawer_pos(drawer_key)[0]
        return object_utils.get_drawer_opened_percentage(
            self.left_opening, self.drawer_min_x_pos,
            self.drawer_max_x_pos, drawer_x_pos)

    def get_drawer_pos(self, drawer_key="drawer"):
        drawer_pos = object_utils.get_drawer_pos(
            self.objects[drawer_key])
        return drawer_pos



    # def get_reward(self, info):
    #     import pdb; pdb.set_trace()
    #     # if self.reward_type == 'pick_place':
    #     #     reward = float(info['place_success_target'])
    #     # elif self.reward_type == 'grasp':
    #     #     reward = float(info['grasp_success_target'])
    #     # else:
    #     #     raise NotImplementedError
    #     return reward

    def get_info(self):
        info = super(Widow250TableEnv, self).get_info()
        info['drawer_x_pos'] = self.get_drawer_pos()[0]
        info['drawer_opened_percentage'] = \
            self.get_drawer_opened_percentage()
        info['drawer_closed_percentage'] = \
            1 - self.get_drawer_opened_percentage()

        info['drawer_opened_success'] = info["drawer_opened_percentage"] > \
            self.drawer_opened_success_thresh

        info['drawer_closed_success'] = info["drawer_opened_percentage"] < \
            self.drawer_closed_success_thresh
        return info
    
    def is_drawer_closed(self):
        info = self.get_info()
        return info['drawer_closed_success']


if __name__ == "__main__":

    # Fixed container position
    env = Widow250TableEnv(
        gui=True,
    )

    # env = Widow250PickPlaceEnv(
    #     reward_type='pick_place',
    #     control_mode='discrete_gripper',
    #     object_names=('shed',),
    #     object_scales=(0.7,),
    #     target_object='shed',
    #     load_tray=False,
    #     object_position_low=(.5, .18, -.25),
    #     object_position_high=(.7, .27, -.25),
    #
    #     container_name='bowl_small',
    #     container_position_low=(.5, 0.26, -.25),
    #     container_position_high=(.7, 0.26, -.25),
    #     container_orientation=(0, 0, 0.707107, 0.707107),
    #     container_scale=0.07,
    #
    #     camera_distance=0.29,
    #     camera_target_pos=(0.6, 0.2, -0.28),
    #     gui=True
    # )
    import time
    for _ in range(10):
        env.reset()
        for _ in range(5):
            env.step(env.action_space.sample()*0.1)
            time.sleep(0.1)
