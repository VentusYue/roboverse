from roboverse.envs.widow250 import Widow250Env
from roboverse.envs.widow250_drawer import Widow250DrawerEnv
from roboverse.envs.widow250_pickplace import Widow250PickPlaceEnv, Widow250PickPlaceMultiObjectEnv
from roboverse.bullet import object_utils
import roboverse.bullet as bullet
from roboverse.envs import objects
from roboverse.envs.multi_object import MultiObjectEnv, MultiObjectMultiContainerEnv
from roboverse.assets.shapenet_object_lists import CONTAINER_CONFIGS, TRAIN_OBJECTS
from roboverse.utils.general_utils import AttrDict
import os.path as osp
import numpy as np
import random


OBJECT_IN_GRIPPER_PATH = osp.join(osp.dirname(osp.dirname(osp.realpath(__file__))),
                'assets/bullet-objects/bullet_saved_states/objects_in_gripper/')
END_EFFECTOR_INDEX = 8
RESET_JOINT_VALUES = [1.57, -0.6, -0.6, 0, -1.57, 0., 0., 0.036, -0.036]
RESET_JOINT_VALUES_GRIPPER_CLOSED = [1.57, -0.6, -0.6, 0, -1.57, 0., 0., 0.015, -0.015]
RESET_JOINT_INDICES = [0, 1, 2, 3, 4, 5, 7, 10, 11]
GUESS = 3.14  # TODO(avi) This is a guess, need to verify what joint this is
JOINT_LIMIT_LOWER = [-3.14, -1.88, -1.60, -3.14, -2.14, -3.14, -GUESS, 0.015,
                     -0.037]
JOINT_LIMIT_UPPER = [3.14, 1.99, 2.14, 3.14, 1.74, 3.14, GUESS, 0.037, -0.015]

JOINT_RANGE = []
for upper, lower in zip(JOINT_LIMIT_LOWER, JOINT_LIMIT_UPPER):
    JOINT_RANGE.append(upper - lower)

GRIPPER_LIMITS_LOW = JOINT_LIMIT_LOWER[-2:]
GRIPPER_LIMITS_HIGH = JOINT_LIMIT_UPPER[-2:]
GRIPPER_OPEN_STATE = [0.036, -0.036]
GRIPPER_CLOSED_STATE = [0.015, -0.015]

ACTION_DIM = 8

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
                load_tray = True,
                tray_position = (.9, 0.0, -.37),
                
                tray_position_high=(0.9, 0., -.35), # (.7, .27, -.35)
                tray_position_low=(0.8, -0.2, -.35),
                
                xyz_action_scale = 0.3,
                random_shuffle_object = True,
                random_shuffle_target = True,
                random_tray = False,
                **kwargs):
        
        self.load_tray = load_tray
        self.tray_position = tray_position
        self.random_tray = random_tray
        self.tray_position_high = tray_position_high
        self.tray_position_low = tray_position_low
        if self.random_tray:
            self.tray_position = np.random.uniform(
                low=self.tray_position_low, high=self.tray_position_high)
        self.drawer_pos = drawer_pos
        self.drawer_quat = drawer_quat
        self.left_opening = left_opening
        self.start_opened = start_opened
        self.drawer_opened_success_thresh = 0.9
        self.drawer_closed_success_thresh = 0.1     
        self.possible_objects = np.asarray(possible_objects) 
        self.random_shuffle_object = random_shuffle_object
        self.num_objects = num_objects
        self.object_position_high = list(object_position_high)
        self.object_position_low = list(object_position_low)

        if self.random_shuffle_object:
            self.object_names = random.sample(object_names, len(object_names))
            self.object_targets = object_targets
            # print(self.object_names)
            # print(self.object_targets)
        else:
            self.object_names = object_names
            self.object_targets = object_targets


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
        
        self.complete_tasks = 0
        self.subtasks = self.generate_tasks()
        self.current_task = self.subtasks.pop(0)
        
    def generate_pickplace_task(self,object_name="tray", object_target="container", 
                    target_position = np.array([0.9, 0., -0.37]), 
                    object_position = np.array([0.5, 0.2, -0.3])):
        task = AttrDict()
        task["type"] = "pickplace"
        task["info"] = AttrDict()
        task.info.object_name = object_name
        task.info.object_target = object_target
        task.info.target_position = target_position
        task.info.object_position = object_position
        task.info.object_current_position = object_position
        task.info.done = False
        return task

    def generate_drawer_open_task(self):
        task = AttrDict()
        task["type"] = "drawer_open"
        task["info"] = AttrDict()
        task.info.object_name = "drawer"
        task.info.object_target = "drawer"
        task.info.done = False
        return task

    def generate_drawer_close_task(self):
        task = AttrDict()
        task["type"] = "drawer_close"
        task["info"] = AttrDict()
        task.info.object_name = "drawer"
        task.info.object_target = "drawer"
        task.info.done = False
        return task

    def generate_tasks(self):
        """ generate the subtasks of the env """

        subtasks = [] # a queue consisting of dictionaries for each subtask
        for object_name, object_target, object_position in zip(self.object_names, self.object_targets, self.original_object_positions):
            target_position = self.get_target_position(object_target)
            if object_target == 'drawer_inside':
                drawer_open_task = self.generate_drawer_open_task()
                pickplace_task = self.generate_pickplace_task(object_name, object_target, object_position, target_position)
                drawer_close_task = self.generate_drawer_close_task()
                subtasks.append(drawer_open_task)
                subtasks.append(pickplace_task)               
                subtasks.append(drawer_close_task)
            else:
                pickplace_task = self.generate_pickplace_task(object_name, object_target, object_position, target_position)
                subtasks.append(pickplace_task) 
        
        return subtasks

    def get_target_position(self, object_target):
        if object_target == 'container':
            return self.container_position
        elif object_target == 'tray':
            return self.tray_position
        elif object_target == 'drawer_top':
            return list(self.top_drawer_position) 
        elif object_target == 'drawer_inside':
            return list(self.inside_drawer_position) 
        else:
            raise NotImplementedError

    def generate_objects_positions(self):
        if self.num_objects == 1:
            container_position, object_positions = \
                object_utils.generate_object_positions_single(
                    self.object_position_low, self.object_position_high,
                    self.container_position_low, self.container_position_high,
                    min_distance_large_obj=self.min_distance_from_object,
                )
        else: 
            container_position, object_positions = \
                object_utils.generate_multiple_object_positions(
                    self.object_position_low, self.object_position_high,
                    self.container_position_low, self.container_position_high,
                    drawer_pos=self.drawer_pos,
                    num_objects=self.num_objects,
                    min_distance_drawer = self.min_distance_drawer,
                    min_distance_container=self.min_distance_container,
                    min_distance_obj=self.min_distance_obj
                )
        return container_position, object_positions

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

        self.container_position, self.original_object_positions = self.generate_objects_positions()


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
        if self.random_tray:
            self.tray_position = np.random.uniform(
                low=self.tray_position_low, high=self.tray_position_high)
        
        # self.container_position, self.original_object_positions = self.generate_objects_positions()

        bullet.reset()
        bullet.setup_headless()
        self._load_meshes()
        bullet.reset_robot(
            self.robot_id,
            self.reset_joint_indices,
            self.reset_joint_values)
        self.is_gripper_open = True  # TODO(avi): Clean this up

        self.complete_tasks = 0
        self.subtasks = self.generate_tasks()
        self.current_task = self.subtasks.pop(0)

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

    def get_reward(self, info):
        # if current
        if self.reward_type == 'pick_place':
            reward = float(info['place_success_target'])
        elif self.reward_type == 'grasp':
            reward = float(info['grasp_success_target'])
        else:
            raise NotImplementedError
        return reward

    def get_info(self):
        object_name = self.current_task.info.object_name
        object_target = self.current_task.info.object_target
        
        info = AttrDict()
        info["reward"] = False

        info['grasp_success'] = False
        grasp_success = object_utils.check_grasp(
            object_name, self.objects, self.robot_id,
            self.end_effector_index, self.grasp_success_height_threshold,
            self.grasp_success_object_gripper_threshold)
        if grasp_success:
            info['grasp_success'] = True
            
        info['grasp_success_target'] = object_utils.check_grasp(
            object_name, self.objects, self.robot_id,
            self.end_effector_index, self.grasp_success_height_threshold,
            self.grasp_success_object_gripper_threshold)

        if self.current_task.type == "drawer_open" or self.current_task.type == "drawer_close":
        # info = super(Widow250TableEnv, self).get_info()
            info['drawer_x_pos'] = self.get_drawer_pos()[0]
            info['drawer_opened_percentage'] = \
                self.get_drawer_opened_percentage()
            info['drawer_closed_percentage'] = \
                1 - self.get_drawer_opened_percentage()
            info['drawer_opened_success'] = info["drawer_opened_percentage"] > \
                self.drawer_opened_success_thresh
            info['drawer_closed_success'] = info["drawer_opened_percentage"] < \
                self.drawer_closed_success_thresh

            if self.current_task.type == "drawer_open":
                if info['drawer_opened_success']:
                    info.reward = True
                    self.current_task.info.done = True
            elif self.current_task.type == "drawer_close":
                if info['drawer_closed_success']:
                    info.reward = True
                    self.current_task.info.done = True
        else:
            target_position = self.current_task.info.target_position
            # info['place_success'] = False
            # place_success = object_utils.check_in_container(
            #     object_name, self.objects, target_position,
            #     self.place_success_height_threshold,
            #     self.place_success_radius_threshold)
            # if place_success:
            #     info['place_success'] = place_success

            info['place_success_target'] = object_utils.check_in_container(
                object_name, self.objects, target_position,
                self.place_success_height_threshold,
                self.place_success_radius_threshold)
            
            if self.current_task.type == "pick_place":
                if info['place_success']:
                    info.reward = True
                    self.current_task.info.done = True
            
        return info
    
    def is_drawer_closed(self):
        info = self.get_info()
        return info['drawer_closed_success']
    
    def step(self, action):

        # TODO Clean this up
        if np.isnan(np.sum(action)):
            print('action', action)
            raise RuntimeError('Action has NaN entries')

        action = np.clip(action, -1, +1)  # TODO Clean this up

        xyz_action = action[:3]  # ee position actions
        abc_action = action[3:6]  # ee orientation actions
        gripper_action = action[6]
        neutral_action = action[7]

        ee_pos, ee_quat = bullet.get_link_state(
            self.robot_id, self.end_effector_index)
        joint_states, _ = bullet.get_joint_states(self.robot_id,
                                                  self.movable_joints)
        gripper_state = np.asarray([joint_states[-2], joint_states[-1]])

        target_ee_pos = ee_pos + self.xyz_action_scale * xyz_action
        ee_deg = bullet.quat_to_deg(ee_quat)
        target_ee_deg = ee_deg + self.abc_action_scale * abc_action
        target_ee_quat = bullet.deg_to_quat(target_ee_deg)

        if self.control_mode == 'continuous':
            num_sim_steps = self.num_sim_steps
            target_gripper_state = gripper_state + \
                                   [-self.gripper_action_scale * gripper_action,
                                    self.gripper_action_scale * gripper_action]

        elif self.control_mode == 'discrete_gripper':
            if gripper_action > 0.5 and not self.is_gripper_open:
                num_sim_steps = self.num_sim_steps_discrete_action
                target_gripper_state = GRIPPER_OPEN_STATE
                self.is_gripper_open = True  # TODO(avi): Clean this up

            elif gripper_action < -0.5 and self.is_gripper_open:
                num_sim_steps = self.num_sim_steps_discrete_action
                target_gripper_state = GRIPPER_CLOSED_STATE
                self.is_gripper_open = False  # TODO(avi): Clean this up
            else:
                num_sim_steps = self.num_sim_steps
                if self.is_gripper_open:
                    target_gripper_state = GRIPPER_OPEN_STATE
                else:
                    target_gripper_state = GRIPPER_CLOSED_STATE
                # target_gripper_state = gripper_state
        else:
            raise NotImplementedError

        target_ee_pos = np.clip(target_ee_pos, self.ee_pos_low,
                                self.ee_pos_high)
        target_gripper_state = np.clip(target_gripper_state, GRIPPER_LIMITS_LOW,
                                       GRIPPER_LIMITS_HIGH)
        # import pdb; pdb.set_trace()

        bullet.apply_action_ik(
            target_ee_pos, target_ee_quat, target_gripper_state,
            self.robot_id,
            self.end_effector_index, self.movable_joints,
            lower_limit=JOINT_LIMIT_LOWER,
            upper_limit=JOINT_LIMIT_UPPER,
            rest_pose=RESET_JOINT_VALUES,
            joint_range=JOINT_RANGE,
            num_sim_steps=num_sim_steps)

        if self.use_neutral_action and neutral_action > 0.5:
            if self.neutral_gripper_open:
                bullet.move_to_neutral(
                    self.robot_id,
                    self.reset_joint_indices,
                    RESET_JOINT_VALUES)
            else:
                bullet.move_to_neutral(
                    self.robot_id,
                    self.reset_joint_indices,
                    RESET_JOINT_VALUES_GRIPPER_CLOSED)

        info = self.get_info()
        reward = info.reward
        if self.current_task.info.done:
            self.current_task = self.subtasks.pop(0)
            self.complete_tasks += 1
        info["complete_tasks"] = self.complete_tasks
        done = False
        return self.get_observation(), reward, done, info

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
