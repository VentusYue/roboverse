import numpy as np
import roboverse.bullet as bullet


class DrawerOpen:

    def __init__(self, env, xyz_action_scale=7.0,
                    return_origin_thresh=0.1):
        self.env = env
        self.xyz_action_scale = xyz_action_scale
        self.gripper_dist_thresh = 0.06
        self.gripper_xy_dist_thresh = 0.03
        self.ending_height_thresh = 0.2
        self.return_base_thresh = 0.4
        
        self.done = False

        self.return_origin_thresh = return_origin_thresh
        self.reset()

    def reset(self):
        # self.dist_thresh = 0.06 + np.random.normal(scale=0.01)
        self.drawer_never_opened = True
        self.done = False
        offset_coeff = (-1) ** (1 - self.env.left_opening)
        self.handle_offset = np.array([offset_coeff * 0.01, 0.0, -0.01])

    def get_action(self):
        ee_pos, _ = bullet.get_link_state(
            self.env.robot_id, self.env.end_effector_index)
        handle_pos = self.env.get_drawer_handle_pos() + self.handle_offset
        gripper_handle_dist = np.linalg.norm(handle_pos - ee_pos)
        gripper_handle_xy_dist = np.linalg.norm(handle_pos[:2] - ee_pos[:2])
        done = False
        if (gripper_handle_xy_dist > self.gripper_xy_dist_thresh
                and not self.env.is_drawer_open()):
            # print('xy - approaching handle')
            action_xyz = (handle_pos - ee_pos) * self.xyz_action_scale
            action_xyz = list(action_xyz[:2]) + [0.]  # don't droop down.
            action_angles = [0., 0., 0.]
            action_gripper = [0.0]
        elif (gripper_handle_dist > self.gripper_dist_thresh
                and not self.env.is_drawer_open()):
            # print("moving down toward handle")
            action_xyz = (handle_pos - ee_pos) * self.xyz_action_scale
            action_angles = [0., 0., 0.]
            action_gripper = [0.0]
        elif not self.env.is_drawer_open():
            # print("opening drawer")
            x_command = (-1) ** (1 - self.env.left_opening)
            action_xyz = np.array([x_command, 0, 0])
            # action = np.asarray([0., 0., 0.7])
            action_angles = [0., 0., 0.]
            action_gripper = [0.0]
        elif (np.abs(ee_pos[2] - self.ending_height_thresh) >
                self.gripper_dist_thresh):
            if (np.abs(ee_pos[2] - self.ending_height_thresh) < self.return_base_thresh):
                action_xyz = [0., 0., 0.]
                action_angles = [0., 0., 0.]
                action_gripper = [0.]
                done = True
                self.done = True
            else:
                self.drawer_never_opened = False
                action_xyz = np.array([0, 0, 0.7])  # force upward action
                action_angles = [0., 0., 0.]
                action_gripper = [0.0]
        else:
            action_xyz = [0., 0., 0.]
            action_angles = [0., 0., 0.]
            action_gripper = [0.0]

        # if done:
        #     if np.linalg.norm(ee_pos - self.env.ee_pos_init) < self.return_origin_thresh:
        #         self.done = done
        #     else:
        #         action_xyz = (self.env.ee_pos_init - ee_pos) * self.xyz_action_scale
        #         # print(ee_pos, self.env.ee_pos_init)
        #         # print(np.linalg.norm(ee_pos - self.env.ee_pos_init)) 
        
        agent_info = dict(done=self.done)
        action = np.concatenate((action_xyz, action_angles, action_gripper))
        return action, agent_info


class DrawerClose:

    def __init__(self, env, xyz_action_scale=5.0, return_origin_thresh=0.1):
        self.env = env
        self.xyz_action_scale = xyz_action_scale
        self.gripper_dist_thresh = 0.06
        self.gripper_xy_dist_thresh = 0.03
        self.ending_z = -0.25
        self.top_drawer_offset = np.array([0, 0, 0.02])

        self.done = False
        self.begin_closing = False

        self.return_origin_thresh = return_origin_thresh
        self.reset()

    def reset(self):
        self.drawer_never_opened = True
        offset_coeff = (-1) ** (1 - self.env.left_opening)
        self.handle_offset = np.array([offset_coeff * 0.01, 0.0, -0.01])
        self.reached_pushing_region = False
        self.neutral_taken = False
        self.begin_closing = False
        self.done = False


    def get_action(self):
        ee_pos, _ = bullet.get_link_state(
            self.env.robot_id, self.env.end_effector_index)
        handle_pos = self.env.get_drawer_handle_pos() + self.handle_offset
        gripper_handle_dist = np.linalg.norm(handle_pos - ee_pos)
        gripper_handle_xy_dist = np.linalg.norm(handle_pos[:2] - ee_pos[:2])
        top_drawer_pos = self.env.get_drawer_pos("drawer")
        # top_drawer_push_target_pos = (
        #     top_drawer_pos + np.array([0.15, 0., 0.05]))

        top_drawer_push_target_pos = (
            top_drawer_pos + np.array([0.2, 0.1, 0.05]))
        # print(f"top_drawer_push_target_pos: {top_drawer_push_target_pos}")
        is_gripper_ready_to_push = (
            ee_pos[0] > top_drawer_push_target_pos[0] and
            ee_pos[2] < top_drawer_push_target_pos[2] 
        )
        done = False
        neutral_action = [0.0]
        if (not self.env.is_drawer_closed() and
                not self.reached_pushing_region and
                not is_gripper_ready_to_push):
            # print("move up and left")
            action_xyz = [0.15, -0.1, -0.15]
            action_angles = [0., 0., 0.]
            action_gripper = [0.0]
        elif not self.env.is_drawer_closed():
            # print("close top drawer")
            self.reached_pushing_region = True
            action_xyz = (top_drawer_pos + self.top_drawer_offset - ee_pos) * 7.0
            action_xyz[0] *= 3
            action_xyz[1] *= 0.6
            action_angles = [0., 0., 0.]
            action_gripper = [-0.7] #0.
            self.begin_closing = True
        if self.env.is_drawer_closed() and self.begin_closing:
            action_xyz = [0., 0., 0.]
            action_angles = [0., 0., 0.]
            action_gripper = [0.]
            done = True
        
        if done:
            if np.linalg.norm(ee_pos - self.env.ee_pos_init) < self.return_origin_thresh:
                self.done = done
            else:
                action_xyz = (self.env.ee_pos_init - ee_pos) * self.xyz_action_scale
                # print(ee_pos, self.env.ee_pos_init)
                # print(np.linalg.norm(ee_pos - self.env.ee_pos_init)) 

        # print(ee_pos, top_drawer_push_target_pos)
        agent_info = dict(done=self.done)
        action = np.concatenate((action_xyz, action_angles, action_gripper, neutral_action))
        return action, agent_info