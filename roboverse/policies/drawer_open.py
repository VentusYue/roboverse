import numpy as np
import roboverse.bullet as bullet


class DrawerOpen:

    def __init__(self, env):
        self.env = env
        self.xyz_action_scale = 7.0
        self.gripper_dist_thresh = 0.06
        self.ending_height_thresh = 0.2
        self.handle_offset = np.array([0.01, 0, -0.01])
        self.reset()

    def reset(self):
        # self.dist_thresh = 0.06 + np.random.normal(scale=0.01)
        self.drawer_never_opened = True

    def get_action(self):
        ee_pos, _ = bullet.get_link_state(
            self.env.robot_id, self.env.end_effector_index)
        handle_pos = self.env.get_drawer_handle_pos() + self.handle_offset
        gripper_handle_dist = np.linalg.norm(handle_pos - ee_pos)
        print("gripper_handle_dist", gripper_handle_dist)
        done = False

        if (gripper_handle_dist > self.gripper_dist_thresh
                and not self.env.is_drawer_open()):
            # print('approaching handle')
            action_xyz = (handle_pos - ee_pos) * 7.0
            action_angles = [0., 0., 0.]
            action_gripper = [0.0]
        elif not self.env.is_drawer_open():
            # print("opening drawer")
            action_xyz = np.array([1, 0, 0])
            # action = np.asarray([0., 0., 0.7])
            action_angles = [0., 0., 0.]
            action_gripper = [0.0]
        elif (np.abs(ee_pos[2] - self.ending_height_thresh) >
                self.gripper_dist_thresh):
            # print("Lift upward")
            self.drawer_never_opened = False
            action_xyz = np.array([0, 0, 0.7])  # force upward action
            action_angles = [0., 0., 0.]
            action_gripper = [0.0]
        else:
            action_xyz = [0., 0., 0.]
            action_angles = [0., 0., 0.]
            action_gripper = [0.0]

        agent_info = dict(done=done)
        action = np.concatenate((action_xyz, action_angles, action_gripper))
        return action, agent_info
