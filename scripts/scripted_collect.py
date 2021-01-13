import numpy as np
import time
import os
import os.path as osp
import roboverse
from roboverse.policies import policies
import argparse
from tqdm import tqdm
import h5py

from roboverse.utils import get_timestamp
EPSILON = 0.1

SAVE_IMAGES = False      # if False, saves dummy image to save disk space

# TODO(avi): Clean this up
# NFS_PATH = '/nfs/kun1/users/avi/imitation_datasets/'
NFS_PATH = '/nfs/kun1/users/avi/imitation_datasets/'


def add_transition(traj, observation, action, reward, info, agent_info, done,
                   next_observation, img_dim, image_rendered=True):
    if image_rendered:
        observation["image"] = np.reshape(np.uint8(observation["image"] * 255.),
                                        (img_dim, img_dim, 3))
        next_observation["image"] = np.reshape(
            np.uint8(next_observation["image"] * 255.), (img_dim, img_dim, 3))
    traj["observations"].append(observation)
    traj["next_observations"].append(next_observation)
    traj["actions"].append(action)
    traj["rewards"].append(reward)
    traj["terminals"].append(done)
    traj["agent_infos"].append(agent_info)
    traj["env_infos"].append(info)
    return traj


def collect_one_traj(env, policy, num_timesteps, noise,
                     accept_trajectory_key, image_rendered):
    num_steps = -1
    rewards = []
    success = False
    img_dim = env.observation_img_dim
    env.reset()
    policy.reset()
    time.sleep(1)
    traj = dict(
        observations=[],
        actions=[],
        rewards=[],
        next_observations=[],
        terminals=[],
        agent_infos=[],
        env_infos=[],
    )
    is_opened = False
    is_closed = False
    for j in range(num_timesteps):

        action, agent_info = policy.get_action()

        # In case we need to pad actions by 1 for easier realNVP modelling
        env_action_dim = env.action_space.shape[0]
        if env_action_dim - action.shape[0] == 1:
            action = np.append(action, 0)
        action += np.random.normal(scale=noise, size=(env_action_dim,))
        action = np.clip(action, -1 + EPSILON, 1 - EPSILON)
        observation = env.get_observation()
        next_observation, reward, done, info = env.step(action)
        add_transition(traj, observation,  action, reward, info, agent_info,
                       done, next_observation, img_dim, image_rendered)
        
        if accept_trajectory_key == 'table_clean':
            info['table_clean'] = False
            if info['drawer_opened_success']:
                is_opened = True
            if is_opened:
                if info['drawer_closed_success']:
                    is_closed = True
            if is_opened and is_closed:
                info['table_clean'] = True
            # print(f"closed? {info['drawer_closed_success']} open? {info['drawer_opened_success']}")

        if info[accept_trajectory_key] and num_steps < 0:
            num_steps = j
        if info[accept_trajectory_key]:
            success = True

        rewards.append(reward)
        if done or agent_info['done']:
            break
        


    return traj, success, num_steps


def dump2h5(traj, path, image_rendered):
    """Dumps a collected trajectory to HDF5 file."""
    # convert to numpy arrays
    states = np.array([o['state'] for o in traj['observations']])
    if image_rendered:
        images = np.array([o['image'] for o in traj['observations']])
    actions = np.array(traj['actions'])
    rewards = np.array(traj['rewards'])
    terminals = np.array(traj['terminals'])

    # create HDF5 file
    f = h5py.File(path, "w")
    f.create_dataset("traj_per_file", data=1)

    # store trajectory info in traj0 group
    traj_data = f.create_group("traj0")
    traj_data.create_dataset("states", data=states)
    if image_rendered:
        traj_data.create_dataset("images", data=images, dtype=np.uint8)
    # else:
        # traj_data.create_dataset("images", data=np.zeros((images.shape[0], 2, 2, 3), dtype=np.uint8))
    traj_data.create_dataset("actions", data=actions)
    traj_data.create_dataset("rewards", data=rewards)

    if np.sum(terminals) == 0:
        terminals[-1] = True

    # build pad-mask that indicates how long sequence is
    is_terminal_idxs = np.nonzero(terminals)[0]
    pad_mask = np.zeros((len(terminals),))
    pad_mask[:is_terminal_idxs[0]] = 1.
    traj_data.create_dataset("pad_mask", data=pad_mask)

    f.close()


def main(args):

    timestamp = get_timestamp()
    if osp.exists(NFS_PATH):
        data_save_path = osp.join(NFS_PATH, args.save_directory)
    else:
        data_save_path = osp.join(__file__, "../..", "data", args.save_directory)
    data_save_path = osp.abspath(data_save_path)
    if not osp.exists(data_save_path):
        os.makedirs(data_save_path)

    env = roboverse.make(args.env_name,
                         gui=args.gui,
                         transpose_image=False)

    data = []
    assert args.policy_name in policies.keys(), f"The policy name must be one of: {policies.keys()}"
    # assert args.accept_trajectory_key in env.get_info().keys(), \
    #     f"""The accept trajectory key must be one of: {env.get_info().keys()}"""
    policy_class = policies[args.policy_name]
    policy = policy_class(env)
    num_success = 0
    num_saved = 0
    num_attempts = 0
    accept_trajectory_key = args.accept_trajectory_key

    progress_bar = tqdm(total=args.num_trajectories)
    while num_saved < args.num_trajectories:
        num_attempts += 1
        traj, success, num_steps = collect_one_traj(
            env, policy, args.num_timesteps, args.noise,
            accept_trajectory_key, args.image_rendered)

        if success:
            if args.gui:
                print("num_timesteps: ", num_steps)
            data.append(traj)
            dump2h5(traj, os.path.join(data_save_path, 'rollout_{}.h5'.format(num_saved)),
                        args.image_rendered)
            num_success += 1
            num_saved += 1
            progress_bar.update(1)
        elif args.save_all:
            data.append(traj)
            num_saved += 1
            progress_bar.update(1)

        if args.gui:
            print("success rate: {}".format(num_success/(num_attempts)))

    progress_bar.close()
    print("success rate: {}".format(num_success / (num_attempts)))
    path = osp.join(data_save_path, "scripted_{}_{}.npy".format(
        args.env_name, timestamp))
    print(path)
    np.save(path, data)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--env-name", type=str, required=True)
    parser.add_argument("-pl", "--policy-name", type=str, required=True)
    parser.add_argument("-a", "--accept-trajectory-key", type=str, required=True)
    parser.add_argument("-n", "--num-trajectories", type=int, required=True)
    parser.add_argument("-t", "--num-timesteps", type=int, required=True)
    parser.add_argument("--save-all", action='store_true', default=False)
    parser.add_argument("--gui", action='store_true', default=False)
    parser.add_argument("-o", "--target-object", type=str)
    parser.add_argument("-d", "--save-directory", type=str, default=""),
    parser.add_argument("--noise", type=float, default=0.1)
    parser.add_argument("-r", "--image-rendered", type=int, default=1)
    
    args = parser.parse_args()

    main(args)
