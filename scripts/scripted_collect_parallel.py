import argparse
import time
import subprocess
import datetime
import os

from roboverse.utils import get_timestamp
import numpy as np
import os
import os.path as osp
import roboverse
from roboverse.policies import policies
from tqdm import tqdm
import h5py


SAVE_IMAGES = False 

def get_data_save_directory(args):
    data_save_directory = args.data_save_directory

    data_save_directory += '_{}'.format(args.env)

    if args.num_trajectories > 1000:
        data_save_directory += '_{}K'.format(int(args.num_trajectories/1000))
    else:
        data_save_directory += '_{}'.format(args.num_trajectories)

    if args.save_all:
        data_save_directory += '_save_all'

    data_save_directory += '_noise_{}'.format(args.noise)
    data_save_directory += '_{}'.format(get_timestamp())

    return data_save_directory


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--env", type=str, required=True)
    parser.add_argument("-pl", "--policy-name", type=str, required=True)
    parser.add_argument("-a", "--accept-trajectory-key", type=str, required=True)
    parser.add_argument("-n", "--num-trajectories", type=int, required=True)
    parser.add_argument("-t", "--num-timesteps", type=int, required=True)
    parser.add_argument("-d", "--data-save-directory", type=str, required=True)
    parser.add_argument("--save-all", action='store_true', default=False)
    parser.add_argument("--target-object", type=str, default="shed")
    parser.add_argument("-p", "--num-parallel-threads", type=int, default=10)
    parser.add_argument("--noise", type=float, default=0.1)
    parser.add_argument("-r", "--image-rendered", type=int, default=0)
    args = parser.parse_args()
    print(f"using {args.num_parallel_threads} threads")
    num_trajectories_per_thread = int(
        args.num_trajectories / args.num_parallel_threads)
    if args.num_trajectories % args.num_parallel_threads != 0:
        num_trajectories_per_thread += 1

    timestamp = get_timestamp()
    # save_directory = get_data_save_directory(args)
    # save_directory = osp.join(__file__, "../..", save_directory, f"p{args.num_parallel_threads}")
    # print(f"saving to: {save_directory}")
    commands = []
    script_name = "scripted_collect.py"
    for i in range(args.num_parallel_threads):
        save_directory = get_data_save_directory(args)
        # save_directory = osp.join(__file__, "../..", save_directory, f"p{i}")
        save_directory = os.path.join(os.environ['YUE_DATA_DIR'], 'roboverse',save_directory, f"p{i}")
        print(f"saving to: {save_directory}")
        command = ['python',
                'scripts/{}'.format(script_name),
                '--policy-name={}'.format(args.policy_name),
                '-a{}'.format(args.accept_trajectory_key),
                '-e{}'.format(args.env),
                '-n {}'.format(num_trajectories_per_thread),
                '-t {}'.format(args.num_timesteps),
                '-o{}'.format(args.target_object),
                '-d{}'.format(save_directory),
                '--image-rendered={}'.format(args.image_rendered),
                ]
        commands.append(command)

    if args.save_all:
        command.append('--save-all')

    subprocesses = []
    for i in range(args.num_parallel_threads):
        subprocesses.append(subprocess.Popen(commands[i]))
        time.sleep(1)

    exit_codes = [p.wait() for p in subprocesses]

    merge_command = ['python',
                     'scripts/combine_trajectories.py',
                     '-d{}'.format(save_directory)]

    subprocess.call(merge_command)

