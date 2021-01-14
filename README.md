# roboverse
A set of environments utilizing [pybullet](https://github.com/bulletphysics/bullet3) was simulation of robotic manipulation tasks. 

## Usage
Creating and using environments is simple:
```python
import roboverse
env = roboverse.make('Widow250DoubleDrawerOpenNeutral-v0', gui=True)
env.reset()
for _ in range(25):
    env.step(env.action_space.sample())
```
## Setup
I recommend using [conda](https://docs.anaconda.com/anaconda/install/) for setup:

```
conda create -n roboverse python=3.6
source activate roboverse
pip install -r requirements.txt
```
When using this repository with other projects, run `pip install -e .` in the root directory of this repo. 

To test if things are working by visualizing a scripted robot policy, run the following command:

`python scripts/scripted_collect.py -n 100 -t 30 -e Widow250DoubleDrawerOpenNeutral-v0 -pl drawer_open_transfer -a drawer_opened_success --noise=0.1 --gui`

## If you want to dig into the code, start here:
`python roboverse/envs/widow250.py`

## Credit
Primary developers: [Avi Singh](https://www.avisingh.org/), Albert Yu, Jonathan Yang, [Michael Janner](https://people.eecs.berkeley.edu/~janner/), Huihan Liu, Gaoyue Zhou

## Table clean Environment 
Download extra objects from: https://drive.google.com/file/d/1LO479WorPHhRZnw9U5FWpzg51EELcSIF/view?usp=sharing

To test the table clean environment, run the following command:

`python scripts/scripted_collect.py -n 100 -t 300 -e Widow250TableCleanTest-v2 -pl tableclean -a drawer_opened_success --noise=0.1 --gui`

To modify the configs, check:

`roboverse/envs/regieration.py, Widow250TableCleanTest`

To modify the sequence of pick place/drawer open/close, check:

`roboverse/policies/table_clean.py`

## commands for environment collection 
### table clean 3 objects random parallel, with image 
'python scripts/scripted_collect_parallel.py -p 10 -n 100 -t 350 -e Widow250TableCleanObjects3Random-v2 -pl tableclean -a table_clean --noise=0.1 -d data'

### table clean 3 objects random parallel, no image rendering
python scripts/scripted_collect_parallel.py -p 32 -n 16000 -t 350 -e Widow250TableCleanObjects3RandomNoimage-v0 -pl tableclean -a table_clean --noise=0.1 -d data

### table clean 2 objects random,
'python scripts/scripted_collect_parallel.py -p 10 -n 100 -t 250 -e Widow250TableCleanObjects2RandomTray-v0 -pl tableclean -a table_clean --noise=0.1 -d data'

For more commands, check command.md, 
