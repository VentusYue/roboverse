# example command
python scripts/scripted_collect.py -n 100 -t 30 -e Widow250DoubleDrawerOpenNeutral-v0 -pl drawer_open_transfer -a drawer_opened_success --noise=0.1 --gui

# pickplace
python scripts/scripted_collect.py -n 100 -t 30 -e Widow250PickPlace-v0 -pl pickplace -a place_success_target --noise=0.1 --gui

# grasp
python scripts/scripted_collect.py -n 100 -t 30 -e Widow250GraspEasy-v0 -pl grasp_transfer -a grasp_success_target --noise=0.1 --gui

# drawer open
python scripts/scripted_collect.py -n 100 -t 50 -e Widow250DrawerTestPos-v0 -pl drawer_open -a drawer_opened_success --noise=0.1 --gui

## double drawer, close the upper one and open the botton one
python scripts/scripted_collect.py -n 100 -t 30 -e Widow250DoubleDrawerCloseOpenNeutral-v0 -pl drawer_close_open_transfer -a drawer_opened_success --noise=0.1 --gui

## pick object and open the drawer
python scripts/scripted_collect.py -n 100 -t 50 -e Widow250DoubleDrawerPickPlaceOpenGraspNeutral-v0 -pl pickplace_open -a drawer_opened_success --noise=0.1 --gui

# pick place three objects, drawer + container
python scripts/scripted_collect.py -n 100 -t 150 -e Widow250TableCleanTest-v1 -pl tableclean -a drawer_opened_success --noise=0.1 --gui

# test pickplace
python scripts/scripted_collect.py -n 100 -t 40 -e TestWidow250PickPlaceTray-v0 -pl pickplace -a place_success_target --noise=0.1 --gui

# test drawer
python scripts/scripted_collect.py -n 100 -t 50 -e Widow250DrawerTestPos-v0 -pl drawer_open -a drawer_opened_success --noise=0.1 --gui

# table clean 3 objects fixed
python scripts/scripted_collect.py -n 100 -t 320 -e Widow250TableCleanTest-v2 -pl tableclean -a drawer_closed_success --noise=0.1 --gui

# table clean 2 objects random 
python scripts/scripted_collect.py -n 100 -t 250 -e Widow250TableCleanObjects2Random-v0 -pl tableclean -a drawer_closed_success --noise=0.1 --gui

# table clean 2 objects random, random tray position
python scripts/scripted_collect.py -n 100 -t 250 -e Widow250TableCleanObjects2RandomTray-v0 -pl tableclean -a drawer_closed_success --noise=0.1 --gui

# table clean 3 objects random 
python scripts/scripted_collect.py -n 100 -t 350 -e Widow250TableCleanObjects3Random-v2 -pl tableclean -a table_clean --noise=0.1 

<!-- # table clean 4 objects random, take too much time to generate
python scripts/scripted_collect.py -n 100 -t 400 -e Widow250TableCleanObjects4Random-v0 -pl tableclean -a table_clean --noise=0.1 --gui -->
python scripts/scripted_collect.py -n 100 -t 350 -e Widow250TableCleanObjects3RandomNoimage-v0 -pl tableclean -a table_clean --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 350 -e Widow250TableCleanObjects3FixedNoimage-v0 -pl tableclean -a table_clean --noise=0.1 --gui


# table clean 3 objects random parallel, render image
python scripts/scripted_collect_parallel.py -p 32 -n 100 -t 350 -e Widow250TableCleanObjects3RandomNoimage-v0 -pl tableclean -a table_clean --noise=0.1 -d data 

# table clean 3 objects random parallel, no render image
python scripts/scripted_collect_parallel.py -p 8 -n 16000 -t 350 -e Widow250TableCleanObjects3RandomNoimage-v0 -pl tableclean -a table_clean --noise=0.1 -d tableclean


python scripts/scripted_collect_parallel.py -p 8 -n 16000 -t 30 -e Widow250GraspEasy-v0 -pl grasp_transfer -a grasp_success_target --noise=0.1  -d reachgrasp

python scripts/scripted_collect_parallel.py -p 8 -n 16000 -t 30 -e Widow250PickPlace-v0 -pl pickplace -a place_success_target --noise=0.1 -d pickplace

python scripts/scripted_collect_parallel.py -p 8 -n 16000 -t 50 -e Widow250DrawerOpen-v0 -pl drawer_open -a drawer_opened_success --noise=0.1 -d draweropen

python scripts/scripted_collect.py -n 100 -t 350 -e Widow250TableCleanObjects3RandomNoimage-v0 -pl tableclean -a table_clean --noise=0.1 --gui
