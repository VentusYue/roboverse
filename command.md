# example command
python scripts/scripted_collect.py -n 100 -t 30 -e Widow250DoubleDrawerOpenNeutral-v0 -pl drawer_open_transfer -a drawer_opened_success --noise=0.1 --gui

# pickplace
python scripts/scripted_collect.py -n 100 -t 30 -e Widow250PickPlace-v0 -pl pickplace -a place_success_target --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 30 -e Widow250PlaceTray-v0 -pl pickplace -a place_success_target --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 30 -e Widow250SinglePutInBowl-v0 -pl pickplace_open -a place_success_target --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 30 -e Widow250SinglePutInBowlRandomBowlPosition-v0 -pl pickplace_old -a place_success_target --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 30 -e Widow250MultiShedPutInMultiBowlTest-v0 -pl pickplace -a place_success_target --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 30 -e Widow250MultiShedPutInMultiBowlTest-v0 -pl pickplace_open -a place_success_target --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 100 -e Widow250MultiShedPutInMultiBowlTest-v0 -pl pickplace_open_suboptimal -a place_success_target --noise=0.1 --gui

python scripts/scripted_collect.py -n 30 -t 200 -e Widow250PickPlaceMultiObjectMultiContainerTrain-v0 -pl pickplace -a place_success_target --noise=0.1 --gui


AssertionError: The policy name must be one of: dict_keys(['grasp', 'grasp_transfer', 'grasp_transfer_suboptimal', 'pickplace', 'pickplace_open', 'drawer_open', 'button_press', 'drawer_open_transfer', 'place', 'drawer_close_open_transfer', 'drawer_open_transfer_suboptimal', 'drawer_close_open_transfer_suboptimal', 'pickplace_open_suboptimal'])

# grasp
python scripts/scripted_collect.py -n 100 -t 30 -e Widow250GraspEasy-v0 -pl grasp_transfer -a grasp_success_target --noise=0.1 --gui

python scripts/scripted_collect.py -n 30 -t 30 -e Widow250MultiThreeObjectGraspTrain-v0 -pl grasp_transfer -a grasp_success_target --noise=0.1 --gui

# drawer open
python scripts/scripted_collect.py -n 100 -t 50 -e Widow250DrawerTestPos-v0 -pl drawer_open -a drawer_opened_success --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 30 -e Widow250DoubleDrawerPickPlaceOpenGraspNeutral-v0 -pl pickplace_open -a drawer_opened_success --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 30 -e Widow250DoubleDrawerPickPlaceOpenGraspNeutral-v0 -pl drawer_close_open_transfer -a drawer_opened_success --noise=0.1 --gui

## double drawer, close the upper one and open the botton one
python scripts/scripted_collect.py -n 100 -t 30 -e Widow250DoubleDrawerCloseOpenNeutral-v0 -pl drawer_close_open_transfer -a drawer_opened_success --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 30 -e Widow250DoubleDrawerCloseOpenNeutral-v0 -pl drawer_close -a drawer_opened_success --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 30 -e Widow250DrawerTestPos-v0 -pl drawer_close -a drawer_opened_success --noise=0.1 --gui

## pick object and open the drawer
python scripts/scripted_collect.py -n 100 -t 50 -e Widow250DoubleDrawerPickPlaceOpenGraspNeutral-v0 -pl pickplace_open -a drawer_opened_success --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 50 -e Widow250DoubleDrawerOpenGraspNeutral-v0 -pl pickplace_open -a drawer_opened_success --noise=0.1 --gui

# pick place three objects, drawer + container
python scripts/scripted_collect.py -n 100 -t 150 -e Widow250TableCleanTest-v1 -pl tableclean -a drawer_opened_success --noise=0.1 --gui


python scripts/scripted_collect.py -n 100 -t 300 -e Widow250TableCleanTest-v2 -pl tableclean -a drawer_opened_success --noise=0.1 --gui


python scripts/scripted_collect.py -n 100 -t 320 -e Widow250TableCleanTest-v2 -pl tableclean -a drawer_closed_success --noise=0.1 --gui

# test pickplace
python scripts/scripted_collect.py -n 100 -t 40 -e TestWidow250PickPlaceTray-v0 -pl pickplace -a place_success_target --noise=0.1 --gui

# test drawer
python scripts/scripted_collect.py -n 100 -t 50 -e Widow250DrawerTestPos-v0 -pl drawer_open -a drawer_opened_success --noise=0.1 --gui
