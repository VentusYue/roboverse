# office task
python scripts/scripted_collect.py -n 100 -t 350 -e Widow250OfficePickPlaceRandom-v0 -pl tableclean -a table_clean --noise=0.1 --gui
python scripts/scripted_collect.py -n 100 -t 350 -e Widow250OfficePickPlaceFixed-v0 -pl tableclean -a table_clean --noise=0.1 --gui

# office data collection
python scripts/scripted_collect_parallel.py -p 8 -n 8000 -t 350 -e Widow250OfficePickPlaceFixed-v0 -pl tableclean -a table_clean --noise=0.1 -d Fixed

python scripts/scripted_collect_parallel.py -p 12 -n 24000 -t 350 -e Widow250OfficePickPlaceRandom-v0 -pl tableclean -a table_clean --noise=0.1 -d statefixed


python scripts/scripted_collect.py -n 100 -t 350 -e Widow250OfficeDrawerPickPlaceFixed-v0 -pl tableclean -a table_clean --noise=0.1 --gui