# office task
python scripts/scripted_collect.py -n 100 -t 350 -e Widow250OfficePickPlaceRandom-v0 -pl tableclean -a table_clean --noise=0.1 --gui
python scripts/scripted_collect.py -n 100 -t 350 -e Widow250OfficePickPlaceFixed-v0 -pl tableclean -a table_clean --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 300 -e Widow250OfficePickPlaceSimpleRandom-v0 -pl tableclean -a table_clean --noise=0.1 --gui 

python scripts/scripted_collect.py -n 100 -t 280 -e Widow250OfficePickPlaceSimpleFixed-v0 -pl tableclean -a table_clean --noise=0.05 --gui 

python scripts/scripted_collect.py -n 100 -t 350 -e Widow250OfficePickPlaceRandom-v0 -pl tableclean -a table_clean --noise=0.1 --gui

python scripts/scripted_collect.py -n 100 -t 280 -e Widow250OfficePickPlaceFixed-v0 -pl tableclean -a table_clean --noise=0.1 --gui -f 1

# office data collection
python scripts/scripted_collect_parallel.py -p 12 -n 12000 -t 350 -e Widow250OfficePickPlaceFixed-v0 -pl tableclean -a table_clean --noise=0.1 -d office-v2

python scripts/scripted_collect_parallel.py -p 12 -n 24000 -t 350 -e Widow250OfficePickPlaceRandom-v0 -pl tableclean -a table_clean --noise=0.1 -d office-v1

python scripts/scripted_collect_parallel.py -p 12 -n 24000 -t 300 -e Widow250OfficePickPlaceSimpleRandom-v0 -pl tableclean -a table_clean --noise=0.1 -d office-two-tasks-v0


python scripts/scripted_collect_parallel.py -p 12 -n 12000 -t 300 -e Widow250OfficePickPlaceSimpleFixed-v0 -pl tableclean -a table_clean --noise=0.1 -d office-simple-fixed-v0


python scripts/scripted_collect_parallel.py -p 8 -n 12000 -t 350 -e Widow250OfficePickPlaceRandom-v0 -pl tableclean -a table_clean --noise=0.15 -d office-v2-noise-0.15

python scripts/scripted_collect_parallel.py -p 12 -n 12000 -t 350 -e Widow250OfficePickPlaceRandom-v0 -pl tableclean -a table_clean --noise=0.15 -d office-v2-noise-0.15

python scripts/scripted_collect_parallel.py -p 8 -n 12000 -t 350 -e Widow250OfficePickPlaceFixed-v0 -pl tableclean -a table_clean --noise=0.05 -d office-v2-fixed-noise-0.05-full-reward -f 1


python scripts/scripted_collect_parallel.py -p 12 -n 16000 -t 260 -e Widow250OfficePickPlaceRandom-v0 -pl tableclean -a table_clean --noise=0.1 -d office-v3 -f 1

python scripts/scripted_collect_parallel.py -p 8 -n 12000 -t 260 -e Widow250OfficePickPlaceFixed-v0 -pl tableclean -a table_clean --noise=0.1 -d office-v3 -f 1


python scripts/scripted_collect_parallel.py -p 12 -n 16000 -t 280 -e Widow250OfficePickPlaceRandom-v0 -pl tableclean -a table_clean --noise=0.1 -d office-v4 -f 1