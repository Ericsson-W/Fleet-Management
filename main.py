import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"

from pathlib import Path

from utils.read_configs import ConfigReader
from robots.run_live import LiveSimulator
from simulator.run_scenario import run as run_scenario

while not os.path.isfile('main.py'):
    par_dir = Path(os.getcwd()).parent
    os.chdir(par_dir)

root_dir = os.getcwd()

params = ConfigReader(root_dir).params

if params['main']['run_live']:
    sim = LiveSimulator(params)
    sim.run_game() 
else:
    run_scenario(params) # must test with osc as well
