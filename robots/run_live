# This part should probably be in main.py, where we then select the `run` script (run_live, run_scenario) from.
# It adds UST0 folder to the sytem path enabling relative imports.
# import sys
# import git
# import os

# ust0_path = git.Repo('.', search_parent_directories=True).working_tree_dir
# sys.path.append(ust0_path)

# os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
#################################################################################

import importlib
import time
# import rospy, all instances of rospy commented
import signal
import numpy as np
import sys

# import robots.ros as ros
# import robots.duckietown as duckietown, all instances of rospy commented
import robots.optitrack as optitrack
# from robots.old_controllers import constant_linear_vel, intersection_demo
from robots.path_generation import PathGenerator
from utils.game import Game


# TODO: custom ROS messages are in ~/catkin_ws/ make script to generate workspace? 

class LiveSimulator:
    def __init__(self, params):

        # Params given by ConfigReader in main.py        
        self.params = params


        self.desktop_hostname = self.params['robots']['desktop_hostname']        
        self.current_map = self.params['robots']['experiment_map']
        self.ranges = self.params['robots']['map_ranges'][self.current_map]
        self.delta_t = 1.0 / self.params['main']['sample_freq']
        self.game = None
        

        # Generating tracked vehicles dictionary
        # keys are integers corresponding to the rigid body id
        ## TODO: [naming] tracked or controlled vehciles?
        self.controlled_vehicles = dict()
        
        ## Duckiebots, name convention="tdXX"
        # self.controlled_vehicles.update(
        #     {int(vehicle_name[2:5]): duckietown.Duckiebot(vehicle_name)  
        #         for vehicle_name in self.params['robots']['controlled_vehicles'] if vehicle_name[0:2] == "td"}
        # )

        # ## Jetracers, name convention="jrXX"
        # self.controlled_vehicles.update(
        #     {int(vehicle_name[2:5]+50): jetracer.Jetracer(vehicle_name)  
        #         for vehicle_name in self.params['robots']['controlled_vehicles'] if vehicle_name[0:2] == "jr"}
        # )

        ########################################

        
        self.predetermined_paths = self.params['robots']['pre_determined_paths']


        # Holds pose history for each vehicle
        self.vehicles_traces = {vehicle: [] for vehicle in self.controlled_vehicles.keys()}

        # Checking wheter we need to start rosbridge (roscore is handled at system level)
        # ros.handle_rosbridge()

        # # Starting UST0 ROS node
        # desktop_hostname_ROScompliant = self.desktop_hostname.replace("-", "") # can't contain "-"
        # # rospy.init_node(f"{desktop_hostname_ROScompliant}_ust0")
        # print(f">> Started `{desktop_hostname_ROScompliant}_ust0` node.")
        # time.sleep(2)


        # Initiliazing OptiTrack Class & NatNet Client 
        self.optitrack = optitrack.OptiTrack(self.params['robots'])


        # Generating pre-determined paths
        self.path_generator = PathGenerator(self.params['robots'])   # fetch from .pre_determined_paths using path name

        self.goal_paths = {vehicle: self.path_generator.pre_determined_paths[self.predetermined_paths[index]]
                        for index, vehicle in enumerate(self.controlled_vehicles.keys())}



        # Setting up Controller(s)
        ## currently same controller for all vehicles)
        controller_name = self.params['robots']['controller']
        self.controller_type = self.params['robots']['controller_type']
        controller_params = self.params['robots']['controller_settings'][controller_name]

        #print(f"controller_params = {controller_params}")

        # Get the correct controller class based on controller name string
        controller_class = getattr(importlib.import_module("robots.old_controllers"), controller_name)

        # # Instantiating controller class(es)
        # # TODO: make function for this, different controller take different arguments for initialization
        # if self.controller_type == "individual":
        #     self.controllers = {vehicle: controller_class(self.params, vehicle)
        #                                 for vehicle in self.controlled_vehicles.keys()}
        
        # elif self.controller_type == "central":

        #     if controller_name == "intersection_demo" or controller_name == "constant_linear_vel":
        #         self.controller = controller_class(
        #             controller_params, self.params['robots'], self.path_generator,
        #             self.controlled_vehicles, self.goal_paths, self.delta_t
        #         )


        # self.background_data = None

        # while True:
        #     user_command = input("\n>> Please prepare the physical environment for the experiment.\n>> Enter 's' when ready to run UST0 or 'q' to quit: ")
        #     if user_command.lower() == "s":
        #         break
        #     elif user_command.lower() == "q":
        #         self.safe_program_shutdown()

        # TODO: change all units from CM to meters
        # TODO: whenever involves controlled_vehicles check if type is duckiebot or jetracer; AND/OR try to make command like publish_cmd() independent of robot type
        # i.e. by using linear v and steering angle for all robot types

        # 1 = scale; TODO: make math according to map range and size of car
        self.game = Game('live', 1, self.ranges, self.params, self.background_data)

        # TODO: assigned car images can be changed (for live at least),
        # the objects in self.tracked vehicles already contain the class + dimensions, 
        # which is all the info used by self.vehicles_Data

        # dummy to be removed latter after changing assign_car_images()
        vehicles_data = {vehicle: {'class': self.controlled_vehicles[vehicle].vehicle_type, 'dimensions': self.controlled_vehicles[vehicle].dimensions} for vehicle in self.controlled_vehicles.keys()}
        self.game.assign_car_images(self.controlled_vehicles, vehicles_data)

    
    def safe_program_shutdown(self, sig=None, frame=None):
        """ Handles stopping robots and properly ending the program. """

        print("\n>> Shutting down.")
        for vehicle in self.controlled_vehicles:
            self.controlled_vehicles[vehicle].publish_cmd(0.0, 0.0)

        time.sleep(1)

        self.optitrack.natnet_client.stop()
        # rospy.signal_shutdown("Shutting down") # TODO: is this needed?

        # Game only exists after accepting the user prompt
        if self.game:
            if sig: self.game.close_window()
            self.game.close()

        sys.exit(0)

    # TODO: change all log messages (print statements) to be from Debug: Info: or Error:

    def run_game(self):
        """ Function that we loop over during experiment.
        """

        # Checks for CTRL+C inputs
        signal.signal(signal.SIGINT, self.safe_program_shutdown)

        while not self.game.close_window():

            # Updating vehicles' pose
            ## TODO: do i have to return dict again or just update its variables inside the function?
            self.controlled_vehicles = self.optitrack.update_poses(self.controlled_vehicles)
            
            # Append pose to pose history
            for vehicle in self.vehicles_traces:
                #print(f"appending to pose -> {np.append(self.controlled_vehicles[vehicle].position, self.controlled_vehicles[vehicle].angle)}")
                self.vehicles_traces[vehicle].append(
                    np.append(self.controlled_vehicles[vehicle].position, self.controlled_vehicles[vehicle].angle)
                )

            # Individual Control Step
            if self.controller_type == "individual":
                for vehicle in self.controlled_vehicles:
                    self.controllers[vehicle].control_step()

            # Central Control Step
            if self.controller_type == "central":
                self.controller.control_step()

            # TODO: do the controllers also publish command to vehicle or is that in run_live.py
            # Friday: controllers .py saves to "cmd" variable in duckie class and the run_live published cmd

            # TODO: multiply pos by 100 (like before) inside game.draw_quick for scale? Sure should change plotting
            # to be based on meters by default
            
            # TODO: clean arguments passed into draw_quick
            self.game.draw_quick(self.controlled_vehicles, self.vehicles_traces, self.goal_paths, self.controller)

        # When game window is closed
        self.safe_program_shutdown()

if __name__ == '__main__':
    Sim = LiveSimulator()
    Sim.run_game()