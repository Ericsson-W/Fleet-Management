import configparser
from pathlib import Path
import os

class ConfigReader:
    def __init__(self, root_dir, config_file_name='config.ini'):
        """Read main parameters from config.ini"""

        self.root_dir = Path(root_dir)

        self.config_file_name = config_file_name

        config_file_path = Path.joinpath(self.root_dir, 'robots', self.config_file_name)

        # Get the absolute path of the config file
        abs_config_file_path = os.path.abspath(config_file_path)


        # self.config = configparser.RawConfigParser()
        # self.config.read(config_file_path)          



        self.config = configparser.RawConfigParser()
        self.config.read(abs_config_file_path)

        main_params = self.get_config_dict('Main Parameters',
                                        {'run_live': bool,
                                            'screen_width': int,
                                            'screen_height': int,
                                            'screen_scale': int,
                                            'sample_freq': int,
                                            'path_images': str,
                                            'path_background': str,
                                            'path_output': str})

        self.params = {'main': main_params}

        # TODO: fix this
        # if self.params['main']['run_live']:
        self.params.update({'robots': self.read_robots()})
        # # else:
        # self.params.update({'simulator': self.read_sim()})

        # self.params.update({'model': self.read_models()})

    def read_robots(self):

        config_file_path = Path.joinpath(self.root_dir, 'robots', self.config_file_name)

        self.config = configparser.RawConfigParser()
        self.config.read(config_file_path)

        robot_params = self.get_config_dict('Robot Parameters',
                                            {'server_ip': str,
                                             'multicast_ip': str,
                                             'command_port': int,
                                             'data_port': int,
                                             'desktop_hostname': str,
                                             'controlled_vehicles': eval,  # lists
                                             'pre_determined_paths': eval,  # lists
                                             'path_loop': bool,
                                             'path_points_per_m': float,
                                             'current_map': str,
                                             'map_ranges': eval,  # dict
                                             'outer_radius': float,
                                             'inner_radius': float,
                                             'experiment_map': str,
                                             'controller': str,
                                             'controller_type': str,
                                             'controller_settings': eval,  # dict
                                             })

        return robot_params

    # def read_sim(self):

    #     config_file_path = Path.joinpath(self.root_dir, 'simulator', self.config_file_name)

    #     self.config = configparser.RawConfigParser()
    #     self.config.read(config_file_path)

    #     sim_params = self.get_config_dict('Simulator Parameters',
    #                                       {'path_data': str,
    #                                        'visualization': bool,
    #                                        'data_type': str,
    #                                        'scenario_id': str,  # Not int here, it's a name
    #                                        'scenario': int,
    #                                        'start_time': int,
    #                                        'end_time': int,
    #                                        'ego_vehicle_id': int,
    #                                        'ego_perspective': bool,
    #                                        'stochastic': bool,
    #                                        'col_pred_time': float,
    #                                        'controllable_agent': bool,
    #                                        'rl_agent': bool,
    #                                        'rl_model': str,
    #                                        'ego_position': bool,
    #                                        'ego_perception': bool,
    #                                        'zoom_to_ego': bool,
    #                                        'path_lanelets': str,
    #                                        'car_length': float,
    #                                        'car_width': float,
    #                                        'perception_r': float,
    #                                        'discrete_acceleration': int,
    #                                        'grid_length': float,
    #                                        'grid_divisions': int
    #                                        })

    #     return sim_params

    # def read_models(self):

    #     config_file_path = Path.joinpath(self.root_dir, 'models', self.config_file_name)

    #     self.config = configparser.RawConfigParser()
    #     self.config.read(config_file_path)

    #     model_parameters = self.get_config_dict('Model Parameters',
    #                                             {'pred_ignore': float,
    #                                              'var_lon': float,
    #                                              'var_steer': float,
    #                                              'var_p': float,
    #                                              'var_v': float
    #                                              })

    #     return model_parameters

    def get_config_dict(self, section_name, type_dict):
        new_dict = dict(self.config.items(section_name))
        return_dict = {}
        for key, value in new_dict.items():
            if type_dict[key] is bool and value == 'False':  # Unfortunately any string returns True
                return_dict[key] = False
            else:
                return_dict[key] = type_dict[key](value)

        return return_dict





### Old version from extract_scenario, do not delete until comprehensive testing done, may contain useful parameters.
def read_config_old(config_file_path='config.ini'):
    """Read parameters from config.ini"""
    config = configparser.RawConfigParser()
    config.read(config_file_path)
    config_dict_data = dict(config.items('Path data'))
    path_data = config_dict_data['path_data']
    scene_params = dict(config.items('Scenario parameters'))
    visual_params = dict(config.items('Visual parameters'))
    car_params = dict(config.items('Car parameters'))
    stochastic_params = dict(config.items('Stochastic parameters'))
    grid_params = dict(config.items('Grid parameters'))
    scene_params['images'] = eval(scene_params['images'])
    scene_params['scenario_ID'] = scene_params['scenario_id']
    scene_params['scenario'] = int(scene_params['scenario'])
    scene_params['start_time'] = int(scene_params['start_time'])
    scene_params['end_time'] = int(scene_params['end_time'])
    scene_params['ego_name'] = scene_params['ego_name']
    scene_params['ego_perspective'] = eval(scene_params['ego_perspective'])
    scene_params['stochastic'] = eval(scene_params['stochastic'])
    scene_params['col_pred_time'] = float(scene_params['col_pred_time'])
    scene_params['sample_freq'] = float(scene_params['sample_freq'])
    scene_params['rl_agent'] = eval(scene_params['rl_agent'])
    scene_params['rl_model'] = scene_params['rl_model']
    visual_params['ego_position'] = eval(visual_params['ego_position'])
    visual_params['ego_perception'] = eval(visual_params['ego_perception'])
    visual_params['agent_perception'] = eval(visual_params['agent_perception'])
    visual_params['screen_width'] = int(visual_params['screen_width'])
    visual_params['screen_height'] = int(visual_params['screen_height'])
    visual_params['screen_scale'] = float(visual_params['screen_scale'])
    visual_params['path_images'] = visual_params['path_images']
    visual_params['path_background'] = visual_params['path_background']
    visual_params['single_quadrant'] = eval(visual_params['single_quadrant'])
    car_params['length'] = float(car_params['length'])
    car_params['width'] = float(car_params['width'])
    car_params['perception_r'] = float(car_params['perception_r'])
    stochastic_params['pred_ignore'] = float(stochastic_params['pred_ignore'])
    stochastic_params['var_lon'] = float(stochastic_params['var_lon'])
    stochastic_params['var_steer'] = float(stochastic_params['var_steer'])
    stochastic_params['var_p'] = float(stochastic_params['var_p'])
    stochastic_params['var_v'] = float(stochastic_params['var_v'])
    grid_params['length'] = float(grid_params['length'])
    grid_params['divisions'] = int(grid_params['divisions'])

    params = {
        'scene': scene_params,
        'visual': visual_params,
        'car': car_params,
        'stochastic': stochastic_params,
        'grid': grid_params
    }

    return path_data, params