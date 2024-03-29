import glob
from pathlib import Path
import numpy as np
from numpy.lib.shape_base import tile
import pygame
import math
# from pyproj import Transformer
from pygame import Vector2
# from utils.pre_processing.osc_data.osm_background_generation import \
    # get_offset, extents_merc_to_gps, extents_gps_to_merc, background_from_limits
# from utils.pre_processing.osc_data.osm_background_generation import \
    #  merc_x_old as merc_x, merc_y_old as merc_y

# import utils.pre_processing.scaling as sc


class Game:
    """
    Class to draw game with cars and collision prediction grid
    """

    def __init__(self, mode="data", *args):

        scale, ranges, params, background_data = args

        self.scale = scale
        self.ranges = ranges
        self.params = params

        # Set screen and game params
        self.car_image_size = 128
        self.screen_width = params['main']['screen_width']
        self.screen_height = params['main']['screen_height']
        self.path_images = params['main']['path_images']
        self.path_image_ego = ''
        self.bg_center = np.array([0, 0])
        self.offset = Vector2([0, 0])

        self.mode = mode

        # TODO: instead of live and what not, just use value of params['main']['run_live']
        if self.mode == 'live':
            # Temporary hacks to make live version work

            params['main']['sample_freq'] = 25  # Manual for now
            params['main']['dt'] = 1 / params['main']['sample_freq']
            
            self.params['simulator']['grid_padding'] = 0
            self.params['simulator']['grid_length'] = 0
            self.params['simulator']['padding'] = int(np.ceil(self.params['simulator']['car_length']))

            self.background_im_name = 'duckietown_' + self.params['robots']['experiment_map'] + '.png'

            background_data = self.background_im_name

        # Pygame initialization
        pygame.init()
        pygame.display.set_caption("Scenario trajectories")
        pygame.font.init()
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        self.myfont = pygame.font.SysFont('Arial', 14)
        self.clock = pygame.time.Clock()
        self.ticks = params['main']['sample_freq']
        self.exit = False

        # Background
        self.bg, self.bg_shift, self.bg_rect = self.load_background(background_data)
        if self.params['simulator']['ego_perspective']:
            self.bg_center = self.get_background_center()
        self.bg_rot = self.bg

    def close(self):
        """Quit pygame if window is closed"""
        pygame.quit()

    def close_window(self):
        """Quit pygame if window is closed"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return True

        return False

    def get_pressed(self):
        """Return pressed key"""
        return pygame.key.get_pressed()

    ###################################################################################################################
    ################################################## Load images ####################################################
    ###################################################################################################################

    # def load_background(self, background_data):
    #     """Load and transform background image"""
    #     if self.mode == "data":
    #         if self.params['simulator']['data_type'] == 'InD':

    #             min_x, max_x, min_y, max_y = self.ranges
    #             grid_shift = self.params['simulator']['padding'] * self.params['simulator']['grid_length']
    #             bg_path = self.params['main']['path_background'] + self.params['simulator'][
    #                 'scenario_id'] + '_background.png'
    #             bg = pygame.image.load(bg_path).convert()

    #             width_dict = {'00': 196,
    #                           '08': 114,
    #                           '18': 114}  # Visually estimated parameters

    #             est_image_width_for_data = width_dict[self.params['simulator']['scenario_id']]
    #             ratio = float(est_image_width_for_data / bg.get_width())
    #             data_image_width = bg.get_width() * ratio
    #             data_image_height = bg.get_height() * ratio
    #             x_shift = 0  # - 1
    #             y_shift = 0  # -1
    #             image_corners_data_frame = (-33.897 + x_shift, -24.790 + y_shift), (
    #                 107.918 + x_shift, 212.309 + y_shift)  # Manually assigned based on image construction
    #             single_quadrant = True  # Assume true while using InD, other stuff will break before this

    #             if single_quadrant:
    #                 # Get coordinates in data frame
    #                 left_edge = 0
    #                 top_edge = 0
    #                 right_edge = data_image_width
    #                 bottom_edge = data_image_height
    #             else:
    #                 (left_edge, top_edge), (right_edge, bottom_edge) = image_corners_data_frame

    #             right_edge_transformed = int((right_edge - min_x + grid_shift) * self.scale)
    #             bottom_edge_transformed = int(-(-bottom_edge - min_y + grid_shift) * self.scale + self.screen_height)
    #             left_edge_transformed = int((left_edge - min_x + grid_shift) * self.scale)
    #             top_edge_transformed = int(-(-top_edge - min_y + grid_shift) * self.scale + self.screen_height)
    #             far_corner = (right_edge_transformed, bottom_edge_transformed)
    #             origin_corner = (left_edge_transformed, top_edge_transformed)
    #             width = int(far_corner[0] - origin_corner[0])
    #             height = int(far_corner[1] - origin_corner[1])
    #             bg = pygame.transform.scale(bg, (width, height))
    #             origin_corner = (origin_corner[0], origin_corner[1])
    #             self.image_offset = origin_corner
    #             bg_rect = bg.get_rect()
    #             self.load_road_key(width, height)

    #         elif self.params['simulator']['data_type'] == 'OSC':

    #             new_version = False
    #             # Change to True, potentially delete old version is Felix makes appropriate change to UST1 code.
    #             # Otherwise this should work as a stopgap measure.

    #             if new_version:

    #                 min_x, max_x, min_y, max_y = self.ranges
    #                 grid_shift = self.params['simulator']['padding'] * self.params['simulator']['grid_length']

    #                 data_extents, image_extents, bg_path = background_data

    #                 x_offset, y_offset = get_offset(limits=data_extents)

    #                 offsets = (x_offset, y_offset)

    #                 # Orders for tuples
    #                 ## min_x, max_x, min_y, max_y
    #                 ## min_lat, min_long, max_lat, max_long

    #                 vehicles_extents = extents_merc_to_gps(self.ranges, offsets)

    #                 all_extents = vehicles_extents, data_extents, image_extents

    #                 min_lat = min([extents[0] for extents in all_extents])
    #                 min_long = min([extents[1] for extents in all_extents])
    #                 max_lat = max([extents[2] for extents in all_extents])
    #                 max_long = max([extents[3] for extents in all_extents])

    #                 largest_extents = (min_lat, min_long, max_lat, max_long)

    #                 # Calculate new extents of the image, note this will not work if Felix has not updated area generation code
    #                 # It is fixable but can't remember currently how I did it.

    #                 new_im_extents = background_from_limits(largest_extents, bg_path)

    #                 bg = pygame.image.load(bg_path).convert()

    #                 min_x_im, max_x_im, min_y_im, max_y_im = extents_gps_to_merc(new_im_extents, offsets)

    #                 left_edge = min_x_im
    #                 top_edge = max_y_im
    #                 right_edge = max_x_im
    #                 bottom_edge = min_y_im

    #                 right_edge_transformed = int((right_edge - min_x + grid_shift) * self.scale)
    #                 bottom_edge_transformed = int(-(bottom_edge - min_y + grid_shift) * self.scale + self.screen_height)
    #                 left_edge_transformed = int((left_edge - min_x + grid_shift) * self.scale)
    #                 top_edge_transformed = int(-(top_edge - min_y + grid_shift) * self.scale + self.screen_height)
    #                 far_corner = (right_edge_transformed, bottom_edge_transformed)
    #                 origin_corner = (left_edge_transformed, top_edge_transformed)
    #                 width = int(abs(far_corner[0] - origin_corner[0]))
    #                 height = int(abs(far_corner[1] - origin_corner[1]))

    #                 bg = pygame.transform.scale(bg, (width, height))
    #                 origin_corner = (origin_corner[0], origin_corner[1])
    #                 self.image_offset = origin_corner
    #                 bg_rect = bg.get_rect()

    #             else:

    #                 # Old Version (works if Felix does not change scaling)

    #                 min_x, max_x, min_y, max_y = self.ranges
    #                 grid_shift = self.params['simulator']['padding'] * self.params['simulator']['grid_length']

    #                 data_extents, image_extents, bg_path = background_data

    #                 min_lat, min_long, max_lat, max_long = image_extents

    #                 gps_to_merc = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    #                 merc_to_gps = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)

    #                 true_min_x, true_min_y = gps_to_merc.transform(min_long, min_lat)
    #                 true_max_x, true_max_y = gps_to_merc.transform(max_long, max_lat)
    #                 # new_long, new_lat = merc_to_gps.transform(x, y)


    #                 # As Qiming's converter code is bespoke, reversing it is harder than perhaps it should be.
    #                 # As a result, we take the shift of the relative values and apply it to the true ones to get the true lat/long range.

    #                 x_offset, y_offset = get_offset(limits=data_extents)

    #                 min_x_im = merc_x(min_long, x_offset)
    #                 max_x_im = merc_x(max_long, x_offset)
    #                 min_y_im = merc_y(min_lat, y_offset)
    #                 max_y_im = merc_y(max_lat, y_offset)

    #                 min_x_offset = min(min_x, min_x_im) - min_x_im
    #                 max_x_offset = max(max_x, max_x_im) - max_x_im
    #                 min_y_offset = min(min_y, min_y_im) - min_y_im
    #                 max_y_offset = max(max_y, max_y_im) - max_y_im

    #                 final_min_long, final_min_lat = merc_to_gps.transform(true_min_x + min_x_offset,
    #                                                                       true_min_y + min_y_offset)
    #                 final_max_long, final_max_lat = merc_to_gps.transform(true_max_x + max_x_offset,
    #                                                                       true_max_y + max_y_offset)

    #                 new_extents = (final_min_lat, final_min_long, final_max_lat, final_max_long)

    #                 # Calculate new extents of the image

    #                 new_im_extents = background_from_limits(new_extents, bg_path)

    #                 bg = pygame.image.load(bg_path).convert()

    #                 min_lat, min_long, max_lat, max_long = new_im_extents

    #                 min_x_im = merc_x(min_long, x_offset)
    #                 max_x_im = merc_x(max_long, x_offset)
    #                 min_y_im = merc_y(min_lat, y_offset)
    #                 max_y_im = merc_y(max_lat, y_offset)

    #                 left_edge = min_x_im
    #                 top_edge = max_y_im
    #                 right_edge = max_x_im
    #                 bottom_edge = min_y_im

    #                 """
    #                 # From OSM Mercator
    #                 # R = 6378137.0
    #                 # def y2lat(y):
    #                 #     return math.degrees(2 * math.atan(math.exp(y / R)) - math.pi / 2.0)
    #                 #
    #                 # def lat2y(lat):
    #                 #     return math.log(math.tan(math.pi / 4 + math.radians(lat) / 2)) * R
    #                 #
    #                 # def x2lng(x):
    #                 #     return math.degrees(x / R)
    #                 #
    #                 # def lon2x(lon):
    #                 #     return math.radians(lon) * R
    #                 #
    #                 # min_x_im = lon2x(min_long) - x_offset
    #                 # max_x_im = lon2x(max_long) - x_offset
    #                 # min_y_im = lat2y(min_lat) - y_offset
    #                 # max_y_im = lat2y(max_lat) - y_offset
    #                 #
    #                 # left_edge = min_x_im
    #                 # top_edge = min_y_im
    #                 # right_edge = max_x_im
    #                 # bottom_edge = max_y_im
    #                 #
    #                 # print(left_edge, top_edge, right_edge, bottom_edge)

    #                 ## Easiest, but requires pyproj

    #                 # gps_to_merc = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    #                 #
    #                 # min_x_im, min_y_im = gps_to_merc.transform(min_long, min_lat)
    #                 # max_x_im, max_y_im = gps_to_merc.transform(max_long, max_lat)
    #                 #
    #                 # left_edge = min_x_im - x_offset
    #                 # top_edge = min_y_im - y_offset
    #                 # right_edge = max_x_im - x_offset
    #                 # bottom_edge = max_y_im - y_offset
    #                 #
    #                 # print(left_edge, top_edge, right_edge, bottom_edge)


    #                 # print(x2lng(min_x), y2lat(min_y))
    #                 #
    #                 # print(min_long, min_lat)
    #                 #
    #                 # m_2_g = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)
    #                 #
    #                 # print(m_2_g.transform(min_x+x_offset, min_y+y_offset))
    #                 #
    #                 # print(min_long, min_lat)
    #                 #
    #                 # breakpoint()
    #                 """

    #                 right_edge_transformed = int((right_edge - min_x + grid_shift) * self.scale)
    #                 bottom_edge_transformed = int(-(bottom_edge - min_y + grid_shift) * self.scale + self.screen_height)
    #                 left_edge_transformed = int((left_edge - min_x + grid_shift) * self.scale)
    #                 top_edge_transformed = int(-(top_edge - min_y + grid_shift) * self.scale + self.screen_height)
    #                 far_corner = (right_edge_transformed, bottom_edge_transformed)
    #                 origin_corner = (left_edge_transformed, top_edge_transformed)
    #                 width = int(abs(far_corner[0] - origin_corner[0]))
    #                 height = int(abs(far_corner[1] - origin_corner[1]))

    #                 bg = pygame.transform.scale(bg, (width, height))
    #                 origin_corner = (origin_corner[0], origin_corner[1])
    #                 self.image_offset = origin_corner
    #                 bg_rect = bg.get_rect()

    #                 print(left_edge, top_edge, right_edge, bottom_edge)


    #                 """R = 6378137.0

    #                 def y2lat(y):
    #                     return math.degrees(2 * math.atan(math.exp(y / R)) - math.pi / 2.0)

    #                 def lat2y(lat):
    #                     return math.log(math.tan(math.pi / 4 + math.radians(lat) / 2)) * R

    #                 def x2lng(x):
    #                     return math.degrees(x / R)

    #                 def lon2x(lon):
    #                     return math.radians(lon) * R"""

    #         return bg, origin_corner, bg_rect

    #     elif self.mode == 'live':

    #         grid_shift = self.params['simulator']['grid_padding'] * self.params['simulator']['grid_length']

    #         bg_path = self.params['main']['path_background'] + background_data

    #         bg = pygame.image.load(bg_path).convert()

    #         # Regular axis, positive x, negative y

    #         tile_size_cm = 59.5

    #         image_width_cm = tile_size_cm * 5 # Actual image width as pertains to real world scale

    #         custom_origin_point = (0,0)

    #         left_edge, top_edge = custom_origin_point

    #         right_edge = left_edge + image_width_cm
    #         bottom_edge = top_edge - image_width_cm

    #         self.ranges = left_edge, right_edge, bottom_edge, top_edge

    #         self.update_scale()

    #         min_x, max_x, min_y, max_y = self.ranges

    #         # custom_origin_point = (-15, 52)  # Found by inspection: NEED A BETTER WAY OF DOING THIS
    #         # custom_scale = 0.45
    #         # left_edge, top_edge = custom_origin_point
    #         #
    #         # right_edge = left_edge + bg.get_width() * custom_scale
    #         # bottom_edge = top_edge + bg.get_height() * custom_scale

    #         grid_shift = 0
    #         left_edge_transformed = int((left_edge - min_x + grid_shift) * self.scale)
    #         top_edge_transformed = int(-(top_edge - min_y + grid_shift) * self.scale + self.screen_height)

    #         right_edge_transformed = int((right_edge - min_x + grid_shift) * self.scale)
    #         bottom_edge_transformed = int(-(bottom_edge - min_y + grid_shift) * self.scale + self.screen_height)


    #         origin_corner = (left_edge_transformed, top_edge_transformed) # should be 0, screen height (if making all positive)
    #         far_corner = (right_edge_transformed, bottom_edge_transformed) # +ve width, +ve height

    #         width = int(abs(far_corner[0] - origin_corner[0]))
    #         height = int(abs(far_corner[1] - origin_corner[1]))

    #         bg = pygame.transform.scale(bg, (width, height))

    #         self.image_offset = origin_corner
    #         bg_rect = bg.get_rect()

    #         #self.load_road_key(width, height, bg_im = bg)

    #     return bg, origin_corner, bg_rect

    # def load_road_key(self, width, height, bg_im = None):
    #     """Load and transform background key image for road identification"""

    #     if self.params['simulator']['data_type'] == 'InD':

    #         key_path = self.params['main']['path_background'] + self.params['simulator'][
    #             'scenario_id'] + '_background_road_key.png'
    #         try:
    #             bg_key = pygame.image.load(key_path).convert()
    #         except FileNotFoundError:
    #             key_path = self.params['main']['path_background'] + 'black_background.png'
    #             bg_key = pygame.image.load(key_path).convert()
    #         bg_key = pygame.transform.scale(bg_key, (width, height))
    #         # bg_key_rect = bg_key.get_rect()
    #         self.bg_key_gt = bg_key
    #         self.bg_key = self.bg_key_gt

    #     elif self.params['simulator']['data_type'] == 'OSC':

    #         self.bg_key_gt = bg_im
    #         self.bg_key = self.bg_key_gt

    # def transform_background_ego(self, bg, ego):
    #     """Rotate background image ego on pivot point for ego perspective"""
    #     # Corner shift
    #     bg_center_orig = self.bg_center
    #     bg_center_ego = bg_center_orig - ego.position
    #     pivot = np.array([self.screen_width / 2, self.screen_height * 3 / 4])
    #     bg_center_ego_sca_x = bg_center_ego[0] * self.scale + pivot[0]
    #     bg_center_ego_sca_y = -bg_center_ego[1] * self.scale + pivot[1]
    #     bg_center_ego_sca = np.array([bg_center_ego_sca_x, bg_center_ego_sca_y])

    #     # Rotate around pivot
    #     angle = -ego.angle * 180 / np.pi + 90
    #     bg_rot = pygame.transform.rotate(bg, angle)  # Rotate the image.
    #     offset_arr = bg_center_ego_sca - pivot
    #     self.offset.update(offset_arr.tolist())
    #     rot_offset = self.offset.rotate(-angle)  # Rotate the offset vector.
    #     bg_rect_rot = bg_rot.get_rect(center=pivot + rot_offset)
    #     bg_shift = bg_rect_rot.topleft
    #     self.bg_key = pygame.transform.rotate(self.bg_key_gt, angle)  # Rotate the image.

    #     return bg_rot, bg_shift

    # def get_background_center(self):
    #     """Calculate original background center"""
    #     grid_shift = self.params['simulator']['grid_padding'] * self.params['simulator']['grid_length']
    #     min_x = self.ranges[0]
    #     min_y = self.ranges[2]

    #     # Find original background center position
    #     bg_orig_x = (self.bg_shift[0] + self.bg_rect.width / 2) / self.scale + min_x - grid_shift
    #     bg_orig_y = -(self.bg_shift[1] + self.bg_rect.height / 2 - self.params['main'][
    #         'screen_height']) / self.scale + min_y - grid_shift

    #     return np.array([bg_orig_x, bg_orig_y])

    # def load_car_images(self):
    #     """Load images of cars"""
    #     image_paths = {}
    #     for vehicle_set in Path(self.path_images).iterdir():
    #         paths_list = glob.glob(self.path_images + vehicle_set.name + "/*")
    #         for p in paths_list:
    #             if 'ego' in p:
    #                 paths_list.remove(p)
    #                 self.path_image_ego = p

    #         image_paths[vehicle_set.name] = paths_list
    #     self.vehicles_image_loops = {class_name: {'max': len(images), 'current': 0} for class_name, images in
    #                                  image_paths.items()}

    #     self.vehicles_images = {}

    #     return image_paths

    # def assign_car_images(self, vehicles, vehicles_data):

    #     """Assign image to car"""
    #     # Load car images
    #     raw_images = self.load_car_images()
    #     for v in vehicles:
    #         # Images will be slightly too thin as the data does not account for wing mirrors but image size does. This
    #         # shouldn't be a major issue.
    #         v_class = vehicles_data[v]['class']
    #         if v == self.params['simulator']['ego_vehicle_id'] and self.path_image_ego != '':
    #             base_image = pygame.image.load(self.path_image_ego)
    #             i = 0
    #         else:
    #             i = self.vehicles_image_loops[v_class]['current']
    #             base_image = pygame.image.load(raw_images[v_class][i])
    #         v_length, v_width = vehicles_data[v]['dimensions']
    
    #         # TODO: giving all the dimensions in meters should be fine
    #         if self.params['main']['run_live']:
    #             v_length *= 100
    #             v_width *= 100

    #         width = int(v_length * self.scale)  # Width here is image width, not car width
    #         height = int(v_width * self.scale)  # Similarly, image height is vehicle width
    #         height = max(height, 1)  # Otherwise bikes and pedestrians crash the system by being 0 size.
    #         width = max(width, 1)

    #         self.vehicles_images[v] = pygame.transform.scale(base_image, (width, height))
    #         if i == self.vehicles_image_loops[v_class]['max'] - 1:
    #             self.vehicles_image_loops[v_class]['current'] = 0
    #         elif v != self.params['simulator']['ego_vehicle_id']:
    #             self.vehicles_image_loops[v_class]['current'] += 1

    # def import_lanelet_data(self, all_lanelet_data):
    #     self.lanelet_plot_data, self.centreline_data, self.graph_data = all_lanelet_data

    # ###################################################################################################################
    # ############################################# Draw objects on screen  #############################################
    # ###################################################################################################################

    # def draw_collision_grid(self, grid_col_draw):
    #     """Draw collision prediction grid"""
    #     filled = 0
    #     width = self.scale * self.params['simulator']['grid_length']
    #     height = width
    #     if np.max(grid_col_draw) > 0.000001:
    #         for x in range(grid_col_draw.shape[0]):
    #             for y in range(grid_col_draw.shape[1]):
    #                 # Do not draw small probabilities – too slow and can't really be seen anyway
    #                 if grid_col_draw[x, y] >= 0.000001:
    #                     if self.params['simulator']['data_type'] == 'InD':
    #                         key_val = self.bg_key.get_at((int(x * width) - self.image_offset[0], int(y * height) - self.image_offset[1]))[0]
    #                         # key_val = 255
    #                         if key_val >= 250:
    #                             red = int(min(255, 255 * np.power(grid_col_draw[x, y], 1 / 3)))
    #                             pygame.draw.rect(self.screen, [red, 0, 0], [x * width, y * height, width, height], filled)
    #                     else:
    #                         red = int(min(255, 255 * np.power(grid_col_draw[x, y], 1 / 3)))
    #                         pygame.draw.rect(self.screen, [red, 0, 0], [x * width, y * height, width, height], filled)

    # def draw_perception_grid(self, grid_per):
    #     """Draw perception grid"""
    #     filled = 1
    #     width = self.scale * self.params['simulator']['grid_length']
    #     height = width
    #     for x in range(grid_per.shape[0]):
    #         for y in range(grid_per.shape[1]):
    #             if grid_per[x, y] == 1:
    #                 green = 255  # * self.grid_per[t2][x, y]
    #                 pygame.draw.rect(self.screen, [0, green, 0], [x * width, y * height, width, height], filled)
    #             if grid_per[x, y] == -1:
    #                 rgb = [153, 50, 204]  # purple
    #                 pygame.draw.rect(self.screen, rgb, [x * width, y * height, width, height], filled)

    # def draw_cars(self, t, vehicles, cars_scaled, agent_scaled):
    #     """Draw car images"""
    #     veh_list = list(vehicles.keys())

    #     if self.params['simulator']['controllable_agent']:
    #         veh_list.remove(self.params['simulator']['ego_vehicle_id'])
    #         rotated_agent = pygame.transform.rotate(self.vehicles_images[self.params['simulator']['ego_vehicle_id']],
    #                                                 -agent_scaled.angle * 180 / np.pi)
    #         rect_agent = rotated_agent.get_rect()
    #         self.screen.blit(rotated_agent,
    #                          agent_scaled.position - np.array([rect_agent.width / 2, rect_agent.height / 2]))
    #     for v in veh_list:
    #         if not any(np.isnan(vehicles[v]['Position'][t])):
    #             rotated = pygame.transform.rotate(self.vehicles_images[v], -cars_scaled[v].angle * 180 / np.pi)
    #             rect = rotated.get_rect()
    #             self.screen.blit(rotated, cars_scaled[v].position - np.array([rect.width / 2, rect.height / 2]))
    #             if True:  # Draw vehicle ID
    #                 text_v = self.myfont.render(str(v), False, (255, 255, 255))
    #                 self.screen.blit(text_v, cars_scaled[v].position - np.array([rect.width / 2, rect.height / 2]))

    # def draw_trajectory(self, t, vehicles, v_name):
    #     """Draw vehicle trajectory between time t and prediction time"""
    #     t2 = int(t + self.params['simulator']['col_pred_time'] / self.params['main']['dt'])
    #     t2 = min(t2, len(vehicles[v_name]['Time']))
    #     interval = int(1 / self.params['main']['dt'])
    #     if t + interval < t2:
    #         trajectory = vehicles[v_name]['Position'][range(t, t2, interval)]
    #         trajectory = trajectory[~np.isnan(trajectory).any(axis=1)]
    #         if len(trajectory) >= 2:
    #             color = [0, 50, 200]
    #             if v_name == 'Ego':
    #                 color = [0, 255, 255]
    #             pygame.draw.lines(self.screen, color, False, trajectory, width=3)

    # def draw_agent_trajectory(self, agent, agent_pos_xy):
    #     """Draw controllable agent trajectory and path"""
    #     # predicted trajectory
    #     for pos_pred in agent_pos_xy:
    #         x, y = pos_pred
    #         pygame.draw.rect(self.screen, (255, 0, 50), pygame.Rect(x, y, 3, 3))
    #     # actual path
    #     for pos in agent.path:
    #         if self.params['simulator']['ego_perspective']:
    #             x, y = sc.scale_ego_coordinates(pos, agent.position, agent.angle, self.scale, self.params)
    #         else:
    #             x, y = sc.scale_coordinates(pos, self.ranges, self.scale, self.params)
    #         pygame.draw.rect(self.screen, (255, 255, 0), pygame.Rect(x, y, 3, 3))

    # def draw_text_box(self, t, agent, vehicles_orig, collision, control_info):
    #     """Draw text box on bottom left corner with ego dynamics at current timestep"""
    #     pygame.draw.rect(self.screen, [255, 255, 255],
    #                      [self.screen_width * 8 / 10, self.screen_height * 7.8 / 10, self.screen_width * 2 / 10,
    #                       self.screen_height * 2.2 / 10], 0)
    #     if self.params['simulator']['controllable_agent']:
    #         pos = np.round_(agent.position, 2)
    #         vel = np.round_(agent.velocity, 2)
    #         acc = np.round_(agent.acceleration, 2)
    #         angle = np.round_(agent.angle, 2)
    #         steering = np.round_(agent.steering, 2)
    #     else:
    #         pos = np.round_(vehicles_orig[self.params['simulator']['ego_vehicle_id']]['Position'][t], 2)
    #         vel = np.round_(vehicles_orig[self.params['simulator']['ego_vehicle_id']]['Velocity'][t], 2)
    #         acc = np.round_(vehicles_orig[self.params['simulator']['ego_vehicle_id']]['Acceleration'][t], 2)
    #         angle = np.round_(vehicles_orig[self.params['simulator']['ego_vehicle_id']]['Angle'][t], 2)
    #         steering = np.round_(vehicles_orig[self.params['simulator']['ego_vehicle_id']]['Angle_Steering'][t], 2)
    #     text_pos = self.myfont.render('Position: ' + str(pos), False, (0, 0, 0))
    #     self.screen.blit(text_pos, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 + 10))
    #     text_vel = self.myfont.render('Velocity: ' + str(vel), False, (0, 0, 0))
    #     self.screen.blit(text_vel, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 + 10 + 20))
    #     text_acc = self.myfont.render('Acceleration: ' + str(acc), False, (0, 0, 0))
    #     self.screen.blit(text_acc, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 + 10 + 40))
    #     text_time = self.myfont.render('Angle: ' + str(angle), False, (0, 0, 0))
    #     self.screen.blit(text_time, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 + 10 + 60))
    #     text_time = self.myfont.render('Steering: ' + str(steering), False, (0, 0, 0))
    #     self.screen.blit(text_time, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 + 10 + 80))
    #     text_time = self.myfont.render('Frame: ' + str(t), False, (0, 0, 255))
    #     self.screen.blit(text_time, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 + 10 + 100))
    #     if collision:
    #         text_collision = self.myfont.render('COLLISION!', False, (255, 0, 0))
    #         self.screen.blit(text_collision, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 - 10))
    #     if control_info is not None:
    #         q_values, action, pedal_intensity = control_info
    #         text_action = self.myfont.render('Action: ' + str(action), False, (200, 30, 0))
    #         self.screen.blit(text_action, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 + 10 + 120))
    #         text_pedal = self.myfont.render('Pedal: ' + str(pedal_intensity), False, (200, 30, 0))
    #         self.screen.blit(text_pedal, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 + 10 + 140))
    #         if q_values is not None:
    #             q_values = np.round_(q_values, 2)
    #             text_q = self.myfont.render('Action values: ' + str(q_values), False, (200, 30, 0))
    #             self.screen.blit(text_q, (self.screen_width * 8 / 10 + 10, self.screen_height * 8 / 10 + 10 + 160))

    # def scale_points_to_screen(self, pos_values):
    #     min_x, max_x, min_y, max_y = self.ranges

    #     grid_shift = self.params['simulator']['padding'] * self.params['simulator']['grid_length']

    #     x, y = pos_values

    #     x_coord = (x - min_x + grid_shift) * self.scale
    #     y_coord = -(y - min_y + grid_shift) * self.scale + self.screen_height

    #     return x_coord, y_coord

    # def draw_lanelets(self):

    #     def arrow(screen, lcolor, tricolor, start, end, trirad, thickness, side):
    #         rad = math.pi / 180
    #         pygame.draw.line(screen, lcolor, start, end, thickness)
    #         rotation = (math.atan2(start[1] - end[1], end[0] - start[0])) + math.pi / 2

    #         perp_shift = 8

    #         if side == 'left':
    #             rot = -math.pi / 2
    #         elif side == 'right':
    #             rot = math.pi / 2
    #         elif side == 'centre':
    #             rot = 0
    #             perp_shift = 0

    #         perp = rotation + rot

    #         end = tuple(np.array(end) + [perp_shift * math.sin(perp), perp_shift * math.cos(perp)])
    #         pygame.draw.polygon(screen, tricolor, ((end[0] + trirad * math.sin(rotation),
    #                                                 end[1] + trirad * math.cos(rotation)),
    #                                                (end[0] + trirad * math.sin(rotation - 120 * rad),
    #                                                 end[1] + trirad * math.cos(rotation - 120 * rad)),
    #                                                (end[0] + trirad * math.sin(rotation + 120 * rad),
    #                                                 end[1] + trirad * math.cos(rotation + 120 * rad))))

    #     # Areas
    #     if self.lanelet_plot_options['areas']:

    #         for area in self.lanelet_plot_data['areas'].values():
    #             nodes = area['nodes']
    #             colour = area['colour']
    #             pygame.draw.polygon(self.screen, colour, nodes)

    #     # Lanelets
    #     if self.lanelet_plot_options['lanelets']:
    #         for lanelet in self.lanelet_plot_data['lanelets'].values():
    #             nodes = lanelet['nodes']
    #             colour = lanelet['colour']
    #             pygame.draw.polygon(self.screen, colour, nodes)

    #     # Edges
    #     if self.lanelet_plot_options['edges']:
    #         for edge in self.lanelet_plot_data['edges'].values():
    #             nodes = edge['nodes']
    #             colour = edge['colour']
    #             pygame.draw.lines(self.screen, colour, False, nodes, 2)

    #     # Centreline 'parents'
    #     if self.lanelet_plot_options['centreline_parent_edges']:

    #         colours = {'left': (255, 0, 0),
    #                    'right': (0, 255, 0)}

    #         line_widths = {'left': 3,
    #                        'right': 2}

    #         for side in ['left', 'right']:
    #             nodes = self.graph_data[side]['nodes']
    #             edges = self.graph_data[side]['edges']

    #             colour = colours[side]
    #             width = line_widths[side]
    #             # pygame.draw.lines(self.screen, colour, False, nodes, 2)

    #             for edge in edges:
    #                 start_pos = self.scale_points_to_screen(nodes[edge[0]])
    #                 end_pos = self.scale_points_to_screen(nodes[edge[1]])

    #                 pygame.draw.line(self.screen, colour, start_pos, end_pos, width)

    #                 # arrow(self.screen, colour, colour, start_pos, end_pos, 8, 1, side)

    #     # Centrelines
    #     if self.lanelet_plot_options['centrelines']:

    #         nodes = self.graph_data['centre']['nodes']
    #         edges = self.graph_data['centre']['edges']

    #         colours = {'right': (0, 255, 0),
    #                    'left': (255, 0, 0),
    #                    'middle': (0, 0, 255),
    #                    'single': (255, 255, 255)}

    #         for edge in edges:
    #             lane = self.centreline_data['centrelines']['nodes'][edge[1]]['lane']
    #             colour = colours[lane]
    #             start_pos = self.scale_points_to_screen(nodes[edge[0]])
    #             end_pos = self.scale_points_to_screen(nodes[edge[1]])
    #             arrow(self.screen, colour, colour, start_pos, end_pos, 8, 1, 'centre')

    # def draw(self, t, all_objects, grid_per_draw, grid_col_draw, agent_pos_xy, collision, control_info):
    #     """Draw all objects on screen at each time step"""
    #     vehicles_scaled, vehicles_orig, agent, agent_scaled, cars, cars_scaled = all_objects
    #     # Draw blank screen
    #     self.screen.fill((0, 0, 0))

    #     # Add background
    #     # if self.params['scene']['data_type'] == 'InD':
    #     if self.params['simulator']['ego_perspective']:
    #         self.bg_rot, self.bg_shift = self.transform_background_ego(self.bg,
    #                                                                    cars[self.params['simulator']['ego_vehicle_id']])
    #         self.image_offset = self.bg_shift
    #     self.screen.blit(self.bg_rot, self.bg_shift)
    #     # self.screen.blit(self.bg_key, bg_shift) # to filter using road_key

    #     # Draw lanelets
    #     if hasattr(self, "lanelet_plot_data"):
    #         self.lanelet_plot_options = {'areas': False,
    #                                      'lanelets': False,
    #                                      'edges': False,
    #                                      'centrelines': False,
    #                                      'centreline_parent_edges': True}
    #         self.draw_lanelets()

    #     # Draw text box with dynamics info
    #     self.draw_text_box(t, agent, vehicles_orig, collision, control_info)

    #     # Draw collision grid
    #     self.draw_collision_grid(grid_col_draw)

    #     # Draw perception grid
    #     if self.params['simulator']['ego_perception']:
    #         self.draw_perception_grid(grid_per_draw)

    #     # Draw rotated (and resized) cars in their position
    #     self.draw_cars(t, vehicles_scaled, cars_scaled, agent_scaled)

    #     # Draw agent trajectory if controlled
    #     if self.params['simulator']['controllable_agent']:
    #         self.draw_agent_trajectory(agent, agent_pos_xy)

    #     # Pygame update
    #     pygame.display.flip()

    #     if self.params['simulator']['data_type'] == 'OSC':
    #         self.ticks = 2

    #     self.clock.tick(self.ticks)

    # def update_scale(self):

    #     min_x, max_x, min_y, max_y = self.ranges

    #     self.scale = min(self.screen_width / (max_x - min_x),
    #                      self.screen_height / (max_y - min_y))


    # def draw_quick(self, vehicles, traces, goal_path, controller):

    #     """Draw a list of objects with minimal other information"""

    #     # Draw blank screen
    #     self.screen.fill((0, 0, 0))

    #     # self.bg_rot, self.bg_shift, self.bg_rect = self.load_background(
    #     #     self.background_im_name)  # TODO: Scale should be based on background size

    #     # Add background
    #     self.screen.blit(self.bg_rot, self.bg_shift)
    #     # self.screen.blit(self.bg_key, bg_shift) # to filter using road_key

    #     # Draw rotated (and resized) cars in their position

    #     for v in vehicles:

    #         v_image = self.vehicles_images[v].copy()

    #         # v_image = pygame.transform.scale(v_image, (int(0.3 * self.scale), int(0.2 * self.scale)))

    #         rotated = pygame.transform.rotate(v_image, np.rad2deg(vehicles[v].angle))
    #         rect = rotated.get_rect()

    #         vehicle_path = goal_path[v]

    #         # TODO: fix the scaling in a proper manner instead of putting * 100 everywhere
    #         # screen should be scaled to plot meters fine
    #         for point in vehicle_path:
    #             pygame.draw.circle(self.screen, (255,0,0),
    #                     self.scale_points_to_screen([point_val*100 for point_val in point]), 1)

    #         route = np.array(traces[v])[:, 0:2]
    #         for point in route[-50:]:
    #             pygame.draw.circle(self.screen, (0,0,255),
    #                                 self.scale_points_to_screen([point_val*100 for point_val in point]), 1)
            
    #         #print(f"curr_pos = {[point_val*100 for point_val in vehicles[v].position]}")
    #         self.screen.blit(rotated,
    #                         self.scale_points_to_screen([point_val*100 for point_val in vehicles[v].position]
    #                         ) - np.array([rect.width / 2, rect.height / 2]))

    #         #print(f"target_idx={target_idx}, value={vehicle_path[target_idx]}")
    #         #print(f"target_pos = {[point_val*100 for point_val in controller.vehicles_control_info[v]['target_pos']]}")
    #         pygame.draw.circle(self.screen, (0,255,0), self.scale_points_to_screen(
    #             [point_val*100 for point_val in controller.vehicles_control_info[v]['target_pos']]), 3)

    #         #route_scaled = [self.scale_points_to_screen(loc) for loc in route]

    #         # if len(route) >= 2:
    #         #     pygame.draw.lines(self.screen, (255, 0, 0), False, route_scaled, 2)

    #         # self.screen.blit(rotated, vehicles[v].position - np.array([rect.width / 2, rect.height / 2]))

    #         # if True:  # Draw vehicle ID
    #         #     text_v = self.myfont.render(str(v), False, (255, 255, 255))
    #         #     self.screen.blit(text_v, cars_scaled[v].position - np.array([rect.width / 2, rect.height / 2]))

        # Pygame update
        # pygame.display.flip()

        # if self.params['simulator']['data_type'] == 'OSC':
        #     self.ticks = 2  # TODO: Change when this is adjusted

        # self.clock.tick(self.ticks)