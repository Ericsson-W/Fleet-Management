from robots.optitrack import OptiTrack
import numpy as np  

class Point():

    def __init__(self, coords, point_type: str,corner_radius: float, turn_direction: int):
        
        # Coors are (x,y) relative to origin
        self.coords = coords
        self.turn_direction = turn_direction # 0=left, 1=straight, 2=right
        
        # Possible types: "curve_s", "curve_e", "inter_s", "inter_e"
        self.point_type = point_type

        # Outer-corner = 0.41; Inner-corner = 0.18; Non-corner = None
        self.corner_radius = corner_radius
        


class PathGenerator():

    def __init__(self, robot_parameters:dict):

        self.points_per_m: int     = robot_parameters['path_points_per_m']
        self.path_loop = robot_parameters['path_loop']

        self.inner_radius: float    = robot_parameters['inner_radius']
        self.outer_radius: float    = robot_parameters['outer_radius']


        ##
        # Hardcoded Paths for IITS Control Room
        ## 

        # Waypoints for each pre-determined path.
        # (x,y) in meters, waypoint type, curve radius, segment type
        self.pre_determined_paths_waypoints = { 
        "loop_SN": [
                    Point([1.2137, -0.2090], "curve_e", self.outer_radius, 0),
                    Point([0.6306, -0.2173], "curve_s", self.outer_radius, 0),
                    Point([0.2126, -0.6289], "curve_e", self.outer_radius, 0),
                    Point([0.2109, -1.2116], "curve_s", self.outer_radius, 0),
                    Point([0.6272, -1.6210], "curve_e", self.outer_radius, 0),
                    Point([1.1642, -1.6249], "inter_s", None, 1),
                    Point([1.8367, -1.6225], "inter_e", None, 1),
                    Point([2.3921, -1.6289], "curve_s", self.inner_radius, 2),
                    Point([2.5840, -1.8036], "curve_e", self.inner_radius, 2),
                    Point([2.5866, -2.3881], "curve_s", self.inner_radius, 2),
                    Point([2.3926, -2.5803], "curve_e", self.inner_radius, 2),
                    Point([1.8196, -2.5782], "curve_s", self.inner_radius, 2),
                    Point([1.6235, -2.3838], "curve_e", self.inner_radius, 2),
                    Point([1.6288, -1.8582], "inter_s", None, 1),
                    Point([1.6288, -1.1936], "inter_e", None, 1),
                    Point([1.6224, -0.6348], "curve_s", self.outer_radius, 0)
                    ],
        "loop_NS": [
                    Point([1.1952, -0.4223], "curve_s", self.inner_radius, 2),
                    Point([1.3922, -0.6253], "curve_e", self.inner_radius, 2),
                    Point([1.3888, -1.1345], "inter_s", None, 1),
                    Point([1.3920, -1.8382], "inter_e", None, 1),
                    Point([1.3983, -2.3716], "curve_s", self.outer_radius, 0),
                    Point([1.8183, -2.8016], "curve_e", self.outer_radius, 0),
                    Point([2.3949, -2.8095], "curve_s", self.outer_radius, 0),
                    Point([2.8102, -2.3867], "curve_e", self.outer_radius, 0),
                    Point([2.8121, -1.8177], "curve_s", self.outer_radius, 0),
                    Point([2.3985, -1.3966], "curve_e", self.outer_radius, 0),
                    Point([1.8797, -1.3855], "inter_s", None, 1),
                    Point([1.1935, -1.3888], "inter_e", None, 1),
                    Point([0.6587, -1.3898], "curve_s", self.inner_radius, 2),
                    Point([0.4399, -1.1799], "curve_e", self.inner_radius, 2),
                    Point([0.4438, -0.6381], "curve_s", self.inner_radius, 2),
                    Point([0.6152, -0.4486], "curve_e", self.inner_radius, 2)
                    ]
        }

        # To
        self.intersection_waypoints = []
        
        for path in self.pre_determined_paths_waypoints.keys():
            self.intersection_waypoints += [point.coords for point in self.pre_determined_paths_waypoints[path] if point.point_type == "inter_s"]
        
        #print(f"Intersection waypoints: {self.intersection_waypoints}")

        # TODO: delete any reference to intersection waypoints in generatePath()

        # TODO: not sure why we need np.asarray (return of function should already be np.ndarray)
        # Generate pre-determined paths using waypoints and self.generatePath(). Extract intersection waypoints.
        
        self.pre_determined_paths = {
                                        path_name: self.generatePath(self.pre_determined_paths_waypoints[path_name])
                                        for path_name in self.pre_determined_paths_waypoints.keys()
                                    }

    ##
    # Helper Functions
    ##

    def get_circleCenter(self, start_point, end_point, radius):

        # TODO: look into this
        if (start_point.turn_direction == 0):
            start_p = start_point
            end_p = end_point
        elif (start_point.turn_direction == 2):
            start_p = end_point
            end_p = start_point


        q = np.sqrt((end_p.coords[0]-start_p.coords[0])**2.0 + (end_p.coords[1]-start_p.coords[1])**2.0)
        x_mean = (start_p.coords[0] + end_p.coords[0]) / 2.0
        y_mean = (start_p.coords[1] + end_p.coords[1]) / 2.0
        
        
        xc = x_mean + np.sqrt(radius**2.0-(q/2.0)**2.0)*(start_p.coords[1]-end_p.coords[1])/q
        yc = y_mean + np.sqrt(radius**2.0-(q/2.0)**2.0)*(end_p.coords[0]-start_p.coords[0])/q  
        
        return [xc, yc]


    def parametric_circle(self, t, circle_center, R):
        x = circle_center[0] + R*np.cos(t)
        y = circle_center[1] + R*np.sin(t)
        return np.asarray([x,y]).T


    def inv_parametric_circle(self, x, xc, R, y_sign_correction: bool):
        
        inside_term = (x-xc)/R
        #print(f"(x-xc)/R={inside_term}")

        t = np.arccos(inside_term)
        
        if y_sign_correction:
            return -t
        
        return t

    def generatePath(self, waypoints):

        # Initiating 2D path -> [Nan, Nan]
        self.path = np.empty([1,2])
        self.path.fill(np.nan)

        nr_waypoints = len(waypoints)
        
        # Keeping coords of waypoints of type "inter_s"
        # self.intersection_waypoints = []

        # For each point
        for index, point in enumerate(waypoints):

            # TODO: this has been moved to outside?
            # if point.point_type == "inter_s":
            #     point_coords_str = [ '%.2f' % elem for elem in point.coords ]
            #     self.intersection_waypoints.append(point_coords_str)

            # If the path loops, connect last point to first, otherwise end at last point
            if (not self.path_loop) and (index == nr_waypoints - 1):
                #print("Finishing path without loop.")    
                continue
            
            start_idx   = index
            end_idx     = (index+1) % (nr_waypoints)

            #print(f"start_idx={start_idx}, end_idx={end_idx}")

            start_point = waypoints[start_idx]
            end_point   = waypoints[end_idx]

            #print(f"start_pt = {start_point}, end_pt = {end_point}")

            # Curved Segment #####################
            if (start_point.point_type == "curve_s") and (end_point.point_type == "curve_e"):
                #print(f"Doing curved segment bewteen point {start_idx} and {end_idx}.")

                # Getting Circle Center
                circle_center = self.get_circleCenter(start_point, end_point, start_point.corner_radius)                    

                # Y-correction TODO: look into this
                if circle_center[1] < ((start_point.coords[1] + end_point.coords[1])/2.0):
                    y_correction = False
                else:
                    y_correction = True

                # 
                start_theta = self.inv_parametric_circle(start_point.coords[0], circle_center[0], start_point.corner_radius, y_correction)
                end_theta   = self.inv_parametric_circle(end_point.coords[0], circle_center[0], start_point.corner_radius, y_correction)

                arc_length = np.abs( (end_theta - start_theta) * start_point.corner_radius )
                #print(arc_length, self.points_per_m)
                segment_points = int(arc_length * self.points_per_m)

                #print(f"Arc length = {arc_length}, nr_segment_points={segment_points}")

                arc_Theta = np.linspace(start_theta, end_theta, segment_points)

                segment = self.parametric_circle(arc_Theta, circle_center, start_point.corner_radius)
            #####################################

            # Straight Segment ##################
            elif (
                ((start_point.point_type == "curve_e") and (end_point.point_type == "curve_s"))
                or ((start_point.point_type == "curve_e") and (end_point.point_type == "inter_s"))
                or ((start_point.point_type == "inter_e") and (end_point.point_type == "curve_s"))
                or ((start_point.point_type == "inter_s") and (end_point.point_type == "inter_e"))
                ):
                #print(f"Doing straight segment bewteen point {start_idx} and {end_idx}.")

                straight_line_length = np.linalg.norm(
                        np.subtract(end_point.coords, start_point.coords))
                segment_points = int(straight_line_length * self.points_per_m)

                #print(f"Straight line length = {straight_line_length}, nr_segment_points={segment_points}")

                segment = np.linspace(start_point.coords, end_point.coords, segment_points)
            #####################################


            self.path = np.concatenate((self.path, segment))
                
       
        self.path = np.delete(self.path, (0), axis=0) # Remove empty-point from initialization

        return self.path#, self.intersection_waypoints
    

