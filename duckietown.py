# Imports
import rospy
import numpy as np
from  typing import List

# from simulator.car import Car
# import utils.pre_processing.dynamics as dyn

from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header



############################
###  Inverse Kinematics  ###
############################


def duckiebot_inverse_kinematics(vehicle_kinematic_parameters, v, omega):
    """ Given the kinematic parameters of the vehicle, as well as the desired linear and angular velocities ("v", "omega"),
    this function calculates the required wheel velocities.

    "v" and "omega" are taken to already be trimmed. A +ve omega leads to a left turn and a -ve omega to a right turn.

    Reference: https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/kinematics_node.py
    """

    # Motor constant is taken to be the same for both motors
    k = vehicle_kinematic_parameters['motor_constant']

    # 
    k_r_inv = (vehicle_kinematic_parameters['motor_gain'] + vehicle_kinematic_parameters['motor_trim']) / k
    k_l_inv = (vehicle_kinematic_parameters['motor_gain'] - vehicle_kinematic_parameters['motor_trim']) / k

    wheel_omega_r = (v + 0.5 * omega * vehicle_kinematic_parameters['wheel_distance']) / vehicle_kinematic_parameters['wheel_radius']
    wheel_omega_l = (v - 0.5 * omega * vehicle_kinematic_parameters['wheel_distance']) / vehicle_kinematic_parameters['wheel_radius']

    wheel_vel_right = wheel_omega_r * k_r_inv
    wheel_vel_left = wheel_omega_l * k_l_inv

    # TODO: do SMTH to trim the velocities? or should be elsewhere

    return wheel_vel_left, wheel_vel_right


############################
###    Duckiebot Class   ###
############################

class Duckiebot():
    # TODO: make duckie inherit from Car class; prob gonna involve changing Car()
    def __init__(self, hostname):
        
        self.vehicle_hostname: str          = hostname
        self.vehicle_type: str              = "duckiebot"
        self.dimensions: List[float, float] = [0.211, 0.131] # [length, width] in meters
        # self.vehicle_name = ['td09']

        # Wrt to global coordinate frame
        self.position       = np.array([0.0, 0.0])
        self.velocity       = np.array([0.0, 0.0])
        self.accel          = np.array([0.0, 0.0])
        self.angle          = 0.0
        self.disp_vector    = np.array([0.0, 0.0])

        self.kinematic_parameters: dict = self.get_kinematics_parameters()

        self.rospub_wheel_vel: rospy.Publisher = rospy.Publisher(
                f'/{self.vehicle_hostname}/wheels_driver_node/wheels_cmd',
                WheelsCmdStamped,
                queue_size=1)

        # self.rospub_2dPose: rospy.Publisher = rospy.Publisher(
        #     f'/{self.hostname}/debug_msgs/2dpose', TwoDimensionalPlotDatapoint, queue_size=1)



    def get_kinematics_parameters(self):
        """ Fetches kinematic parameters of Duckiebot from the ROS parameter server.
        """

        kinematics_parameters: dict = dict()

        kinematics_parameters['motor_trim']        = rospy.get_param(f"/{self.vehicle_hostname}/kinematics_node/trim")
        kinematics_parameters['motor_constant']    = rospy.get_param(f"/{self.vehicle_hostname}/kinematics_node/k")
        kinematics_parameters['motor_gain']        = rospy.get_param(f"/{self.vehicle_hostname}/kinematics_node/gain")
        kinematics_parameters['wheel_radius']      = rospy.get_param(f"/{self.vehicle_hostname}/kinematics_node/radius")
        kinematics_parameters['wheel_distance']    = rospy.get_param(f"/{self.vehicle_hostname}/kinematics_node/baseline")

        return kinematics_parameters

    # TODO: currently this isnt called anywhere
    def updatePose(self, new_pos, new_orientation, delta_t):

        new_vel = (new_pos - self.position) / delta_t
        self.accel = (new_vel - self.velocity) / delta_t
        
        self.velocity = new_vel

        self.disp_vector = (new_pos - self.position)

        self.position   = new_pos
        self.angle      = new_orientation


    def publish_cmd(self, v, omega):
        """ TODO: change command to v, steering angle?
        """
        # print(f"Publishing cmd for {self.vehicle_name}: v: {v}, omega: {omega}")
        wheel_vel_left, wheel_vel_right = duckiebot_inverse_kinematics(self.kinematic_parameters, v, omega)

        wheelVel_msg = WheelsCmdStamped()
        # wheelVel_msg.header = Header()
        # wheelVel_msg.header.stamp = rospy.Time.now()
        wheelVel_msg.vel_left, wheelVel_msg.vel_right = [wheel_vel_left, wheel_vel_right]

        self.rospub_wheel_vel.publish(wheelVel_msg)