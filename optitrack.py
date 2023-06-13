# Imports
import numpy as np

from robots.natnet_client import NatNetClient




########################################################################################################################
###############################################  Quaternion Conversion #################################################
########################################################################################################################
"""
References:
[1] - "A Tutorial on Euler Angles and Quaternions" - Moti Ben-Ari
[2] - "Quaternion kinematics for the error-state KF" - Joan Solà 
"""

def Quat2RMatrix(quaternion):
    """Quat2RMatrix gets the rotation matrix equivalent of a quaternion

    Args:
        quaternion ([array_like]): Unit right-handed quaternion in [qx, qy, qz, qw] form

    Returns:
        [np_array]: 3x3 Rotation matrix
    """

    # assert(np.linalg.norm(quaternion) == 1) # ensures unit quaternion
    # TODO: Check that quaternion is right-handed

    qx, qy, qz, qw = quaternion

    RMatrix = np.array(
        [
            [
                (qw * qw) + (qx * qx) - (qy * qy) - (qz * qz),
                2 * (qx * qy - qw * qz),
                2 * (qx * qz + qw * qy),
            ],
            [
                2 * (qx * qy + qw * qz),
                (qw * qw) - (qx * qx) + (qy * qy) - (qz * qz),
                2 * (qy * qz - qw * qx),
            ],
            [
                2 * (qx * qz - qw * qy),
                2 * (qy * qz + qw * qx),
                (qw * qw) - (qx * qx) - (qy * qy) + (qz * qz),
            ],
        ]
    )

    return RMatrix


def RMatrix2Euler(RMatrix, units="rad"):
    """Matrix2Euler converts the Rotation Matrix into Euler angles that follow the roll-pitch-yaw convention.
    Pitch is limited to [-90, 90], due to arcsin use. Other angles span [-180, 180].

    Args:
        matrix ([array_like]): Rotation matrix to be converted
        units (str, optional): Euler angles can either be returned in "deg" or "rad".
        Defaults to "rad".

    Returns:
        [np_array]: Euler Angles in roll-pitch-yaw convention
    """

    EulerAngles = np.array(
        [
            [
                np.arctan2(RMatrix[2, 1], RMatrix[2, 2]),
                np.arcsin(-RMatrix[2, 0]),
                np.arctan2(RMatrix[1, 0], RMatrix[0, 0]),
            ]
        ]
    )

    if units == "deg":
        EulerAngles = np.rad2deg(EulerAngles)

    return EulerAngles

def Quat2Yaw(quaternion, units="rad"):
    """ This simple function combines the two above to return yaw
    """
    RMatrix = Quat2RMatrix(quaternion)
    euler = RMatrix2Euler(RMatrix, units)

    yaw = euler[0][2]  # OptiTrack data is taken to be streamed "Z-up"

    return yaw


########################################################################################################################
###############################################  OptiTrack Class #######################################################
########################################################################################################################

class OptiTrack:

    def __init__(self, optitrack_params): 
        
        # General
        self.number_controlled_vehicles = None
        self.frame_number            = None

        self.natnet_client = NatNetClient(
            server              = optitrack_params['server_ip'],
            multicast           = optitrack_params['multicast_ip'],
            commandPort         = optitrack_params['command_port'],
            dataPort            = optitrack_params['data_port'],
            rigidBodyListener   = self.cbReceivedRBFrame,
            newFrameListener    = self.cbReceivedNewFrame
        )

        # TODO: understand why we have to use old and "new" is always empty
        # Rigid Body Data: ID is int, Pos in meters and Yaw in rad
        self.rb_IDs                 = [] 
        self.rb_old_Ids             = []
        self.rb_Pos                 = []
        self.rb_old_Pos             = [] # need to set position of 5 veh
        self.rb_Yaw                 = []
        self.rb_old_Yaw             = []


        # Start NatNet Client
        self.natnet_client.run()


    def cbReceivedNewFrame(self, frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount,
                    skeletonCount, labeledMarkerCount, timecode, timecodeSub, timestamp,
                    isRecording, trackedModelsChanged,):
        """receiveNewFrame is a callback function that gets connected to the
        NatNet client and is called once per Mocap frame.

        Args:
            frameNumber ([type]): [description]
            markerSetCount ([type]): [description]
            unlabeledMarkersCount ([type]): [description]
            rigidBodyCount ([type]): [description]
            skeletonCount ([type]): [description]
            labeledMarkerCount ([type]): [description]
            timecode ([type]): [description]
            timecodeSub ([type]): [description]
            timestamp ([type]): [description]
            isRecording (bool): [description]
            trackedModelsChanged ([type]): [description]
        """

        self.rb_old_Pos = np.array(self.rb_Pos)
        self.rb_old_Yaw = np.array(self.rb_Yaw)
        self.rb_old_Ids = np.array(self.rb_IDs)

        # TODO: do some check if bot enters or leaves capture volume?

        # Clear arrays for new Mocap Frame
        self.rb_IDs = []
        self.rb_Pos = []
        self.rb_Yaw = []

        self.number_controlled_vehicles    = rigidBodyCount
        self.frame_number               = frameNumber


    def cbReceivedRBFrame(self, id, position, rotation):
        """receiveRigidBodyFrame is a callback function that gets connected
        to the NatNet client. Called once per rigid body per frame. There are 
        n rigid body frames per camera frame, where n is the number of rigid bodies.

        Args:
            id ([int]): rigid body ID as defined in OptiTrack
            position ([array_like]): (x,y,z) of rigid body, following convention of OptiTrack
            rotation ([array_like]): orientation unit quaternion
        """
        
        self.rb_IDs.append(id)        
        self.rb_Pos.append(position[0:2])
        self.rb_Yaw.append(Quat2Yaw(rotation, units="rad"))


    def update_poses(self, dict_vehicles):

        # TODO: find why have to use _old_ variables
        # TODO: Add exception handling for when ID is not in tracked vehicles or vice versa!!  

        for index, vehicle in enumerate(self.rb_old_Ids):
            
            dict_vehicles[vehicle].position = self.rb_old_Pos[index]
            dict_vehicles[vehicle].angle      = self.rb_old_Yaw[index]

        return dict_vehicles