from upkie.envs.upkie_servos import UpkieServos
from gymnasium import spaces

class UpkieServoPositions(UpkieServos):
    """!
    Child class of UpkieServos that defines the action space as a dictionary of
    joint names with the following keys:    
    - "position": the desired position of the joint
    - "kp_scale": the proportional gain of the joint
    - "kd_scale": the derivative gain of the joint
    Which simplifies the control of the robot by allowing to control the position
    of the joints and the gains.
    """
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        action_space = {joint.name: spaces.Dict(
                {
                    "position": spaces.Box(
                        low=joint.limit.lower,
                        high=joint.limit.upper,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kp_scale": spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=float,
                    ),
                    "kd_scale": spaces.Box(
                        low=0.0,
                        high=1.0,
                        shape=(1,),
                        dtype=float,
                    ),
                }
            ) for joint in self.model.joints}
        self.action_space = spaces.Dict(action_space)