import omni
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from pxr import Usd
import numpy as np
from typing import Tuple
from pxr_utils import createFixedJoint

class Robot(Articulation):
    def __init__(self, 
                 usd_path:str, 
                 prim_path:str, 
                 name:str, 
                 position:np.ndarray):
        create_prim(
            prim_path=prim_path, 
            prim_type="Xform", 
            position=position,)
        add_reference_to_stage(usd_path, prim_path)
        super().__init__(prim_path=prim_path, name=name)
        
        # hung robot to world 
        self.stage = omni.usd.get_context().get_stage()
        createFixedJoint(self.stage, prim_path+"/trunk/fixed_joint", body_path1="/World", body_path2=prim_path+"/trunk")
    
    def init(self, world:Usd.Stage):
        world.reset()
        world.scene.add(self)
        self.initialize()
    
    @property
    def joint_names(self):
        """
        isaac joint order
        ['L_hip_joint', 'R_hip_joint', 
        'L_hip2_joint', 'R_hip2_joint', 
        'L_thigh_joint', 'R_thigh_joint', 
        'L_calf_joint', 'R_calf_joint', 
        'L_toe_joint', 'R_toe_joint']
        """
        return self.dof_names
    
    def apply_effort(self, effort:np.ndarray):
        self.set_joint_efforts(effort)
    
    def get_joint_state(self)->Tuple[np.ndarray, np.ndarray, np.ndarray]:
        joint_position = self.get_joint_positions()
        joint_velocity = self.get_joint_velocities()
        joint_effort_est = self.get_measured_joint_efforts()
        return joint_position, joint_velocity, joint_effort_est
    
    def get_body_state(self)->Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        position, orientation = self.get_world_pose()
        velocity = self.get_linear_velocity()
        angular_velocity = self.get_angular_velocity()
        return position, orientation, velocity, angular_velocity