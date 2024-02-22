import os
import numpy as np
import pybullet as p
from manipulation.gym_info import *
from .robot import Robot, RobotIsaac

class Panda(Robot):
    def __init__(self, controllable_joints='right', slider=True, floating=False):
        self.slider = slider
        self.floating = floating
        if not floating:
            if not slider:
                right_arm_joint_indices = [0, 1, 2, 3, 4, 5, 6] # Controllable arm joints
                right_end_effector = 11 # Used to get the pose of the end effector
                right_gripper_indices = [9, 10] # Gripper actuated joints
            else:
                right_arm_joint_indices = [0, 1, 2, 4, 5, 6, 7, 8, 9, 10]
                right_end_effector = 15 # Used to get the pose of the end effector
                right_gripper_indices = [13, 14] # Gripper actuated joints
                
        else:
            right_arm_joint_indices = []
            right_end_effector = -1
            right_gripper_indices = [0, 1]

        super(Panda, self).__init__(controllable_joints, right_arm_joint_indices, right_end_effector, right_gripper_indices)

    def init(self, directory, id, np_random, fixed_base=False, use_suction=True):
        self.body = p.loadURDF(os.path.join(directory, 'franka_mobile', 'panda_suction_slider_mobile.urdf'), useFixedBase=fixed_base, basePosition=[-1, -1, 0.5], flags=p.URDF_USE_SELF_COLLISION, physicsClientId=id)

        for i in range(p.getNumJoints(self.body, physicsClientId=id)):
            print(p.getJointInfo(self.body, i, physicsClientId=id))
            link_name = p.getJointInfo(self.body, i, physicsClientId=id)[12].decode('utf-8')
            print("link_name: ", link_name)

        super(Panda, self).init(self.body, id, np_random)


class PandaIsaac(RobotIsaac):
    def __init__(self, sim, gym, controllable_joints='right', slider=True, floating=False):
        self.sim = sim
        self.gym = gym
        self.slider = slider
        self.floating = floating
        if not floating:
            if not slider:
                right_arm_joint_indices = [0, 1, 2, 3, 4, 5, 6]  # Controllable arm joints
                right_end_effector = 11  # Used to get the pose of the end effector
                right_gripper_indices = [9, 10]  # Gripper actuated joints
            else:
                right_arm_joint_indices = [0, 1, 2, 4, 5, 6, 7, 8, 9, 10]
                right_end_effector = 15  # Used to get the pose of the end effector
                right_gripper_indices = [13, 14]  # Gripper actuated joints

        else:
            right_arm_joint_indices = []
            right_end_effector = -1
            right_gripper_indices = [0, 1]

        super(Panda, self).__init__(controllable_joints, right_arm_joint_indices, right_end_effector,
                                    right_gripper_indices)

    def init(self, directory, np_random, fixed_base=False, use_suction=True):
        self.body = p.loadURDF(os.path.join(directory, 'franka_mobile', 'panda_suction_slider_mobile.urdf'),
                               useFixedBase=fixed_base, basePosition=[-1, -1, 0.5], flags=p.URDF_USE_SELF_COLLISION,
                               physicsClientId=id)

        asset_options = gymapi.AssetOptions()
        asset_options.flip_visual_attachments = True
        asset_options.fix_base_link = True
        asset_options.collapse_fixed_joints = True
        asset_options.disable_gravity = True
        asset_options.thickness = 0.001
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS
        asset_options.use_mesh_materials = True

        self.body = self.gym.load_asset(self.sim,
                                        os.path.join(directory, 'franka_mobile'),
                                        'panda_suction_slider_mobile.urdf',
                                        asset_options)

        for i in range(p.getNumJoints(self.body, physicsClientId=id)):
            print(p.getJointInfo(self.body, i, physicsClientId=id))
            link_name = p.getJointInfo(self.body, i, physicsClientId=id)[12].decode('utf-8')
            print("link_name: ", link_name)

        super(Panda, self).init(self.body, id, np_random)
