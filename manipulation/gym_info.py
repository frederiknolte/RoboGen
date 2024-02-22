from isaacgym import gymapi
import numpy as np
from pathlib import Path


# global
task_names = ["test", "FrankaPoseCabinetBase", "FrankaPoseCabinetPC"]
algo_names = ["rdm", "ppo_pn" , "ppo", "heuristics", "sac", "pregrasp_ppo", \
    "pregrasp_ppo_pn", "imitation_learning", "collect_data", "behavior_cloning", \
        "sac_il", "dagger", "dagger_ppo", "ILAD", 'curl_sac']
no_training_algos = ["rdm", "heuristics"]

# vec_task:
clip_actions = 3.0 
clip_observations = 5.0
clip_error_pos = 0.05
clip_error_rot = 0.2
clip_delta = 0.1


# base env
plane_params_static_friction = 0.1
plane_params_dynamic_friction = 0.1
control_ik_damping = 0.05

# camera
cam1_pos = gymapi.Vec3(0.5,0,2.0)
cam1_rot = gymapi.Vec3(-2.0,0., -0.8)
cam2_pos = gymapi.Vec3(0.0,0.8,2.3)
cam2_rot = gymapi.Vec3(-3.0,-2.0, -0.2)
cam3_pos = gymapi.Vec3(0.0,-0.8,2.3)
cam3_rot = gymapi.Vec3(-3.0,2.0, -0.2)
video_cam_pos = gymapi.Vec3(0.8,-1.5, 2.2)
video_cam_rot = gymapi.Vec3(-2.8, 2.2, -0.1)
general_cam_pos = gymapi.Vec3(0, 0, 10)
general_cam_rot = gymapi.Vec3(-1, 0, -1)

# simulator
sim_params_dt = 1./60.
sim_params_physx_solver_type = 1
sim_params_physx_num_position_iterations = 8
sim_params_physx_num_velocity_iterations = 8
sim_params_physx_num_threads = 8
sim_params_physx_max_gpu_contact_pairs = 8 * 1024 * 1024
sim_params_physx_rest_offset = 0.0
sim_params_physx_bounce_threshold_velocity = 0.2
sim_params_physx_max_depenetration_velocity = 1000.0
sim_params_physx_default_buffer_size_multiplier = 5.0
sim_params_physx_contact_offset = 1e-3


# robot
asset_options_fix_base_link = True
asset_options_disable_gravity = True
asset_options_flip_visual_attachments = True
asset_options_armature = 0.01
asset_options_thickness = 0.001
# robot initial pose

initial_franka_pose_r = gymapi.Quat(0.0, 0.0, 1.0, 0.0)
initial_franka_pose_p_close_drawer = gymapi.Vec3(1.2, 0.0, 0.7)
initial_franka_pose_p_close_door = gymapi.Vec3(1.2, 0.0, 0.7)
initial_franka_pose_p_open_drawer = gymapi.Vec3(0.8, 0.0, 0.5)
initial_franka_pose_p_open_door = gymapi.Vec3(0.8, 0.0, 0.4)
# initial_franka_pose_p_open_door = gymapi.Vec3(0.4, 0.4, 0.45)
# initial_franka_pose_p_open_door = gymapi.Vec3(0.5, 0.4, 0.45)

# asset

# articulated asset
articulated_asset_options_fix_base_link_object = True
articulated_asset_options_disable_gravity_object = True
articulated_asset_options_collapse_fixed_joints_object = False # Merge links that are connected by fixed joints.
# articulated asset init pose
# object_init_pose_p_np = np.array([-1, 0, 1.4])
# object_init_pose_r_np = np.array([0.,0.,1.,0.])
# object_init_pose_p = gymapi.Vec3(-1, 0, 1.4)
# object_init_pose_r = gymapi.Quat(0.,0.,1.,0.)

object_init_pose_p_np = np.array([-2, 0, 1.2])
object_init_pose_r_np = np.array([0.,0.,1.,0.])
object_init_pose_p = gymapi.Vec3(-2, 0, 1.2)
object_init_pose_r = gymapi.Quat(0.,0.,1.,0.)

# static asset
static_asset_options_fix_base_link_object = True
static_asset_options_disable_gravity_object = True
static_asset_options_collapse_fixed_joints_object = True # Merge links that are connected by fixed joints.
# static asset init pose
static_object_init_pose_p_np = np.array([-30.8, -30.5, -20.4])
static_object_init_pose_r_np = np.array([0.,0.,1.,0.])
static_object_init_pose_p = gymapi.Vec3(-30.8, -30.5, -20.4)
static_object_init_pose_r = gymapi.Quat(0.,0.,1.,0.)


COLOR20 = np.array(
                [[0, 128, 128], [230, 190, 255], [170, 110, 40], [255, 250, 200], [128, 0, 0],
                [170, 255, 195], [128, 128, 0], [255, 215, 180], [0, 0, 128], [128, 128, 128],
                [230, 25, 75], [60, 180, 75], [255, 225, 25], [0, 130, 200], [245, 130, 48],
                [145, 30, 180], [70, 240, 240], [240, 50, 230], [210, 245, 60], [250, 190, 190]])
