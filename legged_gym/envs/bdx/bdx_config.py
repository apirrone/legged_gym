from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO


class BdxRoughCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 4096
        num_observations = 57
        num_actions = 15

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = "plane"
        measure_heights = False

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.21]  # x,y,z [m]
        default_joint_angles = {  # = target angles [rad] when action = 0.0
            "right_hip_yaw": -0.014,
            "right_hip_roll": 0.08,
            "right_hip_pitch": 0.53,
            "right_knee": -1.62,
            "right_ankle": 0.91,
            "left_hip_yaw": 0.013,
            "left_hip_roll": 0.077,
            "left_hip_pitch": 0.59,
            "left_knee": -1.63,
            "left_ankle": 0.86,
            "neck_pitch": -0.17,
            "head_pitch": -0.17,
            "head_yaw": 0.0,
            "left_antenna": 0.0,
            "right_antenna": 0.0,
        }

    class control(LeggedRobotCfg.control):
        # TODO Eyeballed values

        # PD Drive parameters:
        stiffness = {
            "hip_yaw": 50.0,
            "hip_roll": 50.0,
            "hip_pitch": 50.0,
            "knee": 50.0,
            "ankle": 50.0,
        }  # [N*m/rad]

        damping = {
            "hip_yaw": 1.0,
            "hip_roll": 1.0,
            "hip_pitch": 1.0,
            "knee": 1.0,
            "ankle": 1.0,
        }  # [N*m*s/rad]

        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/bdx/urdf/bdx.urdf"
        name = "bdx"
        foot_name = "foot"
        terminate_after_contacts_on = ["body_module"]
        flip_visual_attachments = False
        self_collisions = 1  # 1 to disable, 0 to enable...bitwise filter

    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 0.95
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.9
        max_contact_force = 300.0
        only_positive_rewards = False

        class scales(LeggedRobotCfg.rewards.scales):
            termination = -200.0
            tracking_ang_vel = 1.0
            torques = -5.0e-6
            dof_acc = -2.0e-7
            lin_vel_z = -0.5
            feet_air_time = 5.0
            dof_pos_limits = -1.0
            no_fly = 0.25
            dof_vel = -0.0
            ang_vel_xy = -0.0
            feet_contact_forces = -0.0


class BdxRoughCfgPPO(LeggedRobotCfgPPO):
    class runner(LeggedRobotCfgPPO.runner):
        run_name = ""
        experiment_name = "rough_bdx"

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01
