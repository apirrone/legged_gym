from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO


class BdxRoughCfg(LeggedRobotCfg):
    class env(LeggedRobotCfg.env):
        num_envs = 10
        num_observations = 57
        num_actions = 15

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = "plane"
        measure_heights = False

    class commands(LeggedRobotCfg.commands):
        num_commands = 4  # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        heading_command = True  # if true: compute ang vel command from heading error

        class ranges(LeggedRobotCfg.commands.ranges):
            lin_vel_x = [0, 0.3]  # min max [m/s]
            lin_vel_y = [-0.15, 0.15]  # min max [m/s]
            ang_vel_yaw = [-0.1, 0.1]  # min max [rad/s]s
            heading = [-3.14, 3.14]

    class init_state(LeggedRobotCfg.init_state):
        pos = [0.0, 0.0, 0.19]  # x,y,z [m]
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
        # TODO These are probably critical.
        # Too low now, trouble lifting legs
        # Before, with 10 stiffness and 1 damping, the robot could walk

        # PD Drive parameters:
        stiffness = {
            "hip_yaw": 5.0,
            "hip_roll": 5.0,
            "hip_pitch": 5.0,
            "knee": 5.0,
            "ankle": 5.0,
            "neck_pitch": 5.0,
            "head_pitch": 5.0,
            "head_yaw": 5.0,
            "left_antenna": 5.0,
            "right_antenna": 5.0,
        }  # [N*m/rad]

        damping = {
            "hip_yaw": 0.5,
            "hip_roll": 0.5,
            "hip_pitch": 0.5,
            "knee": 0.5,
            "ankle": 0.5,
            "neck_pitch": 0.5,
            "head_pitch": 0.5,
            "head_yaw": 0.5,
            "left_antenna": 0.5,
            "right_antenna": 0.5,
        }  # [N*m*s/rad]

        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.5
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset(LeggedRobotCfg.asset):
        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/bdx/urdf/bdx.urdf"
        name = "bdx"
        foot_name = "foot"
        terminate_after_contacts_on = [
            "body_module",
            "head",
            "leg_module_2",
            "leg_module_3",
            "leg_module_4",
            "left_antenna_assembly",
            "right_antenna_assembly",
        ]
        flip_visual_attachments = False
        self_collisions = 1  # 1 to disable, 0 to enable...bitwise filter

    class rewards(LeggedRobotCfg.rewards):
        soft_dof_pos_limit = 0.95
        soft_dof_vel_limit = 0.9
        soft_torque_limit = 0.9
        max_contact_force = 100.0
        only_positive_rewards = False
        base_height_target = 0.15
        tracking_sigma = 0.01  # tracking reward = exp(-error^2/sigma)

        class scales(LeggedRobotCfg.rewards.scales):
            termination = -200.0
            tracking_ang_vel = 1.0
            tracking_lin_vel = 2.0
            torques = -5.0e-6
            dof_acc = -2.0e-7
            lin_vel_z = -0.5
            feet_air_time = 0
            dof_pos_limits = -1.0
            # no_fly = 0.1
            dof_vel = -0.0
            ang_vel_xy = -0.0
            feet_contact_forces = -0.0
            head_behavior = -0.1
            base_height = -0.25
            orientation = -0.1
            close_to_init_pos = -0.15
            # gait = 1.0
            # stand_still = -1.0

    # This was pretty good
    # class rewards(LeggedRobotCfg.rewards):
    #     soft_dof_pos_limit = 0.95
    #     soft_dof_vel_limit = 0.9
    #     soft_torque_limit = 0.9
    #     max_contact_force = 300.0
    #     only_positive_rewards = False
    #     base_height_target = 0.15

    #     class scales(LeggedRobotCfg.rewards.scales):
    #         termination = -200.0
    #         tracking_ang_vel = 1.0
    #         torques = -5.0e-6
    #         dof_acc = -2.0e-7
    #         lin_vel_z = -0.5
    #         feet_air_time = 5.0
    #         dof_pos_limits = -1.0
    #         no_fly = 0.25
    #         dof_vel = -0.0
    #         ang_vel_xy = -0.0
    #         feet_contact_forces = -0.0
    #         head_behavior = -0.1
    #         base_height = -0.25
    #         orientation = -0.1
    #         close_to_init_pos = -0.1


class BdxRoughCfgPPO(LeggedRobotCfgPPO):
    class runner(LeggedRobotCfgPPO.runner):
        run_name = "bdx_walk"
        experiment_name = "rough_bdx"

    class algorithm(LeggedRobotCfgPPO.algorithm):
        entropy_coef = 0.01
