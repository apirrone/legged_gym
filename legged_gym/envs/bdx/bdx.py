from time import time
import numpy as np
import os

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch
from typing import Tuple, Dict
from legged_gym.envs import LeggedRobot


class Bdx(LeggedRobot):
    def _reward_no_fly(self):
        contacts = self.contact_forces[:, self.feet_indices, 2] > 0.1
        single_contact = torch.sum(1.0 * contacts, dim=1) == 1
        return 1.0 * single_contact

    def _reward_head_behavior(self):
        head_default_pos = self.default_dof_pos[:, -5:]
        head_pos = self.dof_pos[:, -5:]
        head_diff = torch.square(head_pos - head_default_pos)
        return -torch.sum(head_diff, dim=1)
