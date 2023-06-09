#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  9 17:15:22 2022

@author: nvilla
"""
import example_robot_data
import numpy as np

# ## PATHS

URDF_FILENAME = "talos_reduced_corrected.urdf"
SRDF_FILENAME = "talos.srdf"
SRDF_SUBPATH = "/talos_data/srdf/" + SRDF_FILENAME
URDF_SUBPATH = "/talos_data/robots/" + URDF_FILENAME
modelPath = example_robot_data.getModelPath(URDF_SUBPATH)

# ## Joint settings

blocked_joints = [
    "universe",
    # "arm_left_1_joint",
    # "arm_left_2_joint",
    # "arm_left_3_joint",
    # "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
    # "arm_right_1_joint",
    # "arm_right_2_joint",
    # "arm_right_3_joint",
    # "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
    "gripper_left_joint",
    "gripper_right_joint",
    "head_1_joint",
    "head_2_joint",
]

# ## TIMING
total_steps = 8
T_total = 2000  # Total number of nodes of the simulation
DT = 1e-2  # Time step of the DDP
T = 100  # Time horizon of the DDP (number of nodes)
T2contact = 50  # Double support time  # TODO: (check with 20)
simu_step = simu_period = 1e-3  #

# TODO: landing_advance and takeoff_delay are missing

T1contact = 100  # Single support time
ddpIteration = 1  # Number of DDP iterations

preview_steps = 2
# ## PHYSICS

simulator = (
    # "bullet"
    "pinocchio"
)

gravity = np.array([0, 0, -9.81])

mu = 0.1
cone_box = np.array([0.1, 0.05])
minNforce = 200
maxNforce = 1200  # This may be still too low

planned_push = [[(0, 10000 * simu_period)], [np.zeros(6)], ["base_link"]]

model_name = "talos_flex"  #

# Flexibility Parameters
compensate_deflections = True
exact_deflection = False

if model_name == "talos_flex":
    H_stiff = [2200, 2200, 5000, 5000]  # [LH_pitch, LH_roll, RH_pitch, RH_roll]
    H_damp = 2 * np.sqrt(H_stiff)

    # Number of times that the flexibility is computed in each control period
    flex_ratio = round(4.5 + 8.5e-4 * (max(H_stiff) ** 2) / 10000)

elif model_name == "talos":
    H_stiff = [np.inf, np.inf, np.inf, np.inf]  # [LH_pitch, LH_roll, RH_pitch, RH_roll]
    H_damp = [np.inf, np.inf, np.inf, np.inf]
    flex_ratio = 1

flex_esti_delay = 0.0  # [s]
flex_error = 0.0  # error fraction such that: estimation = real*(1-flex_error)

flex_esti_delay = 0.0  # [s]
flex_error = 0.0  # error fraction such that: estimation = real*(1-flex_error)


# ## WALKING GEOMETRY
xForward = 0.15  # step size
foot_height = 0.03  # foot height
TFootDepth = 220  # Foot depth in ground (#TODO: what is this?)
yCorrection = 0.0  # 0.005 # Correction in y to push the feet away from each other

Nc = int(DT / 1e-3)  # Number of control knots per planification timestep
Tstep = T1contact + T2contact

normal_height = 0.87
omega = -normal_height / gravity[2]


# ## CROCO - CONFIGURATION

# relevant frame names

rightFoot = rf_frame_name = "leg_right_sole_fix_joint"
leftFoot = lf_frame_name = "leg_left_sole_fix_joint"


# Weights for all costs

wFootPlacement = 1000
wStateReg = 0.1
wControlReg = 0.001
wLimit = 1e3
wVCoM = 0
wWrenchCone = 0.05

wFootTrans = wFootPlacement  # This can be removed
wFootXYTrans = 0  # This can be removed
wFootRot = 100  # This can be removed
wGroundCol = 0.05  # This can be removed

weightBasePos = [0, 0, 0, 100000, 100000, 100]  # [0, 0, 0, 100000, 100000, 100000]
weightBaseVel = [0, 0, 0, 1000, 1000, 10]  # [0, 0, 0, 1000, 1000, 1000]
weightLegPos = [100, 100, 100, 10000, 100, 100]
weightLegVel = [1000, 1000, 1000, 1000, 1000, 1000]
weightArmRightPos = [10000, 10000, 10000, 10000]  # ,100,100,100 for 3 last arm joint
weightArmRightVel = [1000, 1000, 1000, 1000]  # ,100,100,100 for 3 last arm joint
weightArmLeftPos = [10000, 10000, 10000, 10000]  # ,100,100,100 for 3 last arm joint
weightArmLeftVel = [100, 100, 100, 100]  # ,100,100,100 for 3 last arm joint
weightTorsoPos = [500, 500]
weightTorsoVel = [500, 500]
stateWeights = np.array(
    weightBasePos
    + weightLegPos * 2
    + weightTorsoPos
    + weightArmLeftPos
    + weightArmRightPos
    + weightBaseVel
    + weightLegVel * 2
    + weightTorsoVel
    + weightArmLeftVel
    + weightArmRightVel
)

weightuBase = [0, 0, 0, 0, 0, 0]
weightuLeg = [1, 1, 1, 1, 1, 1]
weightuArm = [10, 10, 10, 10]
weightuTorso = [1, 1]
controlWeight = np.array(weightuLeg * 2 + weightuTorso + weightuArm * 2)

th_stop = 1e-6  # threshold for stopping criterion
th_grad = 1e-9  # threshold for zero gradient.
