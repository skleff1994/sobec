#!/usr/bin/env python3
"""
Created on Sat Jun 11 17:42:39 2022

@author: nvilla
"""
import matplotlib.pyplot as plt

import configuration as conf

from bullet_Talos import BulletTalos
from cricket.virtual_talos import VirtualPhysics

# from pyRobotWrapper import PinTalos
# from pyMPC import CrocoWBC
# from pyModelMaker import modeller
import pinocchio as pin
from sobec import RobotDesigner, WBC, HorizonManager, ModelMaker, Flex
import numpy as np

# from time import time


def foot_trajectory(T, time_to_land, translation, trajectory="sine"):
    print(time_to_land)
    """Functions to generate steps."""
    tmax = conf.T1contact
    landing_advance = 3
    takeoff_delay = 9
    z = []
    if trajectory == "sine":
        for t in range(
            time_to_land - landing_advance, time_to_land - landing_advance - T, -1
        ):
            z.append(
                0
                if t < 0 or t > tmax - landing_advance - takeoff_delay
                else (np.sin(t * np.pi / (tmax - landing_advance - takeoff_delay)))
                * 0.05
            )

    else:
        for t in range(
            time_to_land - landing_advance, time_to_land - 2 * landing_advance - T, -1
        ):
            z.append(
                0
                if t < 0 or t > tmax - landing_advance - takeoff_delay
                else (
                    1 - np.cos(2 * t * np.pi / (tmax - landing_advance - takeoff_delay))
                )
                * 0.025
            )

    return [np.array([translation[0], translation[1], move_z]) for move_z in z]


def print_trajectory(ref):
    u = [y.translation for y in ref]
    t = np.array([z[2] for z in u])
    fig = plt.figure()
    ax = fig.gca()
    ax.plot(t)
    ax.set_ylim(0, 0.05)


# ####### CONFIGURATION  ############
# ### RobotWrapper
design_conf = dict(
    urdfPath=conf.modelPath + conf.URDF_SUBPATH,
    srdfPath=conf.modelPath + conf.SRDF_SUBPATH,
    leftFootName=conf.lf_frame_name,
    rightFootName=conf.rf_frame_name,
    robotDescription="",
    controlledJointsNames=[
        "root_joint",
        "leg_left_1_joint",
        "leg_left_2_joint",
        "leg_left_3_joint",
        "leg_left_4_joint",
        "leg_left_5_joint",
        "leg_left_6_joint",
        "leg_right_1_joint",
        "leg_right_2_joint",
        "leg_right_3_joint",
        "leg_right_4_joint",
        "leg_right_5_joint",
        "leg_right_6_joint",
        "torso_1_joint",
        "torso_2_joint",
        "arm_left_1_joint",
        "arm_left_2_joint",
        "arm_left_3_joint",
        "arm_left_4_joint",
        "arm_right_1_joint",
        "arm_right_2_joint",
        "arm_right_3_joint",
        "arm_right_4_joint",
    ],
)
design = RobotDesigner()
design.initialize(design_conf)

# Vector of Formulations
MM_conf = dict(
    timeStep=conf.DT,
    gravity=conf.gravity,
    mu=conf.mu,
    coneBox=conf.cone_box,
    minNforce=conf.minNforce,
    maxNforce=conf.maxNforce,
    comHeight=conf.normal_height,
    omega=conf.omega,
    wFootPlacement=conf.wFootPlacement,
    wStateReg=conf.wStateReg,
    wControlReg=conf.wControlReg,
    wLimit=conf.wLimit,
    wVCoM=conf.wVCoM,
    wWrenchCone=conf.wWrenchCone,
    wFootTrans=conf.wFootTrans,
    wFootXYTrans=conf.wFootXYTrans,
    wFootRot=conf.wFootRot,
    wGroundCol=conf.wGroundCol,
    stateWeights=conf.stateWeights,
    controlWeights=conf.controlWeight,
    th_grad=conf.th_grad,
    th_stop=conf.th_stop,
)

formuler = ModelMaker()
formuler.initialize(MM_conf, design)
all_models = formuler.formulateHorizon(lenght=conf.T)

# Horizon

H_conf = dict(leftFootName=conf.lf_frame_name, rightFootName=conf.rf_frame_name)
horizon = HorizonManager()
horizon.initialize(H_conf, design.get_x0(), all_models, all_models[-1])

# MPC
wbc_conf = dict(
    horizonSteps=conf.preview_steps,
    totalSteps=conf.total_steps,
    T=conf.T,
    TdoubleSupport=conf.T2contact,
    TsingleSupport=conf.T1contact,
    Tstep=conf.Tstep,
    ddpIteration=conf.ddpIteration,
    Dt=conf.DT,
    simu_step=conf.simu_period,
    Nc=conf.Nc,
)

# Flex
flex = Flex()
flex.initialize(
    dict(
        left_stiffness=np.array(conf.H_stiff[:2]),
        right_stiffness=np.array(conf.H_stiff[2:]),
        left_damping=np.array(conf.H_damp[:2]),
        right_damping=np.array(conf.H_damp[2:]),
        dt=conf.simu_period,
        MA_duration=0.01,
        left_hip_indices=np.array([0, 1, 2]),
        right_hip_indices=np.array([6, 7, 8]),
    )
)

mpc = WBC()
mpc.initialize(
    wbc_conf,
    design,
    horizon,
    design.get_q0Complete(),
    design.get_v0Complete(),
    "actuationTask",
)
mpc.generateWalkigCycle(formuler)
mpc.generateStandingCycle(formuler)

if conf.simulator == "bullet":
    device = BulletTalos(conf, design.get_rModelComplete())
    device.initializeJoints(design.get_q0Complete().copy())
    device.showTargetToTrack(mpc.ref_LF_poses[0], mpc.ref_RF_poses[0])
    q_current, v_current = device.measureState()

elif conf.simulator == "pinocchio":
    design.rmodelComplete = design.get_rModelComplete()
    design.rmodelComplete.q0 = design.get_q0Complete()
    design.rmodelComplete.v0 = design.get_v0Complete()

    device = VirtualPhysics(conf, view=True, block_joints=conf.blocked_joints)
    device.initialize(design.rmodelComplete)
    init_state = device.measure_state(device.Cq0, device.Cv0, device.Cv0 * 0)
    q_current = init_state["q"]
    v_current = init_state["dq"]

# ### SIMULATION LOOP ###

for s in range(conf.T_total * conf.Nc):
    #    time.sleep(0.001)
    if mpc.timeToSolveDDP(s):

        landing_LF = mpc.land_LF()[0] if mpc.land_LF() else 2 * mpc.horizon.size()
        landing_RF = mpc.land_RF()[0] if mpc.land_RF() else 2 * mpc.horizon.size()
        LF_refs = foot_trajectory(
            len(mpc.ref_LF_poses),
            landing_LF,
            mpc.ref_LF_poses[0].translation,
            "cosine",
        )
        RF_refs = foot_trajectory(
            len(mpc.ref_RF_poses),
            landing_RF,
            mpc.ref_RF_poses[0].translation,
            "cosine",
        )[len(mpc.ref_LF_poses) - 1]

        #        for t in range(len(mpc.ref_LF_poses)):
        #            mpc.ref_LF_poses[t] = pin.SE3(np.eye(3), LF_refs[t])
        #            mpc.ref_RF_poses[t] = pin.SE3(np.eye(3), RF_refs[t])
        mpc.ref_LF_poses[len(mpc.ref_LF_poses) - 1] = pin.SE3(np.eye(3), LF_refs)
        mpc.ref_RF_poses[len(mpc.ref_LF_poses) - 1] = pin.SE3(np.eye(3), RF_refs)

    #        print_trajectory(mpc.ref_LF_poses)

    mpc.iterate(s, q_current, v_current)
    torques = horizon.currentTorques(mpc.x0)

    if conf.simulator == "bullet":
        device.execute(torques)
        q_current, v_current = device.measureState()
        device.moveMarkers(mpc.ref_LF_poses[0], mpc.ref_RF_poses[0])

    elif conf.simulator == "pinocchio":

        correct_contacts = mpc.horizon.get_contacts(0)
        command = {"tau": torques}
        real_state, _ = device.execute(command, correct_contacts, s)

        qc, dqc = flex.correctEstimatedDeflections(
            torques, real_state["q"][7:], real_state["dq"][6:]
        )

        q_current = np.hstack([real_state["q"][:7], qc])
        v_current = np.hstack([real_state["dq"][:6], dqc])

        # esti_state = flex.correctEstimatedDeflections(
        # torques, q_current[7:], v_current[6:]
        # )

        # q_current = np.hstack([q_current[:7], esti_state[0]])
        # v_current = np.hstack([v_current[:6], esti_state[1]])

    # if s == 0:
    # stop

if conf.simulator == "bullet":
    device.close()
