import time
import numpy as np
import pybullet as p
import pybullet_data
import csv

from src.kinematic_model import robotKinematics
from src.pybullet_debugger import pybulletDebug  
from src.gaitPlanner import trotGait


def rendering(render):
    """Enable/disable rendering"""
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, render)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, render)

def robot_init( dt, body_pos, fixed = False ):
    physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
    # turn off rendering while loading the models
    rendering(0)

    p.setGravity(0,0,-10)
    p.setRealTimeSimulation(0)
    p.setPhysicsEngineParameter(
        fixedTimeStep=dt,
        numSolverIterations=100,
        enableFileCaching=0,
        numSubSteps=1,
        solverResidualThreshold=1e-10,
        erp=1e-1,
        contactERP=0.0,
        frictionERP=0.0,
    )
    # add floor
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.loadURDF("plane.urdf")
    # add robot
    body_id = p.loadURDF("stridebot.urdf", body_pos, useFixedBase=fixed)
    joint_ids = []
    
    #robot properties
    maxVel = 3.703 #rad/s
    for j in range(p.getNumJoints(body_id)):
        p.changeDynamics( body_id, j, lateralFriction=1e-5, linearDamping=0, angularDamping=0)
        p.changeDynamics( body_id, j, maxJointVelocity=maxVel)
        joint_ids.append( p.getJointInfo(body_id, j))
    rendering(1)
    return body_id, joint_ids

def robot_stepsim( body_id, body_pos, body_orn, body2feet ):
    #robot properties
    fr_index, fl_index, br_index, bl_index = 3, 7, 11, 15
    maxForce = 2 #N/m
    
    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    fr_angles, fl_angles, br_angles, bl_angles , body2feet_ = robotKinematics.solve( body_orn , body_pos , body2feet )
    #move movable joints
    for i in range(3):
        p.setJointMotorControl2(body_id, i, p.POSITION_CONTROL, targetPosition = fr_angles[i] , force = maxForce)
        p.setJointMotorControl2(body_id, 4 + i, p.POSITION_CONTROL, targetPosition = fl_angles[i] , force = maxForce)
        p.setJointMotorControl2(body_id, 8 + i, p.POSITION_CONTROL, targetPosition = br_angles[i] , force = maxForce) 
        p.setJointMotorControl2(body_id, 12 + i, p.POSITION_CONTROL, targetPosition = bl_angles[i] , force = maxForce)

    p.stepSimulation()
    
    return body2feet_

def robot_quit():
    p.disconnect()
        
        
if __name__ == '__main__':
    dT = 0.005
    bodyId, jointIds = robot_init( dt = dT, body_pos = [0,0,0.18], fixed = False )
    pybulletDebug = pybulletDebug()
    robotKinematics = robotKinematics()
    trot = trotGait()

    """initial foot position"""
    #foot separation (Ydist = 0.16 -> theta =0) and distance to floor
    Xdist, Ydist, height = 0.18, 0.15, 0.10
    #body frame to foot frame vector
    bodytoFeet0 = np.matrix([[ Xdist/2. , -Ydist/2. , height],
                            [ Xdist/2. ,  Ydist/2. , height],
                            [-Xdist/2. , -Ydist/2. , height],
                            [-Xdist/2. ,  Ydist/2. , height]])

    offset = np.array([0.5 , 0.5 , 0. , 0.]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
    footFR_index, footFL_index, footBR_index, footBL_index = 3, 7, 11, 15
    T = 0.5 #period of time (in seconds) of every step
    
    N_steps=50000
    
    for k_ in range(0,N_steps):
        
        pos , orn , L , angle , Lrot , T , sda, offset= pybulletDebug.cam_and_robotstates(bodyId) # takes input from the user
        
        bodytoFeet = trot.loop( L , angle , Lrot , T , offset , bodytoFeet0 , sda) #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
        
        robot_stepsim( bodyId, pos, orn, bodytoFeet ) #simulates the robot to the target position
        
    robot_quit()
