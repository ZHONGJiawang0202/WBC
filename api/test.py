# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
import MOROB

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
import time
import numpy as np
import Trajectoire_q

def connect_to_sim():
    """connect to CoppeliaSim"""
    sim.simxFinish(-1)  # close all connections before
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:
        print("successgully connect to CoppeliaSim")
    else:
        raise Exception("false，pls check CoppeliaSim and remote API are active")
    return client_id

class Control:

    def __init__(self, client_id):
        self.client_id = client_id

    # """Get the handles of the robot arm joint(two series)"""
    def get_joint_handles(self):
        joint_handles_1 = []
        joint_handles_2 = []
        for i in range(1, 8):  # every LBR iiwa 7R800 has 7 joints
            _, handle = sim.simxGetObjectHandle(self.client_id, f'LBR_iiwa_7_R800_joint{i}', sim.simx_opmode_blocking)
            joint_handles_1.append(handle)
        for i in range(8, 15):
            _, handle = sim.simxGetObjectHandle(self.client_id, f'LBR_iiwa_7_R800_joint{i}', sim.simx_opmode_blocking)
            joint_handles_2.append(handle)
        return joint_handles_1, joint_handles_2

    # get the positions of end(two)
    def get_position_effecteur(self):
        res_1, end_effector_1 = sim.simxGetObjectHandle(self.client_id, f'LBR_iiwa_7_R800_connection',
                                                        sim.simx_opmode_blocking)
        res_2, end_effector_2 = sim.simxGetObjectHandle(self.client_id, f'LBR_iiwa_7_R800_connection0',
                                                        sim.simx_opmode_blocking)

        if res_1 == sim.simx_return_ok:
            res, position_1 = sim.simxGetObjectPosition(self.client_id, end_effector_1, -1, sim.simx_opmode_blocking)
            if res != sim.simx_return_ok:
                position_1 = 0
        else:
            position_1 = 0
        if position_1 == 0:
            print("Fail to get position of effecteur_1")
        if res_2 == sim.simx_return_ok:
            # get the position of end
            res, position_2 = sim.simxGetObjectPosition(self.client_id, end_effector_2, -1, sim.simx_opmode_blocking)
            if res != sim.simx_return_ok:
                position_2 = 0
        else:
            position_2 = 0
        if position_2 == 0:
            print("Fail to get position of effecteur_2")

        return position_1, position_2

    # """set the joint target position"""
    def set_joint_positions(self, joint_handles, target_positions):
        for i in range(7):
            sim.simxSetJointTargetPosition(self.client_id, joint_handles[i], target_positions[i], sim.simx_opmode_oneshot)

    # """set the velocity""
    def set_joint_velocity(self, joint_handles, target_velocity):
        for i in range(7):
            sim.simxSetJointTargetVelocity(self.client_id, joint_handles[i], target_velocity[i], sim.simx_opmode_streaming)


def main():
    client_id = connect_to_sim()

    # Get the handle of the robot arm joint
    control = Control(client_id)
    joint_handles_1, joint_handles_2 = control.get_joint_handles()
    #Get the handle of the car wheel
    error_code, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    error_code, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)



    #initial position
    q_0 = [0, np.pi / 4, 0, -np.pi/2, 0, np.pi/4, 0]
    T = 50
    dt = 0.01


    # define the trajet of end effecteur
    def f(t):
        position = np.array([
            0.5657 * np.cos(np.pi * t / 4),
            0.5657 * np.sin(np.pi * t / 4),
            0.214
        ])
        orientation = np.array([np.pi, 0, np.pi])
        return np.concatenate((position, orientation))

    tache = Trajectoire_q.Tache(q_0, T, dt, f)
    #trajet of first robot
    q_trajectory = tache.articulaire_trajectory()
    #velocity of the second robot
    targetvel = [np.pi / 40, 0, 0, np.pi / 40, 0, 0, 0]

    control.set_joint_positions(joint_handles_1, q_0)
    time.sleep(2)
    for i in range(int(T/dt)):
        # get the target position
        target_positions = q_trajectory[i]
        # send the target position of q
        control.set_joint_positions(joint_handles_1, target_positions)
        control.set_joint_velocity(joint_handles_2,targetvel)
        time.sleep(0.05)

    # control the small car

    #aller tout droit
    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0.2, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0.2, sim.simx_opmode_streaming)


    time.sleep(5)

        # turn left
    for t in range(30):
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, -0.1, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0.1, sim.simx_opmode_streaming)
        time.sleep(0.1)


    # 断开连接
    sim.simxFinish(client_id)
    print(" disconnection to CoppeliaSim ")

if __name__ == '__main__':
    main()
