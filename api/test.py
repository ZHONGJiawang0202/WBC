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
    """连接到 CoppeliaSim"""
    sim.simxFinish(-1)  # 关闭所有以前的连接
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if client_id != -1:
        print("成功连接到 CoppeliaSim")
    else:
        raise Exception("连接失败，请检查 CoppeliaSim 是否启动以及远程API是否启用")
    return client_id

class Control:

    def __init__(self, client_id):
        self.client_id = client_id

    # """获取机械臂所有关节的句柄(两套)"""
    def get_joint_handles(self):
        joint_handles_1 = []
        joint_handles_2 = []
        for i in range(1, 8):  # LBR iiwa 7R800 有 7 个关节
            _, handle = sim.simxGetObjectHandle(self.client_id, f'LBR_iiwa_7_R800_joint{i}', sim.simx_opmode_blocking)
            joint_handles_1.append(handle)
        for i in range(8, 15):  # LBR iiwa 7R800 有 7 个关节
            _, handle = sim.simxGetObjectHandle(self.client_id, f'LBR_iiwa_7_R800_joint{i}', sim.simx_opmode_blocking)
            joint_handles_2.append(handle)
        return joint_handles_1, joint_handles_2

    # 获取机器人的末端执行器的位置(两个）
    def get_position_effecteur(self):
        # 获取机器人的末端执行器的句柄
        res_1, end_effector_1 = sim.simxGetObjectHandle(self.client_id, f'LBR_iiwa_7_R800_connection',
                                                        sim.simx_opmode_blocking)
        res_2, end_effector_2 = sim.simxGetObjectHandle(self.client_id, f'LBR_iiwa_7_R800_connection0',
                                                        sim.simx_opmode_blocking)

        if res_1 == sim.simx_return_ok:
            # 获取末端执行器的位置
            res, position_1 = sim.simxGetObjectPosition(self.client_id, end_effector_1, -1, sim.simx_opmode_blocking)
            if res != sim.simx_return_ok:
                position_1 = 0
        else:
            position_1 = 0
        if position_1 == 0:
            print("Fail to get position of effecteur_1")
        if res_2 == sim.simx_return_ok:
            # 获取末端执行器的位置
            res, position_2 = sim.simxGetObjectPosition(self.client_id, end_effector_2, -1, sim.simx_opmode_blocking)
            if res != sim.simx_return_ok:
                position_2 = 0
        else:
            position_2 = 0
        if position_2 == 0:
            print("Fail to get position of effecteur_2")

        return position_1, position_2

    # """设置机械臂关节的目标位置"""
    def set_joint_positions(self, joint_handles, target_positions):
        for i in range(7):
            sim.simxSetJointTargetPosition(self.client_id, joint_handles[i], target_positions[i], sim.simx_opmode_oneshot)

    # """设置机械臂关节的目标速度""
    def set_joint_velocity(self, joint_handles, target_velocity):
        for i in range(7):
            # 设置目标速度（正数为正方向，负数为负方向）
            sim.simxSetJointTargetVelocity(self.client_id, joint_handles[i], target_velocity[i], sim.simx_opmode_streaming)


def main():
    # 连接到 CoppeliaSim
    client_id = connect_to_sim()

    # 获取机械臂关节句柄
    control = Control(client_id)
    joint_handles_1, joint_handles_2 = control.get_joint_handles()
    #获取小车轮子句柄
    error_code, left_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
    error_code, right_motor_handle = sim.simxGetObjectHandle(client_id, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)



    # # 初始化关节角度（弧度制）
    q_0 = [0, np.pi / 4, 0, -np.pi/2, 0, np.pi/4, 0]
    # 仿真时间参数
    T = 50 # 总时间
    dt = 0.01  # 时间步长


    # 计算机械手1轨迹和机械手2速度
    def f(t):
        position = np.array([
            0.5657 * np.cos(np.pi * t / 4),
            0.5657 * np.sin(np.pi * t / 4),
            0.214
        ])
        orientation = np.array([np.pi, 0, np.pi])
        return np.concatenate((position, orientation))

    tache = Trajectoire_q.Tache(q_0, T, dt, f)
    q_trajectory = tache.articulaire_trajectory()
    targetvel = [np.pi / 40, 0, 0, np.pi / 40, 0, 0, 0]

    control.set_joint_positions(joint_handles_1, q_0)
    time.sleep(2)
    for i in range(int(T/dt)):
        # 设置目标关节角度 (单位: 弧度)
        target_positions = q_trajectory[i]
        # 发送目标角度
        control.set_joint_positions(joint_handles_1, target_positions)
        control.set_joint_velocity(joint_handles_2,targetvel)
        time.sleep(0.05)
        # # 等待一段时间以查看运动效果
        # time.sleep(dt)

    sim.simxSetJointTargetVelocity(client_id, left_motor_handle, 0.2, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0.2, sim.simx_opmode_streaming)

        # 等待 5 秒
    time.sleep(5)

        # 让小车左转
    for t in range(30):  # 让小车旋转3秒钟
        sim.simxSetJointTargetVelocity(client_id, left_motor_handle, -0.1, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(client_id, right_motor_handle, 0.1, sim.simx_opmode_streaming)
        time.sleep(0.1)


    # 断开连接
    sim.simxFinish(client_id)
    print("已断开与 CoppeliaSim 的连接")

if __name__ == '__main__':
    main()
