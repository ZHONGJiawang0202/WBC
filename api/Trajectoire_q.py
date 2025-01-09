import MOROB
import numpy as np
import matplotlib.pyplot as plt


# 定义默认任务末端轨迹 f(t)
def g(t):
    position = np.array([
        0.5657,
        0.4 * np.sin(np.pi * t / 4),
        0.214
    ])
    orientation = np.array([np.pi, 0, np.pi])
    return np.concatenate((position, orientation))


class Tache:
    def __init__(self, q_0, T = 15, dt = 0.01, task = g):
        self.t_steps = np.arange(0, T, dt)
        self.q_0 = q_0
        self.task = task
        self.robot = MOROB.LBRiiwa7R800(self.q_0)

    # 伪逆矩阵计算关节轨迹基本项
    def articulaire_trajectory(self):
        t_steps = self.t_steps
        q_trajectory = [self.q_0]
        q = self.q_0
        XX = []
        Y = []
        Z = []
        fX = []
        fY = []
        fZ = []
        for t in t_steps:
            # 计算当前雅可比矩阵
            X = (self.robot.forward_kinematics()[-1])[:3, 3]
            J = self.robot.compute_jacobian()
            XX.append(X[0])
            Y.append(X[1])
            Z.append(X[2])
            fX.append(self.task(t)[0])
            fY.append(self.task(t)[1])
            fZ.append(self.task(t)[2])

            # 计算末端位移
            dX = self.task(t) - np.concatenate((X, np.array([0, 0, 0])))
            dX[3] = dX[4] = dX[5] = 0;

            # 计算伪逆 J^+
            J_pseudo_inverse = np.linalg.pinv(J)

            # 计算关节位移 dq
            dq = np.dot(J_pseudo_inverse, dX)

            # 更新q和机器人实例状态
            q = q + dq
            self.robot.bouger(q)
            q_trajectory.append(q)

        fig, axs = plt.subplots(2, 2, figsize=(8, 6))

        # 绘制X Y Z 和f(t)对比图
        # 绘制第一个数据集
        axs[0, 1].plot(t_steps, XX, label='X', color='blue', linestyle='-', marker='o')
        # 绘制第二个数据集
        axs[0, 1].plot(t_steps, fX, label='f(t)_X', color='red', linestyle='--', marker='x')
        # 添加标题和标签
        axs[0, 1].set_yticks(np.arange(-0.25, 1.2, 0.05))  # y轴从-1到1.5，每隔0.5显示一个刻度
        axs[0, 1].set_title('X-POSITION')
        axs[0, 1].set_xlabel('time')
        axs[0, 1].set_ylabel('X-axis')
        # 显示图例
        axs[0, 1].legend()

        # 绘制第一个数据集
        axs[0, 0].plot(t_steps, Y, label='Y', color='blue', linestyle='-', marker='o')
        # 绘制第二个数据集
        axs[0, 0].plot(t_steps, fY, label='f(t)_Y', color='red', linestyle='--', marker='x')
        # 添加标题和标签
        axs[0, 0].set_title('Y-POSITION')
        axs[0, 0].set_xlabel('time')
        axs[0, 0].set_ylabel('Y-axis')
        # 显示图例
        axs[0, 0].legend()
        # 显示网格

        # 绘制第一个数据集
        axs[1, 1].plot(t_steps, Z, label='Z', color='blue', linestyle='-', marker='o')
        # 绘制第二个数据集
        axs[1, 1].plot(t_steps, fZ, label='f(t)_Z', color='red', linestyle='--', marker='x')
        # 添加标题和标签
        axs[1, 1].set_title('Z-POSITION')
        axs[1, 1].set_xlabel('time')
        axs[1, 1].set_ylabel('Z-axis')
        # 显示图例
        axs[1, 1].legend()
        # 显示网格

        # 调整布局
        plt.tight_layout()

        # 显示结果
        plt.show()

        return np.array(q_trajectory)

    # q_0 = [0, 0.7853981633974483, 0, -1.5707963267948966, 0, 0.7853981633974483, 0]
    # # print(forward_kinematics(q_0)[-1])
    # print(compute_jacobian(q_0))

