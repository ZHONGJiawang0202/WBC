import MOROB
import numpy as np
import matplotlib.pyplot as plt


# define the trajet of end effecteur f(t)
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

    # use J+ to calculate the q element
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
            # calculate the current J
            X = (self.robot.forward_kinematics()[-1])[:3, 3]
            J = self.robot.compute_jacobian()
            XX.append(X[0])
            Y.append(X[1])
            Z.append(X[2])
            fX.append(self.task(t)[0])
            fY.append(self.task(t)[1])
            fZ.append(self.task(t)[2])

            dX = self.task(t) - np.concatenate((X, np.array([0, 0, 0])))
            dX[3] = dX[4] = dX[5] = 0;

            J_pseudo_inverse = np.linalg.pinv(J)

            dq = np.dot(J_pseudo_inverse, dX)

            q = q + dq
            self.robot.bouger(q)
            q_trajectory.append(q)

        fig, axs = plt.subplots(2, 2, figsize=(8, 6))

        # figures des reel position et desiré position
        axs[0, 1].plot(t_steps, XX, label='X', color='blue', linestyle='-', marker='o')
        axs[0, 1].plot(t_steps, fX, label='f(t)_X', color='red', linestyle='--', marker='x')
        axs[0, 1].set_yticks(np.arange(-0.25, 1.2, 0.05))  # y轴从-1到1.5，每隔0.5显示一个刻度
        axs[0, 1].set_title('X-POSITION')
        axs[0, 1].set_xlabel('time')
        axs[0, 1].set_ylabel('X-axis')
        axs[0, 1].legend()

        axs[0, 0].plot(t_steps, Y, label='Y', color='blue', linestyle='-', marker='o')
        axs[0, 0].plot(t_steps, fY, label='f(t)_Y', color='red', linestyle='--', marker='x')
        axs[0, 0].set_title('Y-POSITION')
        axs[0, 0].set_xlabel('time')
        axs[0, 0].set_ylabel('Y-axis')
        axs[0, 0].legend()

        axs[1, 1].plot(t_steps, Z, label='Z', color='blue', linestyle='-', marker='o')
        axs[1, 1].plot(t_steps, fZ, label='f(t)_Z', color='red', linestyle='--', marker='x')
        axs[1, 1].set_title('Z-POSITION')
        axs[1, 1].set_xlabel('time')
        axs[1, 1].set_ylabel('Z-axis')
        axs[1, 1].legend()

        plt.tight_layout()

        plt.show()

        return np.array(q_trajectory)

    # q_0 = [0, 0.7853981633974483, 0, -1.5707963267948966, 0, 0.7853981633974483, 0]
    # # print(forward_kinematics(q_0)[-1])
    # print(compute_jacobian(q_0))

